#include "solver/solver.hpp"
#include "solver_origin.cpp"
#include "solver_march.cpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace vbd {

template <typename T> auto durationInMicroseconds(T start, T end) {
  return std::chrono::duration_cast<std::chrono::microseconds>(end - start)
      .count();
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
Solver::Solver(Environment &env)
    : sharedConfig_(env.getConfig()),
      sharedOccupancyField_(env.getOccupancyField()) {
  nx_ = sharedOccupancyField_->nx();
  ny_ = sharedOccupancyField_->ny();

  // copy the relavant marchControl parameters
  max_nb_of_iter_ = sharedConfig_->max_nb_of_iterations;
  marchStepSize_ = 1;
  colorPivots_ = sharedConfig_->colorPivots;
  colorBoundaries_ = sharedConfig_->colorBoundaries;
  colorCameFrom_ = sharedConfig_->colorCameFrom;
  showContour_ = sharedConfig_->contourLines;
  findNode_ = false;

  // Init environment image
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  uniqueLoadedImage_->create(nx_, ny_, sf::Color::Black);
  sf::Color color;
  color.a = 1;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (sharedOccupancyField_->get(i, j) == 1) {
        uniqueLoadedImage_->setPixel(i, j, color.Black);
      } else {
        uniqueLoadedImage_->setPixel(i, j, color.White);
      }
    }
  }
  // Init maps
  reset();
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::reset() {
  nullPoint_ = point(0, ny_+1);
  nullidx_ = indexAt(0, ny_+1);
  endPoint_ = nullPoint_;
  // reset the fields
  gScore_.reset(nx_, ny_, infinity);
  cameFrom_.reset(nx_, ny_, nullPoint_);
  pivotDir_.reset(nx_, ny_, {cardir::None, cardir::None});
  slopeOrigin_.reset(nx_, ny_, nullidx_);
  blockCorners_.reset(nx_, ny_, std::pair(nullidx_, nullidx_));

  openSet_.reset();
  pivots_.reset(new point[nx_ * ny_]);

  // Reserve heaps
  std::vector<Node> container;
  container.reserve(nx_ * ny_);
  std::priority_queue<Node, std::vector<Node>, std::less<Node>> heap(
      std::less<Node>(), std::move(container));
  openSet_ = std::make_unique<std::priority_queue<Node>>(heap);

  // reset counters
  nb_of_iterations_ = 0;
  nb_of_marches_ = 0;
  nb_of_pivots_ = 0;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::visibilityBasedSolver() {
  if (sharedConfig_->silent) {
        std::cout << "###################### VBD solver output "
                 "######################"
              << std::endl;
  }
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();
  
  auto &initial_frontline = sharedConfig_->initialFrontline;

  // verify that the initial frontline is valid
  if (initial_frontline.size() % 2 != 0) {
    std::cout << "###################### Visibility-based solver output "
                "######################"
              << std::endl;
    std::cout << "Initial frontline must be of size that is a multiple of 2 "
                "for visibility-based solver"
              << std::endl;
    return;
  }
  else if (sharedConfig_->originSolver && initial_frontline.size() != 2) {
      std::cout << "###################### Visibility-based solver output "
                  "######################"
                << std::endl;
      std::cout << "Initial frontline can only contain one point for the use of the origin solver"
                << std::endl;
      return;
  }

  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    int x = initial_frontline[i];
    int y = ny_ - 1 - initial_frontline[i + 1];
    // check if starting positions are inside the map
    if (x >= nx_ || y >= ny_ || x < 0 || y < 0) {
      std::cout << "###################### Visibility-based solver output "
                  "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is outside the map"
                << std::endl;
      return;
    }

    if (sharedOccupancyField_->get(x, y) == 1) {
      std::cout << "###################### Visibility-based solver output "
                  "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is invalid/occupied" << x << " " << y
                << std::endl;
      return;
    }
  }

  // Origin Visiblity
  if (sharedConfig_->originSolver) {
    x_ = initial_frontline[0];
    y_ = ny_ - 1 - initial_frontline[1];
    startPoint_ = point(x_, y_);
    gScore_(x_, y_) = 0; 
    cameFrom_(x_, y_) = startPoint_;
    ComputeOriginVisibility();
  }
  else {
    for (size_t i = 0; i < initial_frontline.size(); i += 2) {
      int x = initial_frontline[i];
      int y = ny_ - 1 - initial_frontline[i + 1];
      gScore_(x, y) = 0; 
      cameFrom_(x, y) = {x, y};
      openSet_->push(Node(0, 0, x, y, cardir::North, cardir::East, 0, 0, 0, false ));
      openSet_->push(Node(1, 0, x, y, cardir::North, cardir::East, 0, 0, 0, false ));
      openSet_->push(Node(0, 0, x, y, cardir::East, cardir::South, 0, 0, 0, false ));
      openSet_->push(Node(1, 0, x, y, cardir::East, cardir::South, 0, 0, 0, false ));
      openSet_->push(Node(0, 0, x, y, cardir::South, cardir::West, 0, 0, 0, false ));
      openSet_->push(Node(1, 0, x, y, cardir::South, cardir::West, 0, 0, 0, false ));
      openSet_->push(Node(0, 0, x, y, cardir::West, cardir::North, 0, 0, 0, false ));
      openSet_->push(Node(1, 0, x, y, cardir::West, cardir::North, 0, 0, 0, false ));
    }
  }
  // Expand search from closest march point
  while(openSet_->size() > 0) {
    if (nb_of_iterations_ >= max_nb_of_iter_) {
      if (!sharedConfig_->marchControl) {
        std::cout << "max iterations reached\n";
      }
      break;
    }
    nb_of_iterations_++;
    Node curNode = std::move(openSet_->top());
    openSet_->pop();
    if (findNode_ && curNode.x == target_x_ && curNode.y == target_y_) {
      if (nbOfVisits_ == 1) {
        findNode_ = false;
        max_nb_of_iter_ = nb_of_iterations_;
      }
      else {
        nbOfVisits_--;
      }
    }
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      switch (curNode.type) {
        case 0:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on secondary march\n";
          break;
        case 1:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on straight primary\n";
          break;
        case 2:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on primary pivot slope\n";
          break;
        case 3:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on primary parent slope\n";
          break;
        case 4:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on occupied point\n";
          break;
        case 5:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on switched search\n";
          break;
        case 6:
          std::cout << "Node (" << curNode.x << ", " << ny_-1-curNode.y << ") on pivot creation\n";
          break;
        default:
          break;
      }
      endPoint_ = point(curNode.x, curNode.y);
    }
    // secondary search
    if (curNode.type == 0) {
      if (advanceSecondaryNode(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, curNode.state)) {
        openSet_->push(curNode);
      }
    }
    // primary searches
    else if (curNode.type <= 3) { 
      // straight primary
      if (curNode.type == 1) {
        addNextStraightPrimary(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.state);
        if (advanceSecondaryNode(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, false, curNode.state)) {
          curNode.type = 0;
          curNode.state = false;
          openSet_->push(curNode);
        }
      }
      // pivot slope primary
      else if (curNode.type == 2) {
        if (addNextSlopePrimary(curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, curNode.state, false)) {
          curNode.state = true;
          if (advanceSecondaryNode(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, curNode.state)) {
            curNode.type = 0;
            openSet_->push(curNode);
          }
        }
      }
      // parent slope primary
      else {
        if (addNextSlopePrimary(curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, curNode.state, true)) {
          curNode.state = false;
          if (advanceSecondaryNode(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, curNode.state)) {
            curNode.type = 0;
            openSet_->push(curNode);
          }
        }
      }
    }
    // occupied scan
    else if (curNode.type == 4) {
      if (advanceOccupiedNode(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist)) {
        openSet_->push(curNode);
      }
    }
    // switched pivot 
    else if (curNode.type == 5) {
      if (addNextSwitchedPrimary(curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.state)) {
        curNode.state = false;
        if (advanceSecondaryNode(curNode.distance, curNode.x, curNode.y, curNode.primaryDir, curNode.secondaryDir, curNode.primaryDist, curNode.secondaryDist, curNode.slope, curNode.state, true, false)) {
          curNode.type = 0;
          openSet_->push(curNode);
        }
      }
    }
    // pivot creation
    else if (curNode.type == 6) {
      createNewPivot({curNode.x, curNode.y}, cameFrom_(curNode.x, curNode.y), curNode.primaryDir, curNode.secondaryDir, curNode.slope, curNode.state);
    }
    // invalid type
    else {
      std::cout << "Error: Invalid node type\n";
    }
  }
  max_nb_of_iter_ = nb_of_iterations_;

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent && sharedConfig_->timer) {
    std::cout << "Execution time in us: " << executionDuration << std::endl;
    std::cout << "Nb of iterations: " << nb_of_iterations_ << std::endl;
    std::cout << "Nb of marches: " << nb_of_marches_ << std::endl;
    std::cout << "Nb of sources: " << nb_of_pivots_ << std::endl;
  }
  if (sharedConfig_->saveVisibilityBasedSolverImage) {
    saveVisibilityBasedSolverImage(gScore_);
  }
  if (sharedConfig_->saveResults) {
    saveResults({});
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
sf::Color getColor(double value) {
  // jet colormap for SFML visualization/plot
  const int color_index = 255 * value;
  double r, g, b;
  if (color_index < 32) {
    r = 0;
    g = 0;
    b = 0.5156 + 0.0156 * color_index;
  } else if (color_index < 96) {
    r = 0;
    g = 0.0156 + 0.9844 * (color_index - 32.0) / 64;
    b = 1;
  } else if (color_index < 158) {
    r = 0.0156 + (color_index - 96.0) / 64;
    g = 1;
    b = 0.9844 - (color_index - 96.0) / 64;
  } else if (color_index < 223) {
    r = 1;
    g = 1 - (color_index - 158.0) / 65;
    b = 0;
  } else {
    r = (2 - (color_index - 223.0) / 32) / 2.0;
    g = 0;
    b = 0;
  }
  return sf::Color(static_cast<sf::Uint8>(r * 255),
                   static_cast<sf::Uint8>(g * 255),
                   static_cast<sf::Uint8>(b * 255));
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
sf::Color getPivotColor(int x, int y, int size) {
  double value = (double)(x % size + (y % size)*size) / (size * size);
  // Cubehelix colormap for visulization of pivots
  double start = 0.5, rotations = -1.5, hue = 1.0, gamma = 1.0;
  // Compute the angle and adjusted value
  double angle = 2.0 * 3.14159265 * (start / 3.0 + rotations * value);
  value = 0.2 + 0.55 * value;
  value = pow(value, gamma);
  // Compute the amplitude
  double amp = hue * value * (1.0 - value) / 2.0;
  // Compute the RGB components
  double r = value + amp * (-0.14861 * cos(angle) + 1.78277 * sin(angle));
  double g = value + amp * (-0.29227 * cos(angle) - 0.90649 * sin(angle));
  double b = value + amp * (1.97294 * cos(angle));
  // Convert to sf::Color, ensuring the values are within [0, 255]
  return sf::Color(static_cast<sf::Uint8>(r * 255),
                    static_cast<sf::Uint8>(g * 255),
                    static_cast<sf::Uint8>(b * 255));
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::saveVisibilityBasedSolverImage(const Field<double> &gScore) const {
  const int width = nx_;
  const int height = ny_;
  sf::Image image;
  image.create(width, height);
  double maxDist = std::numeric_limits<double>::min();

// Find max values in gScore for normalization
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      double val = gScore_(i, j);
      if (val == infinity)
        continue;
      if (val > maxDist)
        maxDist = val;
    }
  }
// Generate the image based on the gScore
  if (!colorCameFrom_) {
    
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        if (sharedOccupancyField_->get(i, j) == 1) {
          image.setPixel(i, j, sf::Color(128,128,128));
        } else {
          const double normalized_value = gScore(i, j) / maxDist;
          sf::Color color = getColor(normalized_value);
          image.setPixel(i, j, color);
        }
      }
    }
  }
// Generate the image based on the cameFrom field
  else {
    // Generate the image
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        if (sharedOccupancyField_->get(i, j) == 1) {
          image.setPixel(i, j, sf::Color(128,128,128));
        } else {
          point pivot = cameFrom_(i, j);
          if (pivot == nullPoint_) {
            image.setPixel(i, j, sf::Color::Black);
          }
          else {
            sf::Color color = getPivotColor(pivot.first, pivot.second, sharedConfig_->pivotColorGridSize);
            image.setPixel(i, j, color);
          }
        }
      }
    }
  }
// color the contour lines in black
  if (showContour_) {
  // compute the step size based on the max and min values
  int number_of_contour_lines = sharedConfig_->number_of_contour_lines;
  double stepSize = maxDist / number_of_contour_lines;

  std::vector<double> contourLevels;
  for (double level = 0; level <= maxDist; level += stepSize) {
    contourLevels.push_back(level);
  }
  // Draw contour lines on the image
  for (double level : contourLevels) {
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        double value = gScore(i, j);
        if (std::abs(value - level) <= stepSize / 15) {
          image.setPixel(i, j, sf::Color::Black);
        }
      }
    }
  }
}
// color all initial frontline points as white circle
  int radius = nx_ / 120;
  int x0, y0;
  for (size_t k = 0; k < sharedConfig_->initialFrontline.size(); k += 2) {
    x0 = sharedConfig_->initialFrontline[k];
    y0 = ny_ - 1 - sharedConfig_->initialFrontline[k + 1];
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        if (i * i + j * j <= radius * radius) {
          if (x0 + i >= 0 && x0 + i < nx_ && y0 + j >= 0 && y0 + j < ny_) {
            image.setPixel(x0 + i, y0 + j, sf::Color::White);
          }
        }
      }
    }
  }
// plot pivots with cross;
  if (colorPivots_) {
    for (size_t idx = 0; idx < nb_of_pivots_; ++idx) {
      sf::Color pivot_color = getPivotColor(pivots_[idx].first, pivots_[idx].second, sharedConfig_->pivotColorGridSize);
      for (int i = -radius; i <= radius; ++i) {
        for (int j = -radius; j <= radius; ++j) {
          if (i * i + j * j <= radius * radius) {
            if (pivots_[idx].first + i >= 0 && pivots_[idx].first + i < nx_ && pivots_[idx].second + j >= 0 && pivots_[idx].second + j < ny_) {
              image.setPixel(pivots_[idx].first + i, pivots_[idx].second + j, sf::Color::Magenta);
            }
          }
        }
      }
    }
  }
// plot boundary lines
  if (colorBoundaries_) {
    for (int i = 0; i < width; ++i) {
      for (int j = 0; j < height; ++j) {
        std::pair<size_t, size_t> blockPair = blockCorners_(i, j);
        if (blockPair.first != nullidx_) {
          sf::Color color = getColor(blockPair.first / (1.0*nx_ * ny_));
          image.setPixel(i, j, color);
        }
        else if (blockPair.second != nullidx_) {
          sf::Color color = getColor(blockPair.second / (1.0*nx_ * ny_));
          image.setPixel(i, j, color);
        }
      }
    }
  }
// color the focusPoint
  if (endPoint_ != nullPoint_) {
    image.setPixel(endPoint_.first, endPoint_.second, sf::Color::White);
  }
// Save the image
  if (!sharedConfig_->silent) {
    std::cout << "Saving visibility-based solver image" << std::endl;
  }
  std::string outputPath = "./output/VBD_solver.png";
  image.saveToFile(outputPath);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::saveResults(const std::vector<point> &resultingPath) const {
  namespace fs = std::filesystem;
  const std::string methodName = "VBD";
  // Define the path to the output file
  std::string outputFilePath = "./output/" + methodName + ".txt";

  // Check if the directory exists, and create it if it doesn't
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string()
                << std::endl;
      return;
    }
  }

  // Save gScore_
  outputFilePath = "./output/" + methodName + "_gScore.txt";
  if (sharedConfig_->savegScore) {
    std::fstream of1(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of1.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of1;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << gScore_(i, j) << " ";
      }
      os << "\n";
    }
    of1.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " gScore" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }

  // Save Pivots
  outputFilePath = "./output/" + methodName + "_pivots.txt";
  if (sharedConfig_->savePivots) {
    std::fstream of3(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of3.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of3;
    for (size_t i = 0; i < nb_of_pivots_; ++i) {
      os << pivots_[i].first << " " << ny_ - 1 - pivots_[i].second << "\n";
    }
    of3.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " pivots" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }

  // Save CameFrom
  outputFilePath = "./output/" + methodName + "_cameFrom.txt";
  if (sharedConfig_->saveCameFrom) { 
    std::fstream of1(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of1.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath
                << std::endl;
      return;
    }
    std::ostream &os1 = of1;
    for (int j = 0; j < ny_; ++j) {
      for (size_t i = 0; i < nx_; ++i) {
        os1 << indexAt(cameFrom_(i, j).first, cameFrom_(i, j).second) << " ";
      }
      os1 << "\n";
    }
    of1.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " cameFrom_" << std::endl;
    }
  } else {
    outputFilePath = "./output/" + methodName + "_cameFrom.txt";
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
} // namespace vbd