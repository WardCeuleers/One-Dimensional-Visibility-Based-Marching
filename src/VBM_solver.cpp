#include "solver/VBM_solver.hpp"

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
sf::Color VBM_getColor(double value) {
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
VBM_Solver::VBM_Solver(Environment &env)
    : sharedConfig_(env.getConfig()),
      sharedVisibilityField_(env.getVisibilityField()),
      sharedSpeedField_(env.getSpeedField()) {
  nx_ = sharedVisibilityField_->nx();
  ny_ = sharedVisibilityField_->ny();
  visibilityThreshold_ = sharedConfig_->visibilityThreshold;

  // Init environment image
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  uniqueLoadedImage_->create(nx_, ny_, sf::Color::Black);
  sf::Color color;
  color.a = 1;
  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      if (sharedVisibilityField_->get(i, j) < 1) {
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
void VBM_Solver::reset() {
  gScore_.reset(nx_, ny_, std::numeric_limits<double>::infinity());
  fScore_.reset(nx_, ny_, std::numeric_limits<double>::infinity());
  cameFrom_.reset(nx_, ny_, 0);
  inOpenSet_.reset(nx_, ny_, false);
  updated_.reset(nx_, ny_, false);

  lightSources_.reset(new VBM_point[nx_ * ny_]);

  visibilityHashMap_.clear();
  openSet_.reset();

  // Reserve openSet_
  std::vector<VBM_Node> container;
  container.reserve(nx_ * ny_);
  std::priority_queue<VBM_Node, std::vector<VBM_Node>, std::less<VBM_Node>> heap(
      std::less<VBM_Node>(), std::move(container));
  openSet_ = std::make_unique<std::priority_queue<VBM_Node>>(heap);

  nb_of_sources_ = 0;
  nb_of_iterations_ = 0;

  // Reserve hash map
  visibilityHashMap_.reserve(nx_ * ny_);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::visibilityBasedSolver() {
  reset();
  auto startTime = std::chrono::high_resolution_clock::now();

// Init
  double d = 0;
  int x = 0, y = 0, neighbour_x = 0, neighbour_y = 0;

  auto &initial_frontline = sharedConfig_->initialFrontline;

  if (initial_frontline.size() % 2 != 0) {
    std::cout << "###################### Visibility-based solver output "
                 "######################"
              << std::endl;
    std::cout << "Initial frontline must be of size that is a multiple of 2 "
                 "for visibility-based solver"
              << std::endl;
    return;
  }
  for (size_t i = 0; i < initial_frontline.size(); i += 2) {
    x = initial_frontline[i];
    y = ny_ - 1 - initial_frontline[i + 1];
    // check if starting positions are inside the map
    if (x >= nx_ || y >= ny_) {
      std::cout << "###################### Visibility-based solver output "
                   "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is outside the map"
                << std::endl;
      return;
    }

    if (sharedVisibilityField_->get(x, y) < 1) {
      std::cout << "###################### Visibility-based solver output "
                   "######################"
                << std::endl;
      std::cout << "At least one of the starting positions is invalid/occupied"
                << std::endl;
      return;
    }

    d = 0;
    gScore_(x, y) = d;
    updated_(x, y) = true;
    cameFrom_(x, y) = nb_of_sources_;
    lightSources_[nb_of_sources_] = {x, y};

    openSet_->push(VBM_Node{x, y, d});
    const auto key = hashFunction(x, y, nb_of_sources_);
    visibilityHashMap_[key] = lightStrength_;
    ++nb_of_sources_;
    ++nb_of_iterations_;
  }

  // For queing unique sources from neighbours of neighbour
  std::vector<size_t> potentialSources;
  potentialSources.reserve(10);
  std::vector<std::pair<double, size_t>> potentialDistances;
  potentialDistances.reserve(10);

  double distance = 0;

  while (openSet_->size() > 0) {
    auto &current = openSet_->top();
    x = current.x;
    y = current.y;
    openSet_->pop();

    // Expand frontline at current & update neighbours
    for (size_t j = 0; j < 16; j += 2) {
      // NOTE those are always positive
      neighbour_x = x + neighbours_[j];
      neighbour_y = y + neighbours_[j + 1];

      // Box check
      if (!isValid(neighbour_x, neighbour_y)) {
        continue;
      }
      if (updated_(neighbour_x, neighbour_y)) {
        continue;
      };
      if (sharedVisibilityField_->get(neighbour_x, neighbour_y) < 1) {
        cameFrom_(neighbour_x, neighbour_y) = cameFrom_(x, y);
        updated_(neighbour_x, neighbour_y) = true;
        continue;
      }

      queuePotentialSources(potentialSources, neighbour_x, neighbour_y);
      getPotentialDistancesSpeedField(potentialSources, potentialDistances,
                                      neighbour_x, neighbour_y);

      auto minimum_element =
          std::min_element(potentialDistances.begin(), potentialDistances.end(),
                           [](const auto &lhs, const auto &rhs) {
                             return lhs.first < rhs.first;
                           });
      distance = minimum_element->first;

      if (distance == std::numeric_limits<double>::infinity()) {
        createNewPivot(x, y, neighbour_x, neighbour_y);
      } else {
        // use source giving least distance
        gScore_(neighbour_x, neighbour_y) = minimum_element->first;
        cameFrom_(neighbour_x, neighbour_y) = minimum_element->second;
      }
      openSet_->push(
          VBM_Node{neighbour_x, neighbour_y, gScore_(neighbour_x, neighbour_y)});
      updated_(neighbour_x, neighbour_y) = true;
      ++nb_of_iterations_;
    }
  };

  auto stopTime = std::chrono::high_resolution_clock::now();
  auto executionDuration = durationInMicroseconds(startTime, stopTime);

  if (!sharedConfig_->silent && sharedConfig_->timer) {
      std::cout << "###################### VBM solver output "
                 "######################" << std::endl;
      std::cout << "Execution time in us: " << executionDuration << std::endl;
      std::cout << "Load factor: " << visibilityHashMap_.load_factor()
                << std::endl;
      std::cout << "Iterations: " << nb_of_iterations_ << std::endl;
      std::cout << "Nb of sources: " << nb_of_sources_ << std::endl;
  }
  if (sharedConfig_->saveResults) {
    saveResults({});
  }
  if (sharedConfig_->saveVisibilityBasedSolverImage) {
    saveVisibilityBasedSolverImage(gScore_);
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
inline void VBM_Solver::queuePotentialSources(std::vector<size_t> &potentialSources,
                                          const int neighbour_x,
                                          const int neighbour_y) const {
  size_t potentialSource_x = 0, potentialSource_y = 0, lightSource_num = 0;
  potentialSources.clear();
  // Queue sources from updated neighbours of neighbour
  for (size_t k = 0; k < 16; k += 2) {
    // NOTE those are always positive
    potentialSource_x = neighbour_x + neighbours_[k];
    potentialSource_y = neighbour_y + neighbours_[k + 1];
    // Box check
    if (!isValid(potentialSource_x, potentialSource_y)) {
      continue;
    };
    if (!updated_(potentialSource_x, potentialSource_y)) {
      continue;
    };

    lightSource_num = cameFrom_(potentialSource_x, potentialSource_y);
    // Pick only unique sources (no repitition in potentialSources)
    if (std::find(potentialSources.begin(), potentialSources.end(),
                  lightSource_num) == potentialSources.end()) {
      potentialSources.push_back(lightSource_num);
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::getPotentialDistances(
    const std::vector<size_t> &potentialSources,
    std::vector<std::pair<double, size_t>> &potentialDistances,
    const int neighbour_x, const int neighbour_y) {
  size_t LS_x = 0, LS_y = 0, potentialSource = 0;
  double distance = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    LS_x = lightSources_[potentialSource].x;
    LS_y = lightSources_[potentialSource].y;
    // update visibility from source
    updatePointVisibility(potentialSource, LS_x, LS_y, neighbour_x,
                          neighbour_y);
    distance = INFINITY;
    const auto key = hashFunction(neighbour_x, neighbour_y, potentialSource);
    if (visibilityHashMap_.at(key) >= visibilityThreshold_) {
      distance = gScore_(LS_x, LS_y) +
                 evaluateDistance(LS_x, LS_y, neighbour_x, neighbour_y);
    }
    potentialDistances.push_back(
        std::pair<double, int>{distance, potentialSource});
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::getPotentialDistancesSpeedField(
    const std::vector<size_t> &potentialSources,
    std::vector<std::pair<double, size_t>> &potentialDistances,
    const int neighbour_x, const int neighbour_y) {
  size_t LS_x = 0, LS_y = 0, potentialSource = 0;
  double distance = 0;
  potentialDistances.clear();
  for (size_t k = 0; k < potentialSources.size(); ++k) {
    potentialSource = potentialSources[k];
    LS_x = lightSources_[potentialSource].x;
    LS_y = lightSources_[potentialSource].y;
    // update visibility from source
    updatePointVisibility(potentialSource, LS_x, LS_y, neighbour_x,
                          neighbour_y);
    distance = INFINITY;
    const auto key = hashFunction(neighbour_x, neighbour_y, potentialSource);
    if (visibilityHashMap_.at(key) >= visibilityThreshold_) {
      distance =
          gScore_(LS_x, LS_y) +
          evaluateDistanceSpeedField(LS_x, LS_y, neighbour_x, neighbour_y);
    }
    potentialDistances.push_back(
        std::pair<double, int>{distance, potentialSource});
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::createNewPivot(const int x, const int y, const int neighbour_x,
                            const int neighbour_y) {
  int pivot_neighbour_x, pivot_neighbour_y;
  // Pushback parent as a new lightSource
  lightSources_[nb_of_sources_] = {x, y}; // {x, y};
  // Pusback pivot & update light source visibility
  const auto key = hashFunction(x, y, nb_of_sources_);
  visibilityHashMap_[key] = lightStrength_;
  // Update maps of new pivot_
  // Update neighbours of initial frontline points - both distance & visibility
  for (size_t p = 0; p < 16; p += 2) {
    // NOTE those are always positive
    pivot_neighbour_x = x + neighbours_[p];
    pivot_neighbour_y = y + neighbours_[p + 1];
    // Box check
    if (!isValid(pivot_neighbour_x, pivot_neighbour_y)) {
      continue;
    }
    // Update neighbour visibility
    updatePointVisibility(nb_of_sources_, x, y, pivot_neighbour_x,
                          pivot_neighbour_y);
  }
  cameFrom_(neighbour_x, neighbour_y) = nb_of_sources_;
  gScore_(neighbour_x, neighbour_y) =
      gScore_(x, y) + evaluateDistance(x, y, neighbour_x, neighbour_y);
  ++nb_of_sources_;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::updatePointVisibility(const size_t lightSourceNumber,
                                   const int LS_x, const int LS_y, const int x,
                                   const int y) {
  // Variable initialization
  double v = 0;
  double c = 0;

  // Check if visibility value already exists
  auto key = hashFunction(x, y, lightSourceNumber);
  if (visibilityHashMap_.count(key)) {
    return;
  }
  if (sharedVisibilityField_->get(x, y) < visibilityThreshold_) {
    visibilityHashMap_[key] = 0;
    return;
  }

  if (x == LS_x) {
    if (y - LS_y > 0) {
      key = hashFunction(x, y - 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1);
      }
      v = visibilityHashMap_.at(key);
    } else {
      key = hashFunction(x, y + 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1);
      }
      v = visibilityHashMap_.at(key);
    }
  } else if (y == LS_y) {
    if (x - LS_x > 0) {
      key = hashFunction(x - 1, y, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y);
      }
      v = visibilityHashMap_.at(key);
    } else {
      key = hashFunction(x + 1, y, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y);
      }
      v = visibilityHashMap_.at(key);
    }
  } else {
    // Q1
    if ((x - LS_x > 0) && (y - LS_y > 0)) {
      key = hashFunction(x - 1, y - 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y - 1);
      }
      if (x - LS_x == y - LS_y) {
        v = visibilityHashMap_.at(key);
      } else if (x - LS_x < y - LS_y) {
        const auto key_1 = hashFunction(x, y - 1, lightSourceNumber);
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1);
        }
        c = static_cast<double>(x - LS_x) / (y - LS_y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (x - LS_x > y - LS_y) {
        const auto key_2 = hashFunction(x - 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y);
        }
        c = static_cast<double>(y - LS_y) / (x - LS_x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
    // Q2
    else if ((x - LS_x < 0) && (y - LS_y > 0)) {
      key = hashFunction(x + 1, y - 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y - 1);
      }
      if (LS_x - x == y - LS_y) {
        v = visibilityHashMap_.at(key);
      } else if (LS_x - x < y - LS_y) {
        const auto key_1 = hashFunction(x, y - 1, lightSourceNumber);
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y - 1);
        }
        c = static_cast<double>(LS_x - x) / (y - LS_y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (LS_x - x > y - LS_y) {
        const auto key_2 = hashFunction(x + 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y);
        }
        c = static_cast<double>(y - LS_y) / (LS_x - x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
    // Q3
    else if ((x - LS_x < 0) && (y - LS_y < 0)) {
      key = hashFunction(x + 1, y + 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y + 1);
      }
      if (LS_x - x == LS_y - y) {
        v = visibilityHashMap_.at(key);
      } else if (LS_x - x < LS_y - y) {
        const auto key_1 = (y + 1) + nx_ * x + ny_ * nx_ * lightSourceNumber;
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1);
        }
        c = static_cast<double>(LS_x - x) / (LS_y - y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (LS_x - x > LS_y - y) {
        const auto key_2 = hashFunction(x + 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x + 1, y);
        }
        c = static_cast<double>(LS_y - y) / (LS_x - x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
    // Q4
    else if ((x - LS_x > 0) && (y - LS_y < 0)) {
      key = hashFunction(x - 1, y + 1, lightSourceNumber);
      if (!visibilityHashMap_.count(key)) {
        updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y + 1);
      }
      if (x - LS_x == LS_y - y) {
        v = visibilityHashMap_.at(key);
      } else if (x - LS_x < LS_y - y) {
        const auto key_1 = hashFunction(x, y + 1, lightSourceNumber);
        if (!visibilityHashMap_.count(key_1)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x, y + 1);
        }
        c = static_cast<double>(x - LS_x) / (LS_y - y);
        double v1 = visibilityHashMap_.at(key_1);
        v = v1 - c * (v1 - visibilityHashMap_.at(key));
      } else if (x - LS_x > LS_y - y) {
        const auto key_2 = hashFunction(x - 1, y, lightSourceNumber);
        if (!visibilityHashMap_.count(key_2)) {
          updatePointVisibility(lightSourceNumber, LS_x, LS_y, x - 1, y);
        }
        c = static_cast<double>(LS_y - y) / (x - LS_x);
        double v2 = visibilityHashMap_.at(key_2);
        v = v2 - c * (v2 - visibilityHashMap_.at(key));
      }
    }
  }
  v = v * sharedVisibilityField_->get(x, y);
  key = hashFunction(x, y, lightSourceNumber);
  visibilityHashMap_[key] = v;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::saveVisibilityBasedSolverImage(const Field<double> &gScore) const {
  const int width = nx_;
  const int height = ny_;

  sf::Image image;
  image.create(width, height);

  double minVal = std::numeric_limits<double>::max();
  double maxVal = std::numeric_limits<double>::min();

  // Find min and max values in gScore for normalization
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      double val = gScore(i, j);
      if (val == std::numeric_limits<double>::infinity())
        continue;
      if (val < minVal)
        minVal = val;
      if (val > maxVal)
        maxVal = val;
    }
  }

  // Generate the image
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      if (sharedVisibilityField_->get(i, j) < 1) {
        image.setPixel(i, j, sf::Color::Black);
      } else {
        const double normalized_value = gScore(i, j) / (maxVal - minVal);
        sf::Color color = VBM_getColor(normalized_value);
        image.setPixel(i, j, color);
      }
    }
  }

  // color all initial frontline points as green circles with radius 10
  sf::Color color;
  color.a = 1;
  int x0, y0;
  int radius = 10;
  for (size_t k = 0; k < sharedConfig_->initialFrontline.size(); k += 2) {
    x0 = sharedConfig_->initialFrontline[k];
    y0 = ny_ - 1 - sharedConfig_->initialFrontline[k + 1];
    for (int i = -radius; i <= radius; ++i) {
      for (int j = -radius; j <= radius; ++j) {
        if (i * i + j * j <= radius * radius) {
          if (x0 + i >= 0 && x0 + i < nx_ && y0 + j >= 0 && y0 + j < ny_) {
            image.setPixel(x0 + i, y0 + j, color.Green);
          }
        }
      }
    }
  }

  // compute the step size based on the max and min values
  int number_of_contour_lines = sharedConfig_->number_of_contour_lines;
  double stepSize = (maxVal - minVal) / number_of_contour_lines;

  std::vector<double> contourLevels;
  for (double level = minVal; level <= maxVal; level += stepSize) {
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

  std::string outputPath = "./output/VBM_solver.png";
  image.saveToFile(outputPath);
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void VBM_Solver::saveResults(const std::vector<VBM_point> &resultingPath) const {
  namespace fs = std::filesystem;
  const std::string methodName = "VBM";
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

  // Save CameFrom_
  outputFilePath = "./output/" + methodName + "_cameFrom.txt";
  if (sharedConfig_->saveCameFrom) {
    std::fstream of1(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of1.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath
                << std::endl;
      return;
    }
    std::ostream &os1 = of1;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os1 << cameFrom_(i, j) << " ";
      }
      os1 << "\n";
    }
    of1.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " cameFrom_" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }

  // SavePivots
  outputFilePath = "./output/" + methodName + "_pivots.txt";
  if (sharedConfig_->savePivots) {
    std::fstream of3(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of3.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath
                << std::endl;
      return;
    }
    std::ostream &os = of3;
    for (size_t i = 0; i < nb_of_sources_; ++i) {
      os << lightSources_[i].x << " " << ny_ - 1 - lightSources_[i].y;
      os << "\n";
    }
    of3.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved " + methodName + " pivots" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }
}

} // namespace vbm