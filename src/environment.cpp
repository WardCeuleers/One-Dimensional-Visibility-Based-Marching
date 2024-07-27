#include "environment/environment.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace vbd {

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
Environment::Environment(Config &config)
    : sharedConfig_(std::make_shared<Config>(config)) {
  if (sharedConfig_->mode == 1) {
    nx_ = sharedConfig_->ncols;
    ny_ = sharedConfig_->nrows;

    if (!sharedConfig_->randomSeed) {
      seedValue_ = sharedConfig_->seedValue;
    }
    generateNewEnvironmentFromSettings();
    if (sharedConfig_->saveResults) {
      saveEnvironment();
    }
  } else if (sharedConfig_->mode == 2) {
    nx_ = sharedConfig_->ncols;
    ny_ = sharedConfig_->nrows;
    generateCustomEnvironmentFromSettings();
    if (sharedConfig_->saveResults) {
      saveEnvironment();
    }
  } else if (sharedConfig_->mode == 3) {
    // loadMaps(sharedConfig_->imagePath);
    if (loadImage(sharedConfig_->imagePath)) {
      if (sharedConfig_->saveResults) {
        saveEnvironment();
      }
    } else {
      std::cerr << "Failed to load image" << std::endl;
      return;
    }
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Environment::generateNewEnvironmentFromSettings() {
  resetEnvironment();
  int seedValue = 0;
  if (!sharedConfig_->randomSeed) {
    seedValue = seedValue_;
  } else {
    // Get the current time point using the high resolution clock
    auto time_point = std::chrono::high_resolution_clock::now();
    // Convert the time point to nanoseconds since the epoch
    auto ns_since_epoch =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(time_point)
            .time_since_epoch()
            .count();
    seedValue = ns_since_epoch;
  }
  std::srand(seedValue);

  for (int i = 0; i < sharedConfig_->nb_of_obstacles; ++i) {
    int col_1 = 1 + (std::rand() % (nx_ - 0 + 1));
    int col_2 =
        col_1 + sharedConfig_->minWidth +
        (std::rand() % (sharedConfig_->maxWidth - sharedConfig_->minWidth + 1));
    col_1 = std::min(col_1, (int)nx_ - 1);
    col_2 = std::min(col_2, (int)nx_ - 1);

    int row_1 = 1 + (std::rand() % (ny_ - 0 + 1));
    int row_2 = row_1 + sharedConfig_->minHeight +
                (std::rand() %
                 (sharedConfig_->maxHeight - sharedConfig_->minHeight + 1));
    row_1 = std::min(row_1, (int)ny_ - 1);
    row_2 = std::min(row_2, (int)ny_ - 1);

    for (int j = col_1; j < col_2; ++j) {
      for (int k = row_1; k < row_2; ++k) {
        sharedOccupancyField_->set(j, k, 1);
        if (sharedConfig_->vbm) {
          sharedVisibilityField_->set(j, k, 0.0);
          sharedSpeedField_->set(j, k, speedValue_);
        }
      }
    }
  }
  if (!sharedConfig_->silent) {
  std::cout << "########################### Environment output "
                "############################ \n"
            << "Generated new environment based on parsed settings at a seed "
                "value of: "
            << seedValue << std::endl;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Environment::generateCustomEnvironmentFromSettings() {
  resetEnvironment();
  auto &objectList = sharedConfig_->objectList;
  for (size_t i = 0; i < objectList.size(); i += 4) {
    int col_1 = objectList[i];
    int col_2 = objectList[i + 1];
    int row_1 = ny_-1-objectList[i + 3];
    int row_2 = ny_-1-objectList[i + 2];
    for (int j = col_1; j < col_2; ++j) {
      for (int k = row_1; k < row_2; ++k) {
        sharedOccupancyField_->set(j, k, 1);
        if (sharedConfig_->vbm) {
          sharedVisibilityField_->set(j, k, 0.0);
          sharedSpeedField_->set(j, k, speedValue_);
        }
      }
    }
  }
  if (!sharedConfig_->silent) {
  std::cout << "########################### Environment output "
                "############################ \n"
            << "Generated new environment based on the given object list" << std::endl;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Environment::loadMaps(const std::string &filename) {
  std::ifstream input(filename);
  std::vector<std::vector<float>> visibilityField;
  for (std::string line; std::getline(input, line);) {
    std::vector<float> floatVector = stringToFloatVector(line, ' ');
    visibilityField.push_back(floatVector);
  }
  nx_ = visibilityField.size();
  ny_ = visibilityField[0].size();
  std::cout << "Loaded vector of dimensions " << nx_ << "x" << ny_
            << " successfully" << std::endl;

  // Resets environment
  resetEnvironment();

  for (size_t i = 0; i < nx_; ++i) {
    for (size_t j = 0; j < ny_; ++j) {
      // set occupancy field
      if (visibilityField[i][j] >= 0.5) {
        sharedOccupancyField_->set(i, j, 0);
      } else {
        sharedOccupancyField_->set(i, j, 1);
      }
      // set visibility and speed field for vbm
      if (sharedConfig_->vbm) {
        sharedVisibilityField_->set(i, j, visibilityField[i][j]);
        if (visibilityField[i][j] == 1.0) {
          sharedSpeedField_->set(i, j, 1.0);
        } else {
          sharedSpeedField_->set(i, j, speedValue_);
        }
      }
    }
  }
  std::cout << "Loaded image of dimensions " << nx_ << "x" << ny_
            << " successfully" << std::endl;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
std::vector<float> Environment::stringToFloatVector(const std::string &str,
                                                    char delimiter) {
  std::vector<float> floatVector;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    floatVector.push_back(std::stof(item));
  }
  return floatVector;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Environment::loadImage(const std::string &filename) {
  uniqueLoadedImage_.reset(std::make_unique<sf::Image>().release());
  // Load the image from a file
  if (!uniqueLoadedImage_->loadFromFile(filename)) {
    std::cout << "Error: Failed to load image" << std::endl;
    return false;
  } else {
    auto size = uniqueLoadedImage_->getSize();
    nx_ = size.x;
    ny_ = size.y;

    resetEnvironment();

    sf::Color pixel;
    int gray;
    // Access the pixel data of the image
    for (size_t x = 0; x < nx_; ++x) {
      for (size_t y = 0; y < ny_; ++y) {
        pixel = uniqueLoadedImage_->getPixel(x, y);
        const int gray = 0.3 * pixel.r + 0.59 * pixel.g + 0.11 * pixel.b;
        if (gray > 128) {
          sharedOccupancyField_->set(x, y, 0);
          if (sharedConfig_->vbm) {
            sharedVisibilityField_->set(x, y, 1.0);
            sharedSpeedField_->set(x, y, 1.0);
          }
        } else {
          sharedOccupancyField_->set(x, y, 1);
          if (sharedConfig_->vbm) {
            sharedVisibilityField_->set(x, y, 0);
            sharedSpeedField_->set(x, y, speedValue_);
          }
        }
      }
    }
    std::cout << "Loaded image of dimensions " << nx_ << "x" << ny_
              << " successfully" << std::endl;
    return true;
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Environment::resetEnvironment() {
  sharedOccupancyField_ = std::make_unique<Field<int>>(nx_, ny_, 0);
  if (sharedConfig_->vbm) {
    sharedVisibilityField_ = std::make_unique<Field<double>>(nx_, ny_, 1.0);
    sharedSpeedField_ = std::make_unique<Field<double>>(nx_, ny_, 1.0);
  }
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Environment::saveEnvironment() {
  // Define the path to the output file
  std::string outputFilePath = "./output/occupancyField.txt";
  // Check if the directory exists, and create it if it doesn't
  namespace fs = std::filesystem;
  fs::path directory = fs::path(outputFilePath).parent_path();
  if (!fs::exists(directory)) {
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory " << directory.string()
                << std::endl;
      return;
    }
  }

  // save Occupancy Field
  if (sharedConfig_->saveResults && sharedConfig_->saveOccupancyField) {
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << sharedOccupancyField_->get(i, j) << " ";
      }
      os << "\n";
    }
    of.close();
        if (!sharedConfig_->silent) {
      std::cout << "Saved visibility field" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }
  // save Visibility Field
  outputFilePath = "./output/visibilityField.txt";
  if (sharedConfig_->vbm && sharedConfig_->saveResults && sharedConfig_->saveVisibilityField) {
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << sharedVisibilityField_->get(i, j) << " ";
      }
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved visibility field" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }
  // save Speed Field
  outputFilePath = "./output/speedField.txt";
  if (sharedConfig_->vbm && sharedConfig_->saveResults && sharedConfig_->saveVisibilityField) {
    std::fstream of(outputFilePath, std::ios::out | std::ios::trunc);
    if (!of.is_open()) {
      std::cerr << "Failed to open output file " << outputFilePath << std::endl;
      return;
    }
    std::ostream &os = of;
    for (int j = ny_ - 1; j >= 0; --j) {
      for (size_t i = 0; i < nx_; ++i) {
        os << sharedSpeedField_->get(i, j) << " ";
      }
      os << "\n";
    }
    of.close();
    if (!sharedConfig_->silent) {
      std::cout << "Saved speed field" << std::endl;
    }
  } else {
    if (fs::exists(outputFilePath))
      fs::remove(outputFilePath);
  }
}

} // namespace vbd