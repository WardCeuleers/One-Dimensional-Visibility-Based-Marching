#include "parser/parser.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

namespace vbd {

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool ConfigParser::parse(const std::string &filename) {
  std::ifstream file(filename);
  if (!file) {
    std::cerr << "Failed to open " << filename << '\n';
    return false;
  }
  std::cout << "########################### Parsing config file  "
               "########################## \n";
  std::cout << "Attempting to parse " << filename << '\n';
  std::string line;
  while (std::getline(file, line)) {
    // Ignore comments and blank lines
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Split the line into key and value
    std::istringstream iss(line);
    std::string key;
    if (!std::getline(iss, key, '=')) {
      continue;
    }
    std::string value;
    if (!std::getline(iss, value)) {
      continue;
    }

    // Trim leading and trailing whitespace from key and value
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);

    // Parse the value based on the key's data type
    if (key == "mode") {
      try {
        config_.mode = std::stoi(value);
        if (config_.mode != 1 && config_.mode != 2 && config_.mode != 3) {
          std::cerr << "Invalid value for " << key << ": " << value
                    << ", using default value 1\n";
          config_.mode = 1;
        }
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "ncols") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.ncols = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "nrows") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.nrows = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "nb_of_obstacles") {
      try {
        config_.nb_of_obstacles = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "minWidth") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.minWidth = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "maxWidth") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.maxWidth = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "minHeight") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.minHeight = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
      }
    } else if (key == "maxHeight") {
      try {
        if (std::stoi(value) < 0) {
          std::cerr << "Invalid value for " << key << ": " << value << '\n';
          std::cerr << "It must be a positive integer\n";
          return false;
        }
        config_.maxHeight = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "pos_x") {
      try {
        config_.pos_x = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "pos_y") {
      try {
        config_.pos_y = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "randomSeed") {
      if (value == "0" || value == "false") {
        config_.randomSeed = false;
      } else if (value == "1" || value == "true") {
        config_.randomSeed = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "seedValue") {
      try {
        config_.seedValue = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "imagePath") {
      config_.imagePath = value;
    } else if (key == "marchControl") {
      if (value == "0" || value == "false") {
        config_.marchControl = false;
      } else if (value == "1" || value == "true") {
        config_.marchControl = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "objectList") {
      config_.objectList = parseVectorString(value);
    } else if (key == "timer") {
      if (value == "0" || value == "false") {
        config_.timer = false;
      } else if (value == "1" || value == "true") {
        config_.timer = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveResults") {
      if (value == "0" || value == "false") {
        config_.saveResults = false;
      } else if (value == "1" || value == "true") {
        config_.saveResults = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveEnvironment") {
      if (value == "0" || value == "false") {
        config_.saveEnvironment = false;
      } else if (value == "1" || value == "true") {
        config_.saveEnvironment = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "savegScore") {
      if (value == "0" || value == "false") {
        config_.savegScore = false;
      } else if (value == "1" || value == "true") {
        config_.savegScore = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "savePivots") {
      if (value == "0" || value == "false") {
        config_.savePivots = false;
      } else if (value == "1" || value == "true") {
        config_.savePivots = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveCameFrom") {
      if (value == "0" || value == "false") {
        config_.saveCameFrom = false;
      } else if (value == "1" || value == "true") {
        config_.saveCameFrom = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "max_nb_of_iterations") {
      try {
        config_.max_nb_of_iterations = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a positive integer\n";
        return false;
      }
    } else if (key == "silent") {
      if (value == "0" || value == "false") {
        config_.silent = false;
      } else if (value == "1" || value == "true") {
        config_.silent = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "debugCardinalSearch") {
      if (value == "0" || value == "false") {
        config_.debugCardinalSearch = false;
      } else if (value == "1" || value == "true") {
        config_.debugCardinalSearch = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "debugPivotCreation") {
      if (value == "0" || value == "false") {
        config_.debugPivotCreation = false;
      } else if (value == "1" || value == "true") {
        config_.debugPivotCreation = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "debugBoundary") {
      if (value == "0" || value == "false") {
        config_.debugBoundary = false;
      } else if (value == "1" || value == "true") {
        config_.debugBoundary = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "debugPivotSearch") {
      if (value == "0" || value == "false") {
        config_.debugPivotSearch = false;
      } else if (value == "1" || value == "true") {
        config_.debugPivotSearch = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "saveVisibilityBasedSolverImage") {
      if (value == "0" || value == "false") {
        config_.saveVisibilityBasedSolverImage = false;
      } else if (value == "1" || value == "true") {
        config_.saveVisibilityBasedSolverImage = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "contourLines") {
      if (value == "0" || value == "false") {
        config_.contourLines = false;
      } else if (value == "1" || value == "true") {
        config_.contourLines = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "number_of_contour_lines") {
      try {
        config_.number_of_contour_lines = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "colorPivots") {
      if (value == "0" || value == "false") {
        config_.colorPivots = false;
      } else if (value == "1" || value == "true") {
        config_.colorPivots = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "pivotColorGridSize") {
      try {
        config_.pivotColorGridSize = std::stoi(value);
      } catch (...) {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be an integer\n";
        return false;
      }
    } else if (key == "colorBoundaries") {
      if (value == "0" || value == "false") {
        config_.colorBoundaries = false;
      } else if (value == "1" || value == "true") {
        config_.colorBoundaries = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else if (key == "colorCameFrom") {
      if (value == "0" || value == "false") {
        config_.colorCameFrom = false;
      } else if (value == "1" || value == "true") {
        config_.colorCameFrom = true;
      } else {
        std::cerr << "Invalid value for " << key << ": " << value << '\n';
        std::cerr << "It must be a boolean\n";
        return false;
      }
    } else {
      std::cerr << "Invalid/Irrelevant key: " << key << '\n';
      std::cerr << "Ignoring...\n";
    }
  }
  if (!config_.silent) {
    if (config_.mode == 1) {
      std::cout << "Random environment mode" << std::endl;
      std::cout << "########################### Environment settings "
                   "########################## \n"
                << "ncols: " << config_.ncols << "\n"
                << "nrows: " << config_.nrows << "\n"
                << "Nb of obstacles: " << config_.nb_of_obstacles << "\n"
                << "Min width: " << config_.minWidth << "\n"
                << "Max width: " << config_.maxWidth << "\n"
                << "Min height: " << config_.minHeight << "\n"
                << "Max height: " << config_.maxHeight << std::endl;
      if (config_.randomSeed) {
        std::cout << "Random seed: " << config_.randomSeed << std::endl;
      } else {
        std::cout << "Fixed seed value: " << config_.seedValue << std::endl;
      }
    } else if (config_.mode == 2) {
        std::cout << "Custom environment mode"
                  << "\n";
        if (config_.objectList.empty()) {
          std::cerr << "Empty objectList\n";
          return false;
        }
        else if (config_.objectList.size() % 4 != 0) {
          std::cerr << "ObjectList must be of size that is a multiple of 4\n";
          return false;
        }
        std::cout << "########################### Environment settings "
            "########################## \n"
        << "ncols: " << config_.ncols << "\n"
        << "nrows: " << config_.nrows << std::endl;
        for (size_t i = 0; i < config_.objectList.size(); i += 4) {
          std::cout << "Object " << i / 4 + 1 << ": (" << config_.objectList[i]
                    << ", " << config_.objectList[i + 2] << ") -> ("
                    << config_.objectList[i + 1] << ", "
                    << config_.objectList[i + 3] << ")" << std::endl;
        }
    } else if (config_.mode == 3) {
        std::cout << "########################### Environment settings "
            "########################## \n";
        std::cout << "Import image mode"
                << "\n"
                << "Image path: " << config_.imagePath << std::endl;
    }
    std::cout
        << "############################ Solver settings "
           "############################## \n";
    std::cout << "Robot Position: " << config_.pos_x << ", " << config_.pos_y << "\n"
              << "max number of iterations: " << config_.max_nb_of_iterations << "\n"
              << "marchControl: " << config_.marchControl <<std::endl;
    std::cout << "############################ Output settings "
                 "############################## \n"
              << "timer: " << config_.timer << "\n"
              << "saveResults: " << config_.saveResults << "\n"
              << "saveEnvironment: " << config_.saveEnvironment << "\n"
              << "savegScore: " << config_.savegScore << "\n"
              << "savePivots: " << config_.savePivots << "\n"
              << "saveCameFrom: " << config_.saveCameFrom << std::endl;
  }
  return true;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
std::vector<int> ConfigParser::parseVectorString(const std::string &str) {
  // Remove outer braces
  std::string innerStr = str.substr(1, str.size() - 2);
  // Split string into individual integers
  std::vector<int> result;
  std::string delimiter = ",";
  size_t pos = 0;
  while ((pos = innerStr.find(delimiter)) != std::string::npos) {
    std::string token = innerStr.substr(0, pos);
    result.push_back(std::stoi(token));
    innerStr.erase(0, pos + delimiter.length());
  }
  result.push_back(std::stoi(innerStr)); // Add the last integer
  return result;
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
const Config &ConfigParser::getConfig() const { return config_; }

} // namespace vbd