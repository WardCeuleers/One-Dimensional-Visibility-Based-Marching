#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <string>
#include <vector>

namespace vbd {

using size_t = std::size_t;

struct Config {
  int mode = 1;
  size_t ncols = 100;
  size_t nrows = 100;
  int nb_of_obstacles = 10;
  size_t minWidth = 10;
  size_t maxWidth = 20;
  size_t minHeight = 10;
  size_t maxHeight = 20;
  bool randomSeed = true;
  int seedValue = 0;
  std::vector<int> objectList;
  std::string imagePath = "C:\\..."; // or /home/...
  bool marchControl = false;
  size_t marchStepSize = 1;
  int pos_x;
  int pos_y;
  bool timer = true;
  size_t max_nb_of_iterations = 1000;
  bool saveResults = true;
  bool saveEnvironment = true;
  bool savegScore = true;
  bool savePivots = true;
  bool saveCameFrom = true;
  bool silent = false;
  bool debugCardinalSearch = false;
  bool debugPivotCreation = false;
  bool debugBoundary = false;
  bool debugPivotSearch = false;
  bool saveVisibilityBasedSolverImage = true;
  bool contourLines = true;
  int number_of_contour_lines = 50;
  bool colorPivots = false;
  int pivotColorGridSize = 16;
  bool colorBoundaries = false;
  bool colorCameFrom = true;
};

// Singleton parser classs
class ConfigParser {
public:
  static ConfigParser &getInstance() {  // static such that there can only be one instance of the ConfigParser
    static ConfigParser instance;       // Create the ConfigParser called Instance when getInstance is called for the first time
    return instance;
  }

  bool parse(const std::string &filename); 
  const Config &getConfig() const;     // getter that can't change any class members and returns a reference of which you cannot change the content

  ConfigParser(ConfigParser const &) = delete;    // delete the copy constructor
  void operator=(ConfigParser const &) = delete;  // delete the assignment operator

private:
  ConfigParser() = default; // Private constructor
  Config config_;
  std::vector<int> parseVectorString(const std::string &str);
};

} // namespace vbd
#endif // CONFIG_PARSER_HPP