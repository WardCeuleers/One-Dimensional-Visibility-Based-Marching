#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <cmath>
#include <queue>
#include <vector>

#include "environment/environment.hpp"

#define infinity std::numeric_limits<float>::infinity()

namespace vbd {
/**
 * NodeTypes:
 *  1: Primary of straight
 *  2: Secondary march of straight search
 *  3: Primary of Slope, with reference at the pivot
 *  4: Primary of Slope, with reference at the parent with same directions
 *  5: Primary of Slope, with reference at the parent with inverse directions
 *  6: Secondary march of slope
 *  7: Occupied point
 *  8: Back of object, pivot correction
 *  9: Primary of switched straight search
 *  10: Node waiting to be turned into a pivot
*/
struct Node {
  uint8_t type;
  double distance;
  int x, y; 
  cardir primaryDir, secondaryDir;
  int primaryDist, secondaryDist;
  float slope;
  bool state;
  bool operator<(const Node &other) const { return distance > other.distance; }
};

class Solver {
public:
  using point = std::pair<int, int>;

  explicit Solver(Environment &env);
  ~Solver() = default;

  void visibilityBasedSolver();
  // march control functions
  void increaseMaxIter() { max_nb_of_iter_ += marchStepSize_;};
  void decreaseMaxIter() { max_nb_of_iter_ = max_nb_of_iter_ >= marchStepSize_ ? max_nb_of_iter_ -= marchStepSize_ : 0;};
  void togglePivots() { colorPivots_ = !colorPivots_;};
  void toggleBoundaries() { colorBoundaries_ = !colorBoundaries_;};
  void toggleContourLines() { showContour_ = !showContour_;};
  void toggleCameFrom() { colorCameFrom_ = !colorCameFrom_;};
  void changeMarchStepSize();
  void setTarget();
  void setMaxIter();
private:
  void reset();
  void saveResults(const std::vector<point> &path,
                   const std::string &methodName) const;
  void saveVisibilityBasedSolverImage(const Field<double> &gScore) const;

  std::shared_ptr<Field<int>> sharedOccupancyField_;

  Field<double> gScore_;    // Distance value to the start position
  Field<point> cameFrom_;   // Parent coordinates
  Field<point> blockCorners_;    // contains blockslope for four different cardinal directions

  std::shared_ptr<Config> sharedConfig_;
  std::unique_ptr<point[]> pivots_;
  // Robot Position
  int x_;
  int y_;
  point startPoint_;
  point nullPoint_;
  point focusPoint_;
  // Dimensions.
  size_t ny_;
  size_t nx_;
  
  size_t nb_of_iterations_;
  size_t nb_of_marches_;
  size_t nb_of_pivots_;

  // marchControl parameters
  size_t max_nb_of_iter_;
  size_t marchStepSize_;
  bool colorPivots_;
  bool colorBoundaries_;
  bool colorCameFrom_;
  bool showContour_;
  bool findNode_;
  int target_x_;
  int target_y_;
  int nbOfVisits_;
  // Heaps
  std::unique_ptr<std::priority_queue<Node>> openSet_;

  // Unique pointer to image holder
  std::unique_ptr<sf::Image> uniqueLoadedImage_;

  // ========= inline functions ==================================================================================

  inline int indexAt(const int x, const int y) const {
    return x + y * nx_;
  };
  inline point coordinatesAt(const int index) const {
    const int x = index % nx_;
    const int y = (index) / nx_;
    return {x, y};
  }
  inline const double evaluateDistance(const int x1, const int y1, const int x2, const int y2) const {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  };
  inline const double evaluateDistance(const point p1, const int x2, const int y2) const {
    return sqrt((p1.first - x2) * (p1.first - x2) + (p1.second - y2) * (p1.second - y2));
  };
  inline const double evaluateDistance(const point p1, const point p2) const {
    return sqrt((p1.first - p2.first) * (p1.first - p2.first) + (p1.second - p2.second) * (p1.second - p2.second));
  };
  inline bool closerToCurrent(const int& x, const int& y, const point& current, const point& other) {
  return (gScore_(current) + sqrt((current.first - x) * (current.first - x) + (current.second - y) * (current.second - y)) <= 
            gScore_(other) + sqrt((other.first - x) * (other.first - x) + (other.second - y) * (other.second - y)));
  };

  // ========= utility functions ==================================================================================
  int const evaluateCardinalDistance(const cardir& dir, const point& p1, const point& p2);
  int const evaluateCardinalDistance(const cardir& dir, const int& x1, const int& y1, const int& x2, const int& y2);
  double const getDistance(int x, int y, cardir dir_1, bool reverse = false);
  int const distanceToEdge(const cardir& dir, const int& x, const int& y);
  void const forceMove(int& x, int& y, const cardir& dir, int steps = 1);
  void const forceMove(int& x, int& y, const cardir& dir_1, int steps_1, const cardir& dir_2, int steps_2);
  bool const move(int& x, int& y, const cardir& dir, int steps = 1);
  bool const move(int& x, int& y, const cardir& dir_1, int steps_1, const cardir& dir_2, int steps_2);
  bool const advance(int& x, int& y, const cardir& dir);
  bool const advance(int& x, int& y, const cardir& dir, bool& outOfBound);
  bool const reverse(int& x, int& y, const cardir& dir);
  bool const check(const int& x, const int& y);
  bool const checkForwards(const int& x, const int& y, const cardir& dir, int steps_1=1);
  bool const checkForwards(const int& x, const int& y, const cardir& dir_1, const cardir& dir_2, int steps_1=1, int steps_2=1);
  bool const checkBackwards(const int& x, const int& y, const cardir& dir, int steps_1=1);
  bool const checkBackwards(const int& x, const int& y, const cardir& dir_1, const cardir& dir_2, int steps_1=1, int steps_2=1);
  point getNeighbour(const int& x, const int& y, const cardir& dir, bool reverse = false);
  point getPivot(const int& x, const int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, const int& secondaryDist);
  float const calcBlockSlope(const cardir& primaryDir, const cardir& secondaryDir, const point& parent, const point& block);

// ========= origin visibility functions ==========================================================================
  int  traverseObject(int& x, int& y, searchdir dir, const double maxMarchDist);
  int  traverseVoid(int& x, int& y, cardir dir, const double maxMarchDist);
  void march_to_slope(cardir dir, int& x, int& y, point parent, const int& primaryDist, const int& secondaryDist, const float& slope, int& visibilityDist, bool& marchBool, bool& outOfBound);
  int  advancePrimaryVisibility(searchdir dir, int& x, int& y, int& primaryDist, int& secondaryDist, float& primarySlope);
  void processSteepSlope(searchdir dir, int& x, int& y, const int visibilityDiff);
  void processMarchOver(searchdir dir, int& x, int& y, const float slope, const int visibilityDiff);
  void processJump(searchdir dir, int& x, int& y, const int primaryDist, const int secondaryDist, const float stopSlope, const int visibilityDist, const int prevVisibilityDist, bool traverseFirstObject = true);
  void ComputeDistanceFromCarindal(searchdir dir, const int basePrimaryVisibility, const int baseSecondaryVisibility);
  void ComputeDistanceBetweenSlopes(searchdir dir, int x_start, int y_start, int primaryDist, int secondaryDist, int prevVisibilityDist, int gapWidth, float blockSlope = infinity);
  void ComputeOriginVisibility();

  // ========= visibility march functions ==========================================================================
  void createNewPivot(const point pivot, const point parent, const cardir primaryDirt, const cardir secondaryDir, const float slope, bool inverseParent);
  void addNextStraightPrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist);
  void addNextPivotSlopePrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, int secondaryDist, const float& slope, bool onVisiblePoint);
  void addNextParentSlopePrimary(const u_int8_t& type, const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, int secondaryDist, const float& slope, bool onVisiblePoint);
  bool advanceSecondaryNode(uint8_t& type, double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist,  int& secondaryDist, bool& moveBoundary);
  bool advanceOccupiedNode(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist,  int& secondaryDist, float& slope);
  bool advancePivotSearch(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, int& primaryDist, const int& secondaryDist, float& slope, bool& nextToObject);
  void initiateSwitchedSearch(const point& pivot, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, const int& secondaryDist);
  bool addNextSwitchedPrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, const int& secondaryDist, bool firstPoint);
};

} // namespace vbd
#endif // SOLVER_HPP