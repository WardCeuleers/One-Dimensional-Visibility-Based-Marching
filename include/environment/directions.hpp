#ifndef DIRECTIONS_HPP
#define DIRECTIONS_HPP

#include <memory>

namespace vbd {

enum class cardir : uint8_t {
  North, East, South, West, None
};

// Cardir methods
inline cardir nextDirection(cardir dir) {
  switch(dir) {
    case cardir::North:   return cardir::East;
    case cardir::East:    return cardir::South;
    case cardir::South:   return cardir::West; 
    case cardir::West:    return cardir::North; 
    default: return dir;
  }
}
inline cardir prevDirection(cardir dir) {
  switch(dir) {
    case cardir::North:   return cardir::West;
    case cardir::East:    return cardir::North;
    case cardir::South:   return cardir::East; 
    case cardir::West:    return cardir::South; 
    default: return dir;
  }
}
inline cardir oppDirection(cardir dir) {
  switch(dir) {
    case cardir::North:   return cardir::South;
    case cardir::East:    return cardir::West;
    case cardir::South:   return cardir::North;
    case cardir::West:    return cardir::East;
    default: return dir;
  }
}
inline int cardir_to_int(cardir dir) {
  return static_cast<uint8_t>(dir);
}
inline const char* cardir_to_string(cardir dir) {
  switch(dir) {
    case cardir::North:   return "North";
    case cardir::East:    return "East";
    case cardir::South:   return "South";
    case cardir::West:    return "West";
    default: return "None";
  }
}
// Searchdir methods
using searchdir = std::pair<cardir, cardir>;

inline searchdir inverseSearch(searchdir dir) {
  return {dir.second, dir.first};
}
inline searchdir reverseSecondarySearch(searchdir dir) {
  return {dir.first, oppDirection(dir.second)};
}
inline searchdir reversePrimarySearch(searchdir dir) {
  return {oppDirection(dir.first), dir.second};
}
inline searchdir reverseSearch(searchdir dir) {
  return {oppDirection(dir.first), oppDirection(dir.second)};
}
inline searchdir orthogonalSearch(searchdir dir) {
  return {dir.second, oppDirection(dir.first)};
}
inline searchdir flippedSearch(searchdir dir) {
  return {oppDirection(dir.second), oppDirection(dir.first)};
}
// cardirIterator
class cardirIterator {
public:
  cardirIterator()
    : current_(cardir::North), count_(4) {}
  cardirIterator(const cardir& start)
    : current_(start), count_(4) {}
  cardirIterator(const cardir& start,int count)
    : current_(start), count_(count) {}
 
  cardirIterator operator++(int) {
    current_ = nextDirection(current_);
    count_--;
    return *this;
  }
  cardirIterator operator--(int) {
    current_ = prevDirection(current_);
    count_--;
    return *this;
  }
  const cardir operator*() const {return current_; }
  bool finished() const { return count_ != 0;}
private:
  cardir current_;
  int count_;
};
} // namespace vbd

#endif // 