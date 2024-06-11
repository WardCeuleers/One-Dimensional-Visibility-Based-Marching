#ifndef SOLVER_UTILS_CPP
#define SOLVER_UTILS_CPP

#include "solver/solver.hpp"
#include <iostream>

namespace vbd {
	
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int const Solver::evaluateCardinalDistance(const cardir& dir, const point& p1, const point& p2) {
  switch (dir) {
    case cardir::North:
      return p1.second-p2.second;
    case cardir::South: 
      return p2.second-p1.second;
    case cardir::East:
      return p2.first-p1.first;
    case cardir::West:  
      return p1.first-p2.first;
    default: 
      return std::numeric_limits<int>::max();
  }
}
/*****************************************************************************/
int const Solver::evaluateCardinalDistance(const cardir& dir, const int& x1, const int& y1, const int& x2, const int& y2) {
  switch (dir) {
    case cardir::North:
      return y1-y2;
    case cardir::South: 
      return y2-y1;
    case cardir::East:
      return x2-x1;
    case cardir::West:  
      return x1-x2;
    default: 
      return std::numeric_limits<int>::max();
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
double const Solver::getDistance(int x, int y, cardir dir_1, bool reverse) {
  // march
  if (reverse) {
    switch (dir_1) {
      case cardir::North:
        if (y == ny_-1) {return infinity;}
        y += 1; break;
      case cardir::East:
        if (x == 0) {return infinity;}
        x -= 1; break;
      case cardir::South: 
        if (y == 0) {return infinity;}
        y -= 1; break;
      case cardir::West: 
        if (x == nx_-1) {return infinity;}
        x += 1; break;
      default: 
        std::cout << "Error: invalid direction in getDistance" << std::endl;
        return infinity;
    }
  }
  else {
    switch (dir_1) {
      case cardir::North: 
        if (y == 0) {return infinity;}
        y -= 1; break;
      case cardir::East:
        if (x == nx_-1) {return infinity;}
        x += 1; break;
      case cardir::South: 
        if (y == ny_-1) {return infinity;}
        y += 1; break;
      case cardir::West: 
        if (x == 0) {return infinity;}
        x -= 1; break;
      default: 
        std::cout << "Error: invalid direction in getDistance" << std::endl;
        return infinity;
    }
  }
  return gScore_(x, y);
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int const Solver::distanceToEdge(const cardir& dir, const int& x, const int& y) {
  switch (dir) {
    case cardir::North: return y;
    case cardir::East:  return nx_- x - 1;
    case cardir::South: return ny_- y - 1;
    case cardir::West:  return x;
    default: 
      std::cout << "Error: invalid direction in distanceToEdge" << std::endl;
      return 0;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void const Solver::forceMove(int& x, int& y, const cardir& dir, int steps) {
  nb_of_marches_++;
  // march
  switch (dir) {
    case cardir::North: y -= steps; break;
    case cardir::East:  x += steps; break;
    case cardir::South: y += steps; break;
    case cardir::West:  x -= steps; break;
    default: 
      std::cerr << "Error: invalid direction in forceMove" << std::endl;
      break;
  }
}
/*****************************************************************************/
void const Solver::forceMove(int& x, int& y, const cardir& dir_1, int steps_1, const cardir& dir_2, int steps_2) {
  nb_of_marches_++;
  // march
  switch (dir_1) {
    case cardir::North: 
      y -= steps_1;
      if (dir_2 == cardir::East) {x += steps_2; return;}
      else if (dir_2 == cardir::West) {x -= steps_2; return;}
      else {break;}
    case cardir::East:  
      x += steps_1;
      if (dir_2 == cardir::North) {y -= steps_2; return;}
      else if (dir_2 == cardir::South) {y += steps_2; return;}
      else {break;}
    case cardir::South: 
      y += steps_1;
      if (dir_2 == cardir::East) {x += steps_2; return;}
      else if (dir_2 == cardir::West) {x -= steps_2; return;}
      else {break;}
    case cardir::West:  
      x -= steps_1;
      if (dir_2 == cardir::North) {y -= steps_2; return;}
      else if (dir_2 == cardir::South) {y += steps_2; return;}
      else {break;}
    default: break;
  }
  std::cerr << "Error: invalid direction in forceMove" << std::endl;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::move(int& x, int& y, const cardir& dir, int steps) {
  nb_of_marches_++;
  // march
  switch (dir) {
    case cardir::North: 
      if (y < steps) {return false;}
      else {y -= steps; return true;}
    case cardir::East:  
      if (x >= nx_-steps) {return false;}
      else {x += steps; return true;}
    case cardir::South: 
      if (y >= ny_-steps) {return false;}
      else {y += steps; return true;}
    case cardir::West:  
      if (x < steps) {return false;}
      else {x -= steps; return true;}
    default: 
      std::cout << "Error: invalid direction in move" << std::endl;
      return false;
  }
}
/*****************************************************************************/
bool const Solver::move(int& x, int& y, const cardir& dir_1, int steps_1, const cardir& dir_2, int steps_2) {
  nb_of_marches_++;
  // march
  switch (dir_1) {
    case cardir::North:
      if (y < steps_1) {return false;}
      else if (dir_2 == cardir::East) {
        if (x >= nx_-steps_2) {return false;}
        else {y -= steps_1; x += steps_2; return true;}
      }
      else if (dir_2 == cardir::West) {
        if (x < steps_2) {return false;}
        else {y -= steps_1; x -= steps_2; return true;}
      }
      else {break;}
    case cardir::East:  
      if (x >= nx_-steps_1) {return false;}
      else if (dir_2 == cardir::North) {
        if (y < steps_2) {return false;}
        else {x += steps_1; y -= steps_2; return true;}
      }
      else if (dir_2 == cardir::South) {
        if (y >= ny_-steps_2) {return false;}
        else {x += steps_1; y += steps_2; return true;}
      }
      else {break;}
    case cardir::South: 
      if (y >= ny_-steps_1) {return false;}
      else if (dir_2 == cardir::East) {
        if (x >= nx_-steps_2) {return false;}
        else {y += steps_1; x += steps_2; return true;}
      }
      else if (dir_2 == cardir::West) {
        if (x < steps_2) {return false;}
        else {y += steps_1; x -= steps_2; return true;}
      }
      else {break;}
    case cardir::West:  
      if (x < steps_1) {return false;}
      else if (dir_2 == cardir::North) {
        if (y < steps_2) {return false;}
        else {x -= steps_1; y -= steps_2; return true;}
      }
      else if (dir_2 == cardir::South) {
        if (y >= ny_-steps_2) {return false;}
        else {x -= steps_1; y += steps_2; return true;}
      }
      else {break;}
    default: 
      break;
  }
  std::cout << "Error: invalid direction in move" << std::endl;
  return false;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::advance(int& x, int& y, const cardir& dir) {
  nb_of_marches_++;
  // march and check for out of bounds
  switch (dir) {
    case cardir::North: 
      if (y == 0) {return false;}
      else {y -= 1; break;}
    case cardir::East:  
      if (x == nx_-1) {return false;}
      else {x += 1; break;}
    case cardir::South: 
      if (y == ny_-1) {return false;}
      else {y += 1; break;}
    case cardir::West:  
      if (x == 0) {return false;}
      else {x -= 1; break;}
    default: 
      std::cout << "Error: invalid direction in advance" << std::endl;
      return false;
  }
  // true if free, false if occupied
  return (!sharedOccupancyField_->get(x,y));
}
/*****************************************************************************/
bool const Solver::advance(int& x, int& y, const cardir& dir, bool& outOfBound) {
  nb_of_marches_++;
  // march and check for out of bounds
  switch (dir) {
    case cardir::North: 
      if (y == 0) {outOfBound = true; return false;}
      else {y -= 1; break;}
    case cardir::East:  
      if (x == nx_-1) {outOfBound = true; return false;}
      else {x += 1; break;}
    case cardir::South: 
      if (y == ny_-1) {outOfBound = true; return false;}
      else {y += 1; break;}
    case cardir::West:  
      if (x == 0) {outOfBound = true; return false;}
      else {x -= 1; break;}
    default: 
      std::cout << "Error: invalid direction in advance" << std::endl;
      return false;
  }
  // true if free, false if occupied
  return (!sharedOccupancyField_->get(x,y));
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::reverse(int& x, int& y, const cardir& dir) {
  nb_of_marches_++;
  // march and check for out of bounds
  switch (dir) {
    case cardir::North: 
      if (y == ny_-1) {return false;}
      else {y += 1; break;}
    case cardir::East:  
      if (x == 0) {return false;}
      else {x -= 1; break;}
    case cardir::South: 
      if (y == 0) {return false;}
      else {y -= 1; break;}
    case cardir::West:  
      if (x == nx_-1) {return false;}
      else {x += 1; break;}
    default: 
      break;
  }
  // true if free, false if occupied
  return (!sharedOccupancyField_->get(x,y));
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::validCell(const int& x, const int& y) {
  if (x >= nx_ || x < 0 || y >= ny_ || y < 0) {return false;}
  // return true if cell is occupied and false if free
  return (!sharedOccupancyField_->get(x,y));
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::checkForwards(const int& x, const int& y, const cardir& dir, int steps) {
  nb_of_marches_++;
  // march
  switch (dir) {
    case cardir::North: 
      if (y < steps) {return true;}
      else {return (sharedOccupancyField_->get(x,y-steps));}
    case cardir::East:  
      if (x >= nx_-steps) {return true;}
      else {return (sharedOccupancyField_->get(x+steps,y));}
    case cardir::South: 
      if (y >= ny_-steps) {return true;}
      else {return (sharedOccupancyField_->get(x,y+steps));}
    case cardir::West:  
      if (x < steps) {return true;}
      else {return (sharedOccupancyField_->get(x-steps,y));}
      break;
    default: 
      break;
  }
  // if cell is occupied
  return (sharedOccupancyField_->get(x,y));
}
/*****************************************************************************/
bool const Solver::checkForwards(const int& x, const int& y, const cardir& dir_1, const cardir& dir_2, int steps_1, int steps_2) {
  nb_of_marches_++;
  // march
  switch (dir_1) {
    case cardir::North: 
      if (y < steps_1) {return true;}
      switch (dir_2) {
        case cardir::East:  
          if (x >= nx_-steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x+steps_2,y-steps_1));}
        case cardir::West:
          if (x < steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x-steps_2,y-steps_1));}
        default:
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    case cardir::East:  
      if (x >= nx_-steps_1) {return true;}
      switch (dir_2) {
        case cardir::North: 
          if (y < steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x+steps_1,y-steps_2));}
        case cardir::South:
          if (y >= ny_-steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x+steps_1,y+steps_2));}
        default:
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    case cardir::South: 
      if (y >= ny_-steps_1) {return true;}
      switch (dir_2) {
        case cardir::East:  
          if (x >= nx_-steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x+steps_2,y+steps_1));}
        case cardir::West:
          if (x < steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x-steps_2,y+steps_1));}
        default:
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    case cardir::West:  
      if (x < steps_1) {return true;}
      switch (dir_2) {
        case cardir::North: 
          if (y < steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x-steps_1,y-steps_2));}
        case cardir::South: 
          if (y >= ny_-steps_2) {return true;}
          else {return (sharedOccupancyField_->get(x-steps_1,y+steps_2));}
        default:
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    default: 
      std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
      return true;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::checkBackwards(const int& x, const int& y, const cardir& dir, int steps) {
  nb_of_marches_++;
  // march
  switch (dir) {
    case cardir::North: 
      return sharedOccupancyField_->get(x,y+steps);
    case cardir::East:  
      return sharedOccupancyField_->get(x-steps,y);
    case cardir::South: 
      return sharedOccupancyField_->get(x,y-steps);
    case cardir::West:  
      return sharedOccupancyField_->get(x+steps,y);
      break;
    default: 
      break;
  }
  // return true if cell is occupied false otherwise
  return sharedOccupancyField_->get(x,y);
}
/*****************************************************************************/
bool const Solver::checkBackwards(const int& x, const int& y, const cardir& dir_1, const cardir& dir_2, int steps_1, int steps_2) {
  nb_of_marches_++;
  // march
  switch (dir_1) {
    case cardir::North: 
      switch (dir_2) {
        case cardir::East:  
          return (sharedOccupancyField_->get(x-steps_2,y+steps_1));
        case cardir::West:
          return (sharedOccupancyField_->get(x+steps_2,y+steps_1));
        default: 
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    case cardir::East:  
      switch (dir_2) {
        case cardir::North: 
          return (sharedOccupancyField_->get(x-steps_1,y+steps_2));
        case cardir::South: 
          return (sharedOccupancyField_->get(x-steps_1,y-steps_2));
        default: 
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    case cardir::South: 
      switch (dir_2) {
        case cardir::East:  
          return (sharedOccupancyField_->get(x-steps_2,y-steps_1));
        case cardir::West:
          return (sharedOccupancyField_->get(x+steps_2,y-steps_1));
        default: 
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    case cardir::West:  
      switch (dir_2) {
        case cardir::North: 
          return (sharedOccupancyField_->get(x+steps_1,y+steps_2));
        case cardir::South: 
          return (sharedOccupancyField_->get(x+steps_1,y-steps_2));
        default: 
          std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
          return true;
      }
    default: 
      std::cout << "Error: invalid direction in checkOccupancy" << std::endl;
      return true;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool const Solver::nextToSibling(const int& x, const int& y, const cardir& dir, const point& parent, bool reverse) {
  if (reverse) {
    switch (dir) {
      case cardir::North: return (cameFrom_(x,y+1) == parent);
      case cardir::East:  return (cameFrom_(x-1,y) == parent);
      case cardir::South: return (cameFrom_(x,y-1) == parent);
      case cardir::West:  return (cameFrom_(x+1,y) == parent);
      default: 
        std::cout << "Error: invalid direction in nextToSibling" << std::endl;
        return false;
    }
  }
  else {
    switch (dir) {
      case cardir::North: return (cameFrom_(x,y-1) == parent);
      case cardir::East:  return (cameFrom_(x+1,y) == parent);
      case cardir::South: return (cameFrom_(x,y+1) == parent);
      case cardir::West:  return (cameFrom_(x-1,y) == parent);
      default: 
        std::cout << "Error: invalid direction in nextToSibling" << std::endl;
        return false;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
Solver::point Solver::getNeighbour(const int& x, const int& y, const cardir& dir, bool reverse) {
  if (reverse) {
    switch (dir) {
      case cardir::North: return {x, y+1};
      case cardir::East:  return {x-1, y};
      case cardir::South: return {x, y-1};
      case cardir::West:  return {x+1, y};
      default: 
        std::cout << "Error: invalid direction in getNeighbour" << std::endl;
        return {x, y};
    }
  }
  else {
    switch (dir) {
      case cardir::North: return {x, y-1};
      case cardir::East:  return {x+1, y};
      case cardir::South: return {x, y+1};
      case cardir::West:  return {x-1, y};
      default: 
        std::cout << "Error: invalid direction in getNeighbour" << std::endl;
        return {x, y};
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
Solver::point Solver::getPivot(const int& x, const int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, const int& secondaryDist) {
  switch (primaryDir) {
    case cardir::North:
      switch (secondaryDir) {
        case cardir::East:  return {x-secondaryDist, y+primaryDist};
        case cardir::West:  return {x+secondaryDist, y+primaryDist};
        default: 
          std::cout << "Error: invalid direction in getPivot" << std::endl;
          return {x, y};
      }
    case cardir::East:
      switch (secondaryDir) {
        case cardir::North: return {x-primaryDist, y+secondaryDist};
        case cardir::South: return {x-primaryDist, y-secondaryDist};
        default: 
          std::cout << "Error: invalid direction in getPivot" << std::endl;
          return {x, y};
      }
    case cardir::South:
      switch (secondaryDir) {
        case cardir::East:  return {x-secondaryDist, y-primaryDist};
        case cardir::West:  return {x+secondaryDist, y-primaryDist};
        default: 
          std::cout << "Error: invalid direction in getPivot" << std::endl;
          return {x, y};
      }
    case cardir::West:
      switch (secondaryDir) {
        case cardir::North: return {x+primaryDist, y+secondaryDist};
        case cardir::South: return {x+primaryDist, y-secondaryDist};
        default: 
          std::cout << "Error: invalid direction in getPivot" << std::endl;
          return {x, y};
      }
    default: 
      std::cout << "Error: invalid direction in getPivot" << std::endl;
      return {x, y};
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
float const Solver::calcBlockSlope(const cardir& primaryDir, const cardir& secondaryDir, const point& parent, const point& block) {
  switch (primaryDir) {
    case cardir::North:
      if (secondaryDir==cardir::East) {
        return (block.first-parent.first-0.5)/(parent.second-block.second+0.5);
      }
      else if (secondaryDir==cardir::West) {
        return (parent.first-block.first-0.5)/(parent.second-block.second+0.5);
      }
      else {
        std::cout << "Error: invalid direction in calcBlockSlope" << std::endl;
        return infinity;
      }
    case cardir::South: 
      if (secondaryDir==cardir::East) {
        return (block.first-parent.first-0.5)/(block.second-parent.second+0.5);
      }
      else if (secondaryDir==cardir::West) {
        return (parent.first-block.first-0.5)/(block.second-parent.second+0.5);
      }
      else {
        std::cout << "Error: invalid direction in calcBlockSlope" << std::endl;
        return infinity;
      }
    case cardir::East:
      if (secondaryDir==cardir::North) {
        return (parent.second-block.second-0.5)/(block.first-parent.first+0.5);
      }
      else if (secondaryDir==cardir::South) {
        return (block.second-parent.second-0.5)/(block.first-parent.first+0.5);
      }
      else {
        std::cout << "Error: invalid direction in calcBlockSlope" << std::endl;
        return infinity;
      }
    case cardir::West: 
      if (secondaryDir==cardir::North) {
        return (parent.second-block.second-0.5)/(parent.first-block.first+0.5);
      }
      else if (secondaryDir==cardir::South) {
        return (block.second-parent.second-0.5)/(parent.first-block.first+0.5);
      }
      else {
        std::cout << "Error: invalid direction in calcBlockSlope" << std::endl;
        return infinity;
      }
    default: 
      std::cout << "Error: invalid direction in calcBlockSlope" << std::endl;
      return infinity;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
} // namespace vbd
#endif // SOLVER_UTILS_HPP