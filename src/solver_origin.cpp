#ifndef SOLVER_ORIGIN_CPP
#define SOLVER_ORIGIN_CPP

#include "solver/solver.hpp"
#include "solver_utils.cpp"
#include <iostream>

namespace vbd {
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::ComputeOriginVisibility() {
  // Calculate Primary visibility distance for all cardinal directions
  int CardinalVisibility[4] = {0};
  int x_cur;
  int y_cur;
  int primary_idx;
  int secondary_idx;
  for (cardirIterator dir; dir.finished(); dir++) {
    primary_idx = cardir_to_int(*dir);
    x_cur = x_;
    y_cur = y_;
    while (advance(x_cur, y_cur, *dir)) {
      CardinalVisibility[primary_idx]++;
      gScore_(x_cur, y_cur) = evaluateDistance(x_, y_, x_cur, y_cur);
      cameFrom_(x_cur, y_cur) = startPoint_;
    }
  }
  // Secundary visibilities for all directions
  for (cardirIterator primary; primary.finished(); primary++ ) {
    cardir nextDir = nextDirection(*primary);
    ComputeDistanceFromCarindal({*primary, nextDir}, CardinalVisibility[cardir_to_int(*primary)], CardinalVisibility[cardir_to_int(nextDir)]);
  }   
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int Solver::traverseObject(int& x, int& y, searchdir dir, const double maxMarchDist) {
  bool in_object = true;
  int dist_in_object = 0;
  while (in_object && !(dist_in_object > maxMarchDist)) {
    if (checkBackwards(x, y, dir.first))
      return -dist_in_object;
    forceMove(x, y, dir.second);
    dist_in_object++;
    //box check
    if (x >= nx_ || x < 0 || y >= ny_ || y < 0) 
      return dist_in_object;
    // check for free space
    else if (!sharedOccupancyField_->get(x, y))
      in_object = false;
  }
  if (checkBackwards(x, y, dir.first))
    return -dist_in_object;
  else
    return dist_in_object;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int Solver::traverseVoid(int& x, int& y, cardir dir, const double maxMarchDist) {
  bool in_void = true;
  int dist_in_void = 0;
  while (in_void && (dist_in_void <= maxMarchDist)) {
    forceMove(x, y, dir);
    dist_in_void++;
    //box check
    if (x >= nx_ || x < 0 || y >= ny_ || y < 0) 
      return dist_in_void;
    // check for occupied cell
    else if (sharedOccupancyField_->get(x, y))
      in_void = false;
  }
  return dist_in_void;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::march_to_slope(cardir dir, int& x, int& y, point parent, const int& primaryDist, const int& secondaryDist, const float& slope, int& visibilityDist, bool& marchBool, bool& outOfBound) {
  visibilityDist = 0;
  marchBool = false;
  outOfBound = false;
  while (advance(x, y, dir, outOfBound)) {
    visibilityDist++;
    if (visibilityDist + secondaryDist <= primaryDist*slope) {
      gScore_(x, y) = evaluateDistance(x, y, parent.first, parent.second);
      cameFrom_(x, y) = parent;
    } 
    else {
      marchBool = true;
      visibilityDist--;
      break;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
int Solver::advancePrimaryVisibility(searchdir dir, int& x, int& y, int& primaryDist, int& secondaryDist, float& primarySlope) {
  // march in primary direction and return error if out of bounds
  if(!move(x, y, dir.first))
    return 3;
  primaryDist++;
  // secondary dist where clipping could happen
  int secondaryClippingDist = round((primaryDist-0.5)*primarySlope);
  bool clipping = false;
  // march in secondary direction untill you are past the slope or out of bounds
  while (primaryDist*primarySlope > secondaryDist) {
    if(!move(x, y, dir.second))
      return 3;
    secondaryDist++;
    // check for valid path to the current point
    if (getDistance(x, y, dir.first, true) == infinity) 
      return 3;
    // check for clipping
    if (secondaryDist >= secondaryClippingDist) {
      if (checkBackwards(x, y, dir.first)) {
        primarySlope = (secondaryDist + 0.5)/(primaryDist - 0.5);
        clipping = true;
      }
    } 
  }
  // check for object at end of march
  if (!sharedOccupancyField_->get(x, y)) {
    if (clipping)
      return 1;
    else
      return 0;
  }
  else
    return 2;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::processSteepSlope(searchdir dir, int& x, int& y, const int visibilityDiff) {
  if (sharedConfig_->debugCardinalSearch) {
    std::cout <<  "-----SteepSlope detected at (" << x << ", " << ny_-1-y << ")-----\n";
  }
  int march_dist = 0;
  bool foundGap = false;
  for (march_dist; march_dist < visibilityDiff-1; march_dist++) {
    if (!foundGap && !checkBackwards(x, y, dir.first, dir.second)) {
      foundGap = true;
    }
    else if (foundGap && checkBackwards(x, y, dir.first, dir.second)) {
      if (sharedConfig_->debugPivotCreation) {
        std::cout << "  -> created a pivot at (" << x << ", " << ny_-1-y << "): gap pivot\n";
      }
      // create pivot with direction ortogonal to current
      openSet_->push(Node{10, evaluateDistance(x_, y_, x, y), x, y, dir.second, oppDirection(dir.first), 0, 0, 0, false});
      foundGap = false; 
    }
    forceMove(x, y, dir.second, -1);
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::processMarchOver(searchdir dir, int& x, int& y, const float slope, const int visibilityDiff) {
  if (sharedConfig_->debugCardinalSearch) {
    std::cout <<  "-----MarchOver detected at (" << x << ", " << ny_-1-y << ")-----\n";
    std::cout << "       stopSlope   = " << slope << "\n";
  }
  forceMove(x, y, dir.second, -1);
  // for small visibiltiyDiff, gap and march beyond are impossible
  if (visibilityDiff <= 1) {
    if (sharedConfig_->debugPivotCreation) {
      std::cout << "  -> created a pivot at (" << x << ", " << ny_-1-y << "): small visibility diff\n";
    }
    // create pivot with same direction
    openSet_->push(Node{10, evaluateDistance(x_, y_, x, y), x, y, dir.first, dir.second, 0, 0, slope, false});
    return;
  }
  //if large visibilityDiff, add only lowest possible pivot
  Node pivot;
  int march_dist = 0;
  if (checkBackwards(x, y, dir.first)) {
    if (sharedConfig_->debugPivotCreation) {
      std::cout << "  -> created a pivot at (" << x << ", " << ny_-1-y << "): object next to start\n";
    }
    // create pivot with same direction
    pivot = Node{8, evaluateDistance(x_, y_, x, y), x, y, dir.first, dir.second, 0, 0, slope, false};
  }
  // else marchover location is beyond the object -> march back to the object
  else {
    while(!checkBackwards(x, y, dir.first, dir.second)) {
      forceMove(x, y, dir.second, -1);
      march_dist++;
    }
    if (sharedConfig_->debugPivotCreation) {
      std::cout << "  -> created a pivot at (" << x << ", " << ny_-1-y << "): object after marchdown\n";
    }
    // create pivot with direction ortogonal to current
    pivot = Node{8, evaluateDistance(x_, y_, x, y), x, y, dir.second, oppDirection(dir.first), 0, 0, -1/slope, false};
  }
  // if the march is not at the previous visibility -> their could be a gap 
  for (march_dist; march_dist < visibilityDiff-1; march_dist++) {
    bool foundGap = false;
    if (!foundGap && !checkBackwards(x, y, dir.first, dir.second)) {
      foundGap = true;
    }
    else if (foundGap && checkForwards(x, y, dir.first, dir.second)) {
      if (sharedConfig_->debugPivotCreation) {
        std::cout << "  -> created a pivot at (" << x << ", " << ny_-1-y << "): gap pivot\n";
      }
      // create pivot with direction ortogonal to current
      pivot = Node{8, evaluateDistance(x_, y_, x, y), x, y, dir.second, oppDirection(dir.first), 0, 0, -1/slope, false};
      foundGap = false; 
    }
    forceMove(x, y, dir.second, -1);
  }
  // push the last found node
  openSet_->push(pivot);
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::processJump(searchdir dir, int& x, int& y, const int primaryDist, const int secondaryDist, const float stopSlope, const int visibilityDist, const int prevVisibilityDist, bool traverseFirstObject) {
  if (sharedConfig_->debugCardinalSearch) {
    std::cout << "-----jump detected at (" << x << ", " << ny_-1-y << ")-----\n";
    std::cout << "      primaryDist  = " << primaryDist << "\n";
    std::cout << "     secondaryDist = " << secondaryDist << "\n";
    std::cout << "    visibilityDist = " << visibilityDist << "\n";
    std::cout << "     prevVisibDist = " << prevVisibilityDist << "\n";
    std::cout << "       stopSlope   = " << stopSlope << "\n";
  }
// calculate the lengt of visible jumpline = maxMarchDist
  double maxMarchDist = primaryDist*stopSlope - secondaryDist;
  int dist_to_edge = distanceToEdge(dir.second, x, y);
  if (maxMarchDist > dist_to_edge)
    maxMarchDist = dist_to_edge;
  int marchDist = 0;
  // traverse object and scan one primary bellow for object
  if (traverseFirstObject) {
    marchDist = traverseObject(x, y, dir, maxMarchDist); // = march distance to back of the last object.
    if (marchDist <= 0)
      return;
  }
   // check for valid path to point behind object
  if (getDistance(x, y, dir.first, true) == infinity)
    return;
  // scan for other objects on the jump line
  while(marchDist <= maxMarchDist) {
    int x_cur = x;
    int y_cur = y;
    if (sharedConfig_->debugCardinalSearch) {
      std::cout << "  -> Current backside of the object is at (" << x_cur << ", " << ny_-1-y_cur << ")\n";
    }
    int dist_to_next = traverseVoid(x, y, dir.second, maxMarchDist-marchDist);
  // #----------if the current object is the last object on the visible jump line----------#
    if (marchDist + dist_to_next > maxMarchDist) {
      if (sharedConfig_->debugCardinalSearch) {
        std::cout << "     This is the last object on the jumpline \n";
      }
      if (!checkForwards(x_cur, y_cur, dir.second, dist_to_next))
        ComputeDistanceBetweenSlopes(dir, x_cur, y_cur, primaryDist, secondaryDist+marchDist, prevVisibilityDist, dist_to_next, stopSlope);
      else
        ComputeDistanceBetweenSlopes(dir, x_cur, y_cur, primaryDist, secondaryDist+marchDist, prevVisibilityDist, dist_to_next);
      return;
    }
  // #--------else if the object is not the last object on the visible jump line---------#
    else {
      if (sharedConfig_->debugCardinalSearch) {
        std::cout << "     This is not the last object on the jumpline \n";
      }
      ComputeDistanceBetweenSlopes(dir, x_cur, y_cur, primaryDist, secondaryDist+marchDist, prevVisibilityDist, dist_to_next-1);
      int dist_to_back = traverseObject(x, y, dir, maxMarchDist-marchDist-dist_to_next);
      if (dist_to_back <= 0 || getDistance(x, y, dir.first, true) == infinity)
        return;
      marchDist += dist_to_next + dist_to_back;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::ComputeDistanceFromCarindal(searchdir dir, const int basePrimaryVisibility, const int baseSecondaryVisibility) {
  // print the input parameters of this function
  if (sharedConfig_->debugCardinalSearch || sharedConfig_->debugPivotCreation) {
    std::cout << "\n" << "==================== Search Direction - (" << cardir_to_string(dir.first) << ", " << cardir_to_string(dir.second) << ") ====================\n";
  }
  int x_pri = x_;
  int y_pri = y_;
  float blockSlope = (baseSecondaryVisibility + 0.5) / 0.5;
  int prevVisibilityDist = baseSecondaryVisibility;
  bool marchOver;
  bool outOfBound;
  bool marchOverSame = false;
  int visibilityDist;
// march over primary direction
  for (int primaryDist = 1;primaryDist <= basePrimaryVisibility;primaryDist++) {
    forceMove(x_pri, y_pri, dir.first);
    // march over secondary direction
    int x_sec = x_pri;
    int y_sec = y_pri;
    march_to_slope(dir.second, x_sec, y_sec, startPoint_, primaryDist, 0, blockSlope, visibilityDist, marchOver, outOfBound);
// Process result of the march over secondary direction
  // ------- marchOver: new line of sight --------
    if (marchOver && !marchOverSame) {
      marchOverSame = true;
      processMarchOver(dir, x_sec, y_sec, blockSlope, visibilityDist-prevVisibilityDist);
    }   
  // ------ marchOver: same line of sight --------
    else if (marchOver && marchOverSame) {}
  // ------ line of sight not reached --------
    // visibility increased while not marching over line of of sight
    else if (!marchOverSame && prevVisibilityDist < visibilityDist) {
      processSteepSlope(dir, x_sec, y_sec, visibilityDist-prevVisibilityDist);
    }
    // jump due to edge of the map
    else if (outOfBound) {}
    // possible normal jump
    else {
      if (visibilityDist != prevVisibilityDist) {
        processJump(dir, x_sec, y_sec, primaryDist, visibilityDist+1, blockSlope, visibilityDist, prevVisibilityDist);
      }
        //update blockslope
        blockSlope = (visibilityDist + 0.5) / (primaryDist + 0.5); 
        marchOverSame = false;
    }
    prevVisibilityDist = visibilityDist;
  }
  // after march finished check if boundary of map was reached in primary direction
  if(move(x_pri, y_pri, dir.first)) {
    processJump(dir, x_pri, y_pri, basePrimaryVisibility+1, 0, blockSlope, -1, prevVisibilityDist);
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::ComputeDistanceBetweenSlopes(searchdir dir, int x_start, int y_start, int primaryDist, int secondaryDist, int prevVisibilityDist, int gapWidth, float blockSlope) {
  if (sharedConfig_->debugCardinalSearch) {
    std::cout << "-----Computing sloped visibility at (" << x_start << ", " << ny_-1-y_start << ")-----\n";
    std::cout << "        primaryDist  = " << primaryDist << "\n";
    std::cout << "       secondaryDist = " << secondaryDist << "\n";
    std::cout << "       prevVisibDist = " << prevVisibilityDist << "\n";
    std::cout << "         gapWidth    = " << gapWidth << "\n";
    std::cout << "        blockSlope   = " << blockSlope << "\n";
  }
// ****************** creating new pivot ********************************************************
  int x_pivot = x_start;
  int y_pivot = y_start;
  float startSlope = (secondaryDist - 0.5)/(primaryDist -0.5);
  // if the point behind the object is visible
  if (secondaryDist >= primaryDist*startSlope) {
    int pivotPrimaryDist = primaryDist;
    bool createPivot = false;
    bool marchBeyond = false;
    // check if the next point on primary is also visible
    while(true) {
      if (advance(x_pivot, y_pivot, dir.first)) {
        pivotPrimaryDist++;
        if (secondaryDist >= pivotPrimaryDist*startSlope) {
          if (!checkBackwards(x_pivot, y_pivot, dir.second)) {
            marchBeyond = true;
            break;
          } 
        }
        else {
          forceMove(x_pivot, y_pivot, dir.first, -1);
          createPivot = true;
          break;
        }
      }
      else 
        break;
    }
    // create pivot 
    if (createPivot) {
      if (sharedConfig_->debugPivotCreation) {
        std::cout << "  -> created a pivot at (" << x_pivot << ", " << ny_-1-y_pivot << "): march up primary\n";
      }
      // create pivot in inverse search direction
      openSet_->push(Node{10, evaluateDistance(x_, y_, x_pivot, y_pivot), x_pivot, y_pivot, dir.second, dir.first, 0, 0, 1/startSlope, false});
    }
    // march beyond
    else if (marchBeyond) {
      if (sharedConfig_->debugPivotCreation) {
        std::cout << "  -> created a pivot at (" << x_pivot << ", " << ny_-1-y_pivot << "): march up primary and beyond\n";
      }
      // create pivot with inverse secondary direction
      openSet_->push(Node{10, evaluateDistance(x_, y_, x_pivot, y_pivot), x_pivot, y_pivot, dir.first, oppDirection(dir.second), 0, 0, -startSlope, false});
    }
  }
  // if the point behind the object is not visible
  else {
    forceMove(x_pivot,y_pivot, dir.first, -1);
    if (gScore_(x_pivot,y_pivot) != infinity) {
      if (sharedConfig_->debugPivotCreation) {
        std::cout << "  -> created a pivot at (" << x_pivot << ", " << ny_-1-y_pivot << "): march one step back on primary\n";
      }
      // create pivot in inverse search direction
      openSet_->push(Node{10, evaluateDistance(x_, y_, x_pivot, y_pivot), x_pivot, y_pivot, dir.second, dir.first, 0, 0, 1/startSlope, false});
    }
  }
// ****************** First secondary march ********************************************************
  int x_pri = x_start;
  int y_pri = y_start;
  // march start postition to a visble point behind the start slope
  while (secondaryDist < primaryDist*startSlope) {
    if (advance(x_pri, y_pri, dir.second))
      secondaryDist++;
    else 
      return;
    }
  // stop if there is not a vallid path to this start position
  if (getDistance(x_pri, y_pri, dir.first, true) == infinity)
    return;
  // assign distance value to first valid position
  if (secondaryDist >= primaryDist*startSlope) {
    gScore_(x_pri,y_pri) = evaluateDistance(x_pri,y_pri,x_,y_);
    cameFrom_(x_pri,y_pri) = startPoint_;
  }
  // initialization
  int visibilityDist;
  bool marchOverBlock;
  bool outOfBound;
  bool marchOverSame = false;
  if (blockSlope == infinity) {
    blockSlope = (secondaryDist + gapWidth + 0.5)/(primaryDist-0.5);
  }
  else {
    marchOverSame = true;
  }
  // effectively march over first direction
  int x_sec = x_pri;
  int y_sec = y_pri;
  march_to_slope(dir.second, x_sec, y_sec, startPoint_, primaryDist, secondaryDist, blockSlope, visibilityDist, marchOverBlock, outOfBound);

  if (marchOverBlock) {
    // object right below the jump line
    if (checkForwards(x_, y_, dir.first, dir.second, primaryDist-1, prevVisibilityDist+1)) {
      if (sharedConfig_->debugCardinalSearch) {
        std::cout << "  -> First secondary march: marched over block\n";
      }
      processMarchOver(dir, x_sec, y_sec, blockSlope, secondaryDist+visibilityDist-prevVisibilityDist);
    }
    // object bellow the jump line with at least one pixel inbetween
    else {
      if (sharedConfig_->debugCardinalSearch) {
        std::cout << "  -> First secondary march: marched over stop line of previous pivot\n";
      }
    }
  }
  else {
    // if hit same object as in previous step
    if (secondaryDist + visibilityDist - prevVisibilityDist <= 1) {
      if (sharedConfig_->debugCardinalSearch) {
        std::cout << "  -> First secondary march: marched against same object\n";
      }
      blockSlope = (secondaryDist + visibilityDist + 0.5) / (primaryDist + 0.5);
    }
    // extreme slope detected
    else { 
      if (sharedConfig_->debugCardinalSearch) {
        std::cout << "  -> First secondary march: extreme slope detected \n";
      }
      if (!outOfBound) {
        blockSlope = (secondaryDist + visibilityDist + 0.5) / (primaryDist + 0.5);
      }
      processSteepSlope(dir, x_sec, y_sec, secondaryDist+visibilityDist-prevVisibilityDist);
    }
  }
  prevVisibilityDist = secondaryDist + visibilityDist;
// ****************** Marching ********************************************************
  while (true) {
  //-----------------advance primary direction------------------------
    int advanceResult = advancePrimaryVisibility(dir, x_pri, y_pri, primaryDist, secondaryDist, startSlope); 
    // successful march without clipping
    if (advanceResult == 0) {
      gScore_(x_pri,y_pri) = evaluateDistance(x_pri,y_pri,x_,y_);
      cameFrom_(x_pri,y_pri) = startPoint_;
    }
    // successful march with clipping
    else if (advanceResult == 1) {
      gScore_(x_pri,y_pri) = evaluateDistance(x_pri,y_pri,x_,y_);
      cameFrom_(x_pri,y_pri) = startPoint_;
      processJump(dir, x_pri, y_pri, primaryDist, secondaryDist, blockSlope, -1, prevVisibilityDist, false);
      return;
    }
    // if march over primary direction was unsuccesfull
    else if (advanceResult == 2)
        break;
    else // advanceResult == 3
      return;
  //-----------------advance secondary directon------------------------
    x_sec = x_pri;
    y_sec = y_pri;
    march_to_slope(dir.second, x_sec, y_sec, startPoint_, primaryDist, secondaryDist, blockSlope, visibilityDist, marchOverBlock, outOfBound);

  // __marchOver: new line of sight__
    if (marchOverBlock && !marchOverSame) {
      marchOverSame = true;
      processMarchOver(dir, x_sec, y_sec, blockSlope, secondaryDist+visibilityDist-prevVisibilityDist);
    } 
  // __marchOver: same line of sight or stop line __
    else if (marchOverBlock) {

    }
  // __line of sight not reached__
    // -> jump due to edge of the map
    else if (outOfBound) {}
    else {
    // -> possible jump
      if (secondaryDist + visibilityDist != prevVisibilityDist) {
        processJump(dir, x_sec, y_sec, primaryDist, secondaryDist+visibilityDist+1, blockSlope, visibilityDist, prevVisibilityDist);
      }
      blockSlope = (secondaryDist + visibilityDist + 0.5) / (primaryDist + 0.5);
      marchOverSame = false;
    }
    prevVisibilityDist = secondaryDist + visibilityDist;
  }
  // ****************** After March ********************************************************
  if (sharedConfig_->debugCardinalSearch) {
    std::cout << "  -> position after march: (" << x_pri << ", " << ny_-1-y_pri << ")\n";
  }
  processJump(dir, x_pri, y_pri, primaryDist, secondaryDist, blockSlope, -1, prevVisibilityDist);
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
} // namespace vbd
#endif // SOLVER_ORIGIN_HPP