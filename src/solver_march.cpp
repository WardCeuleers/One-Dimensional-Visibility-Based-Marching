#ifndef SOLVER_PIVOTS_CPP
#define SOLVER_PIVOTS_CPP

#include "solver/solver.hpp"
#include "solver_utils.cpp"
#include <iostream>

namespace vbd {
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::createNewPivot(const point pivot, const point parent, const cardir primaryDir, const cardir secondaryDir, float slope, bool adaptedSlope) {
  // if a distance from another pivot was set on first secondary
  if (getDistance(pivot.first, pivot.second, secondaryDir) != infinity) {
    return;
  }
// 1) Add pivot to the pivot list
  pivots_[nb_of_pivots_] = pivot;
  ++nb_of_pivots_;
  pivotDir_(pivot.first, pivot.second) = {primaryDir, secondaryDir};
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - direction: " << cardir_to_string(primaryDir) << ", " << cardir_to_string(secondaryDir) << std::endl;
    std::cout << "  - pivot: " << pivot.first << ", " << ny_-1-pivot.second << std::endl;
    std::cout << "  - parent: " << parent.first << ", " << ny_-1-parent.second << std::endl;
    std::cout << "  - slope: " << slope << std::endl;
    if (adaptedSlope) {
      point slopeFromParentOrigin = coordinatesAt(slopeOrigin_(parent.first, parent.second));
      std::cout << "  - slopeFromParentOrigin: " << slopeFromParentOrigin.first << ", " << ny_-1-slopeFromParentOrigin.second << std::endl;
    }
  }
  if (adaptedSlope)
    slopeOrigin_(pivot.first, pivot.second) = slopeOrigin_(parent.first, parent.second);
  else
    slopeOrigin_(pivot.first, pivot.second) = indexAt(parent.first, parent.second);

  bool negSlope = slope < 0;
  // ----- add slope search -----
  if (negSlope) {
    slope = -slope;
    addNextSlopePrimary(pivot.first, pivot.second, oppDirection(secondaryDir), primaryDir, 0, 0, slope, false, true);
    addNextStraightPrimary(gScore_(pivot.first, pivot.second), pivot.first, pivot.second, primaryDir, secondaryDir, 0, false);
  }
  else {
    addNextSlopePrimary(pivot.first, pivot.second, primaryDir, secondaryDir, 0, 0, slope, false, true);
  }
  // ----- add edge search -----
  int x = pivot.first, y = pivot.second;
  // if you can move to the correct position
  if (move(x, y, secondaryDir)) {
    if (negSlope) {
      gScore_(x, y) = gScore_(pivot.first, pivot.second) + 1;
      cameFrom_(x, y) = pivot;
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added secondary node at: " << x << ", " << ny_-1-y << std::endl;
      }
      openSet_->push(Node{0, gScore_(pivot.first, pivot.second)+1, x, y, primaryDir, secondaryDir , 0, 1, 0, true});
    }
    // add node to heap
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added first primary straight point at: " << x << ", " << ny_-1-y << std::endl;
    }
    openSet_->push(Node{1, gScore_(pivot.first, pivot.second)+1, x, y, secondaryDir, oppDirection(primaryDir) , 1, 0, 0, true});
    // add block corner if needed
    if (reverse(x, y, primaryDir)) {
      forceMove(x, y, secondaryDir, -1);
      point corner = {x, y};
      // set reference of the blockSlope
      blockCorners_(x, y) = pivot;
      // set block corner for first secondary search
      forceMove(x, y, secondaryDir);
      blockCorners_(x, y) = corner;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::addNextStraightPrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, bool OnVisibleSide) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, 0);
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - direction: " << cardir_to_string(primaryDir) << ", " << cardir_to_string(secondaryDir) << std::endl;
    std::cout << "  - pivot: " << pivot.first << ", " << ny_-1-pivot.second << std::endl;
    std::cout << "  - onVisibleSide: " << (OnVisibleSide ? "TRUE":"FALSE") << std::endl;
  }
  if (cameFrom_(x, y) != pivot) {
    if (OnVisibleSide)
      return;
  }
  else {
    OnVisibleSide = true;
  }
  //move the point along the primary axis
  if (!move(x, y, primaryDir)) {
    return;
  }
  // if moved to an occupied point
  if (sharedOccupancyField_->get(x,y)) {
    // create occupied node to search in secondary dir
    openSet_->push(Node{4, distance+1, x, y, primaryDir, secondaryDir, primaryDist + 1, 0, 0, false});
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added an occupied node at: " << x << ", " << ny_-1-y << std::endl;
    }
  }
  //if moved to a free point
  else {
    openSet_->push(Node{1, distance+1, x, y, primaryDir, secondaryDir, primaryDist + 1, 0, 0, OnVisibleSide});
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added new primary straight point at: " << x << ", " << ny_-1-y << std::endl;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::setSlopePrimaryDistance(const double& distance, const int& x, const int& y, const point& pivot, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, const int& secondaryDist) {
  // if no distance assigned yet
  if (gScore_(x, y) == infinity) {
    // update distance and relation to the parent
    gScore_(x, y) = distance;
    cameFrom_(x, y) = pivot;
    return true;
  }
  // if already distance assigned
  else {
    // if closer to current pivot then to other
    if (closerToCurrent(x, y, pivot, cameFrom_(x, y))) {
      gScore_(x, y) = distance;
      cameFrom_(x, y) = pivot;
      return true;
    }
    // if closer to other 
    else {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> moved to a visble point to which no distance could be assigned,the beginning of a switched search was set at: " << x << ", " << ny_-1-y << std::endl;
      }
      addInitialSwitchedPrimary(pivot, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
      return false;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::addNextSlopePrimary(int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, int secondaryDist, float& slope, bool fromVisiblePoint, bool slopeFromParent) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  point parent = coordinatesAt(slopeOrigin_(pivot.first, pivot.second));
  uint8_t type = slopeFromParent ? 3 : 2;
  int parentPrimaryDist, parentSecondaryDist;
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - direction: " << cardir_to_string(primaryDir) << ", " << cardir_to_string(secondaryDir) << std::endl;
    std::cout << "  - fromVisiblePoint: " << (fromVisiblePoint ? "TRUE":"FALSE") << std::endl;
    std::cout << "  - slope: " << slope << std::endl;
    std::cout << "  - pivot: " << pivot.first << ", " << ny_-1-pivot.second << std::endl;
    if (slopeFromParent) {
      std::cout << "  - parent: " << parent.first << ", " << ny_-1-parent.second << std::endl;
    }
  }
  // ********************  from visible primary point ********************
  if (fromVisiblePoint) {
    // if the starting point doesn't belong to the pivot anymore
    if (cameFrom_(x, y) != pivot) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> the startpoint changed ownership to " << cameFrom_(x, y).first << ", " << ny_-1-cameFrom_(x, y).second << " the beginning of a switched search was set at: " << x << ", " << ny_-1-y << std::endl;
      }
      addInitialSwitchedPrimary(pivot, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
      return true;
    }
    //  ----- set initial parameters and move ----- 
    bool fromBehindObject = checkBackwards(x, y, secondaryDir);
    bool fromBeforeObject = checkForwards(x, y, secondaryDir);
    if (!move(x, y, primaryDir)) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> moved out of bounds in primaryDir" << std::endl;
      }
      return true;
    }
    primaryDist ++;
    double distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
    // -----   if moved from visblepoint to an occupied point -----  
    if (sharedOccupancyField_->get(x, y)) {      
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
      }
      openSet_->push(Node{4, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, false});
      return true;
    }
    // if moved next to an object -> check if slope needs to be updated
    bool nextToObject = checkBackwards(x, y, secondaryDir);
    if (!fromBehindObject && nextToObject) {
      float blockSlope = calcSlope(secondaryDir, primaryDist - 0.5, secondaryDist - 0.5);
      if (slopeFromParent) {
        slopeFromParent = false;
        type = 2;
        slope = blockSlope;
      }
      else if (smallerSlope(primaryDir, blockSlope, slope)) {
        slope = blockSlope;
      }
    }
    parentPrimaryDist = slopeFromParent ? evaluateCardinalDistance(primaryDir, parent, {x, y}) : primaryDist;
    parentSecondaryDist = slopeFromParent ? evaluateCardinalDistance(secondaryDir, parent, {x, y}) : secondaryDist;
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "  - parentPrimaryDist: " << parentPrimaryDist << std::endl;
      std::cout << "  - parentSecondaryDist: " << parentSecondaryDist << std::endl;
    }
    // -----  if moved from a visble point to a visble point ----- 
    if (onVisibleSide(secondaryDir, parentPrimaryDist, parentSecondaryDist, slope, slopeFromParent)) {
      if (setSlopePrimaryDistance(distance, x, y, pivot, primaryDir, secondaryDir, primaryDist, secondaryDist)) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new visible primary point at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{type, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, true});
        if (fromBehindObject && !nextToObject) {
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> created a node to create a pivot with negative slope at: " << x << ", " << ny_-1-y << std::endl;
          }
          openSet_->push(Node{6, distance, x, y, primaryDir, oppDirection(secondaryDir), secondaryDist, primaryDist, -slope, false});
        }
      }
      return true;
    }
    //  ----- if moved from a visble point to a non-visble point ----- 
    else {
      if (!fromBeforeObject) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new non-visible primary point at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{type, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, false});
      }
      forceMove(x, y, primaryDir, -1);
      // if there was an object on one of the cells below on secondary
      if (fromBehindObject || nextToObject) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{6, gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y), x, y, secondaryDir, primaryDir, secondaryDist, primaryDist, slope, false});
      }
      // if there was an object in front -> primary is being squeezed out of existance
      else if (fromBeforeObject) {
        if (move(x, y, secondaryDir)) {
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
          }
          openSet_->push(Node{4, gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y), x, y, primaryDir, secondaryDir, primaryDist-1, secondaryDist+1, 0, false});
        }
        return false;
      }
    }
    return true;
  }
  // ******************** from non-visible primary point ********************
  else {
    //  ----- set initial parameters and move -----
    bool fromInsideObject = sharedOccupancyField_->get(x, y);
    if (!move(x, y, secondaryDir)) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> moved out of bounds in secondaryDir" << std::endl;
      }
      return true;
    }
    if (primaryDist != 0) {
      if (checkBackwards(x, y, primaryDir)) {
        if (!slopeFromParent) {
          blockCorners_(x, y) = nullPoint_;
          forceMove(x, y, primaryDir, -1);
          blockCorners_(x, y) = nullPoint_;
        }
        return true;
      }
    }
    secondaryDist ++;
    double distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
    parentPrimaryDist = slopeFromParent ? evaluateCardinalDistance(primaryDir, parent, {x, y}) : primaryDist;
    parentSecondaryDist = slopeFromParent ? evaluateCardinalDistance(secondaryDir, parent, {x, y}) : secondaryDist;
    // ----- if moved from a non-visible point to a visible point -----
    if (onVisibleSide(secondaryDir, parentPrimaryDist, parentSecondaryDist, slope, slopeFromParent)) {
      // if moved to an occupied point
      if (sharedOccupancyField_->get(x, y)) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{4, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, false});
      }
      // if moved to a visible point
      else if (setSlopePrimaryDistance(distance, x, y, pivot, primaryDir, secondaryDir, primaryDist, secondaryDist)) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new visible primary point at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{type, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, true});
      }
      else if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> moved to a visble point to which no distance could be assigned" << std::endl;
      }
      return false;
    }
    // ----- if moved from a non-visible point to a non-visible point -----
    else {
      // if the new point is occupied
      if (sharedOccupancyField_->get(x, y)) {
        if (checkBackwards(x, y, primaryDir)) {
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> ended the search as it was in an object and next to an object " << std::endl;
          }
          return false;
        }
        else {
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> added next non-visble primary point inside an object at: " << x << ", " << ny_-1-y << std::endl;
          }
          openSet_->push(Node{type, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, false});
        }
      }
      // if the new point is free
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added next non-visble primary point at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{type, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, false});
        if (fromInsideObject) {
          forceMove(x, y, primaryDir, -1);
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
          }
          openSet_->push(Node{6, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, false});
        }
      }
    }
    return false;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::advanceSecondaryNode(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist,  int& secondaryDist, const float& slope, const bool& fromPivotSlope, bool onVisibleSide, bool checkStartBound) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  // if start changed ownership
  if (onVisibleSide && cameFrom_(x, y) != nullPoint_ && cameFrom_(x, y) != pivot) {
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> the startpoint changed ownership " << std::endl;
    }
    return false;
  }
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - direction: " << cardir_to_string(primaryDir) << ", " << cardir_to_string(secondaryDir) << std::endl;
    std::cout << "  - pivot: " << pivot.first << ", " << ny_-1-pivot.second << std::endl;
    std::cout << "  - startSlope: " << slope << std::endl;
    std::cout << "  - fromPivotSlope: " << (fromPivotSlope ? "TRUE":"FALSE") << std::endl;
  }
  point startblock = blockCorners_(x, y);
  bool moveBoundary = (checkStartBound && startblock != nullPoint_ && blockCorners_(startblock) == pivot);
  bool outOfBound = false;
  if (advance(x, y, secondaryDir, outOfBound)) {
  // if the advance was succesfull
  //****************************************************************************************************************
    distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
    secondaryDist += 1;
    point nextblock = blockCorners_(x, y);
    point blockSource = blockCorners_(nextblock);
    // Check if the cell was no a boundary after the move
    bool movedOnBoundary = false;
    if (nextblock != nullPoint_ && blockSource == pivot) {
        movedOnBoundary = true;
    }
    // effectivly move the boundary if moveboundary is true and the new point does not have a boundary from another search
    if (moveBoundary && nextblock == nullPoint_) {
      blockCorners_(x, y) = startblock; 
    }
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "  - moveBoundary: " << (moveBoundary ? "TRUE":"FALSE") << std::endl;
      std::cout << "  - movedOnBoundary: " << (movedOnBoundary ? "TRUE":"FALSE") << std::endl;
    }
    // process boundary point if on boundary
    //------------------------------------------------------------------------------------------------------
    if (moveBoundary) {
      if (!processBoundaryPoint(x, y, distance, pivot, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, calcBlockSlope(primaryDir, secondaryDir, pivot, startblock), startblock, fromPivotSlope, false)) {
        return false;
      }
    }
    else if (movedOnBoundary) {
      if (!processBoundaryPoint(x, y, distance, pivot, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, calcBlockSlope(primaryDir, secondaryDir, pivot, nextblock), nextblock, fromPivotSlope, false)) {
        return false;
      }
      moveBoundary = true;
    }
    // update information of the new visible point
    //------------------------------------------------------------------------------------------------------
    // if the new point has no distance yet
    if (gScore_(x, y) == infinity) {
      if (!movedOnBoundary && moveBoundary) {
        blockCorners_(x, y) = startblock;
      }
      gScore_(x, y) = distance;
      cameFrom_(x, y) = pivot;
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> updated the secondary point to: " << x << ", " << ny_-1-y << std::endl;
      }
      return true;
    }
    // if it already has a distance
    else {
      auto [x_other, y_other] = cameFrom_(x, y);
      if (distance < gScore_(x_other, y_other) + evaluateDistance(x_other, y_other, x, y)) {
        gScore_(x, y) = distance;
        cameFrom_(x, y) = pivot;
        return true;
      }
      else
        return false;
    }
  }
// if the advance failed
//****************************************************************************************************************
  // if out of bounds
  else if (outOfBound) {
    return false;
  }
  // if hit an object
  else {
  // set block for march on next primary
    int x_adj = x, y_adj = y;
    // set block if next primary element is free and there is a path to it. 
    if (advance(x_adj, y_adj, primaryDir) && !checkBackwards(x_adj, y_adj, secondaryDir) && cameFrom_(x_adj, y_adj) == nullPoint_) {
      float blockSlope = calcBlockSlope(primaryDir, secondaryDir, pivot, {x, y});
      // if there is a valid path around the corner of the newly discovered object
      if (checkValidPathBack(primaryDir, secondaryDir, x_adj, y_adj, primaryDist+1, slope, blockSlope, pivot, coordinatesAt(slopeOrigin_(pivot)))) {
        // if moveBoundary was true and the slope coming from the moveblock was smaller than the slope of the new occupied point
        if (moveBoundary && smallerSlope(primaryDir, blockSlope, calcBlockSlope(primaryDir, secondaryDir, pivot, startblock))) {
          blockCorners_(x_adj, y_adj) = startblock;
        }
        else {
          blockCorners_(x_adj, y_adj) = {x, y};
          blockCorners_(x, y) = pivot;
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> set start of a boundary of pivot " << pivot.first << ", " << ny_-1-pivot.second << " for blockcorner: " << x << ", " << ny_-1-y << std::endl;
          }
        }
      }
      else {
        blockCorners_(x_adj, y_adj) = {x, y};
        blockCorners_(x, y) = pivot;
      } 
    }
  // determine if you need to continue as an occupied node
    // if on moveboundary or a boundary was discovered inside the object
    if (moveBoundary || blockCorners_(x, y) != nullPoint_) {
      return false;
    }
    // if on the first secondary of the edge search
    else if (primaryDist == 1 && pivotDir_(pivot).second != secondaryDir) {
      return false;
    }
    // if the prev point on primary is occupied
    else if (checkBackwards(x, y, primaryDir) || checkBackwards(x, y, primaryDir, secondaryDir)) {
      return false;
    }
    // else continue secondary search as occupied node
    else {
      distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
      openSet_->push(Node{4, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist+1, 0, false});
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> updated the secondary to an occupied node at: " << x << ", " << ny_-1-y << std::endl;
      }
      return false;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::processBoundaryPoint(int& x, int& y, const double& distance, const point& pivot, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, const int& secondaryDist, const float& slope, float blockSlope, point& block, const bool& fromPivotSlope, bool adaptedSlope) {
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - blockSlope: " << blockSlope << std::endl;
  }
  // if not on first primary after block and there is an occupied point next to the block
  if (!adaptedSlope && evaluateCardinalDistance(primaryDir, block.first, block.second, x, y)==1 && checkBackwards(x, y, primaryDir)) {
    float newBlockSlope = calcSlope(secondaryDir, primaryDist-0.5, secondaryDist-0.5);
    if (smallerSlope(primaryDir, blockSlope, newBlockSlope)) {
      blockSlope = newBlockSlope;
      forceMove(x, y, primaryDir, -1);
      blockCorners_(x, y) = pivot;
      block = {x, y};
      forceMove(x, y, primaryDir);
      blockCorners_(x, y) = block;
    }
  }
  // if the point is visible
  if (!onVisibleSide(secondaryDir, primaryDist, secondaryDist, blockSlope)) {
    // if the point back on primary is free and the one below on secondary is occupied
    if (!checkBackwards(x, y, primaryDir) && checkBackwards(x, y, primaryDir, secondaryDir)) {
      // check for valid path between primarySlope and blockSlope
      if (fromPivotSlope ? checkValidPathBack(primaryDir, secondaryDir, x, y, primaryDist, slope, blockSlope, pivot) 
                         : checkValidPathBack(primaryDir, secondaryDir, x, y, primaryDist, slope, blockSlope, pivot, coordinatesAt(slopeOrigin_(pivot)))) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{6, distance, x, y, secondaryDir, oppDirection(primaryDir), 0, 0, -blockSlope, false});
      }
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot with adapted slope at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{6, distance, x, y, secondaryDir, oppDirection(primaryDir), 0, 0, -slope, !fromPivotSlope});
      }
    }
    return true;
  }
  // if the current point is not visible
  else {
    // if it is next to an object
    if (checkBackwards(x, y, primaryDir) || checkBackwards(x, y, primaryDir, secondaryDir)) {
      forceMove(x, y, secondaryDir, -1);
      if (fromPivotSlope ? checkValidPathBack(primaryDir, secondaryDir, x, y, primaryDist, slope, blockSlope, pivot) 
                         : checkValidPathBack(primaryDir, secondaryDir, x, y, primaryDist, slope, blockSlope, pivot, coordinatesAt(slopeOrigin_(pivot)))) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{6, distance, x, y, primaryDir, secondaryDir, 0, 0, blockSlope, false});
      }
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot with adapted slope at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{6, distance, x, y, primaryDir, secondaryDir, 0, 0, slope, !fromPivotSlope});
      }
      forceMove(x, y, secondaryDir);
    }
    // set block for march on next primary
    if (move(x, y, primaryDir)) {
      // if the boundary is on a null point and the corner does 
      if (cameFrom_(x, y) == nullPoint_ || slopeOrigin_(cameFrom_(x, y)) == indexAt(pivot.first, pivot.second)) {
        blockCorners_(x, y) = block;
      }
    }
    return false;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::advanceOccupiedNode(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist,  int& secondaryDist) {
  point parent = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - direction: " << cardir_to_string(primaryDir) << ", " << cardir_to_string(secondaryDir) << std::endl;
    std::cout << "  - parent: " << parent.first << ", " << ny_-1-parent.second << std::endl;
    std::cout << "  - primaryDist: " << primaryDist << std::endl;
    std::cout << "  - secondaryDist: " << secondaryDist << std::endl;
  }
  // if the occupied point is not next to a sibling -> stop
  if (secondaryDist != 0 && !prevFromSibling(x, y, primaryDir, parent)) {
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> the start point was, not next to a sibling" << std::endl;
    }
    return false;
  }
  // move to next point
  if (!move(x, y, secondaryDir)) {
    return false;
  }
  // stop the occupied search if new point is on a boundary
  if (blockCorners_(x, y) != nullPoint_ && blockCorners_(blockCorners_(x, y)) == parent){
    // cleanup boundary if inside a object 
    blockCorners_(x, y) = nullPoint_;
    return false;
  }
  // ---------- if the point after the move is occupied -----------
  if (sharedOccupancyField_->get(x, y)) {
    // if the prev point on primary is occupied -> stop
    if (checkBackwards(x, y, primaryDir)) {
      return false;
    }
    // if the prev point on primary is free and still next to points assiged to its pivot -> continue occupied search
    else {
      cameFrom_(x, y) = parent;
      distance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
      secondaryDist += 1;
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added another occupied node at: " << x << ", " << ny_-1-y << std::endl;
      }
      return true;
    }
  }
  // ---------- if the point after the move is free -----------
  else {
    // if there is an object blocking the path arround the corner
    if (checkBackwards(x, y, primaryDir)) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> the new point was free, but the path around the corner was blocked" << std::endl;
      }
      return false;
    }
    // // if not next to a sibling
    else if (!prevFromSibling(x, y, primaryDir, parent)) {
      point corner = getPrevPoint(x, y, secondaryDir);
      point other = cameFrom_(getPrevPoint(x, y, primaryDir));
      if (other.first != x && other.second != y) {
        blockCorners_(x, y) = corner;
        forceMove(x, y, secondaryDir, -1);
        blockCorners_(x, y) = other;
      }
      return false;
    }
    // if there is a path to the point behind the object
    else {
      secondaryDist += 1;
      distance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
      float blockSlope = calcSlope(secondaryDir, primaryDist-0.5, secondaryDist-0.5);
      if (onVisibleSide(secondaryDir, primaryDist, secondaryDist, blockSlope, false)) {
        // if no distance assigned yet or if closer to current then to other
        if (gScore_(x, y) == infinity || closerToCurrent(x, y, parent, cameFrom_(x, y))) {
          // update distance and relation to the parent
          gScore_(x, y) = distance;
          cameFrom_(x, y) = parent;
          // if their is a blockslope coming around the corner
          point block = blockCorners_(getPrevPoint(x, y, primaryDir));
          if (block != nullPoint_ && blockCorners_(block) == parent) {
            float stopSlope = calcBlockSlope(primaryDir, secondaryDir, parent, block);
            if (!checkValidPathFront(primaryDir, secondaryDir, x, y, secondaryDist, blockSlope, stopSlope, parent)) {
              if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
                std::cout << "-> created a node to create a pivot with adapted slope at: " << x << ", " << ny_-1-y << std::endl;
              }
              openSet_->push(Node{6, distance, x, y, secondaryDir, primaryDir, primaryDist, secondaryDist, stopSlope, true});
              return false;
            }
          }
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> added a first visible pivot primary point at: " << x << ", " << ny_-1-y << std::endl;
          }
          openSet_->push(Node{2, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, blockSlope, true});
        }
      }
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added a first non-visible pivot primary point at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{2, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, blockSlope, false});
        forceMove(x, y, primaryDir, -1);
        // if the pivot location is on a boundary
        if (blockCorners_(x, y) != nullPoint_ && blockCorners_(blockCorners_(x, y)) == parent) {
          float stopSlope = calcBlockSlope(primaryDir, secondaryDir, parent, blockCorners_(x, y));
          if (!checkValidPathFront(primaryDir, secondaryDir, x, y, secondaryDist, blockSlope, stopSlope, parent)) {
            if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
              std::cout << "-> created a node to create a pivot with adapted slope at: " << x << ", " << ny_-1-y << std::endl;
            }
            openSet_->push(Node{6, distance, x, y, secondaryDir, primaryDir, primaryDist, secondaryDist, stopSlope, true});
            return false;
          }
        }
        // if no blockcorner or a valid path
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{6, distance, x, y, secondaryDir, primaryDir, primaryDist, secondaryDist, blockSlope, false});
      }
      return false;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::addInitialSwitchedPrimary(const point& pivot, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, int secondaryDist) {
  bool outOfBound = false;
  bool firstMarch = true;
  forceMove(x, y, primaryDir, -1);
  primaryDist --;
  while (advance(x, y, secondaryDir, outOfBound)) {
    secondaryDist ++;
    // if marched on the zone of another pivot
    if (cameFrom_(x, y) != pivot) {
      if (cameFrom_(x, y) == nullPoint_ || !closerToCurrent(x, y, pivot, cameFrom_(x, y))) {
        break;
      }
    }
    // if marched on a boundary of the pivot
    else if (blockCorners_(blockCorners_(x, y)) == pivot) {
      break;
    }
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added secondary march at " << x << ", " << ny_-1-y << std::endl;
    }
    openSet_->push(Node{0, gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y), x, y, secondaryDir, primaryDir, secondaryDist, primaryDist, 0, false});
    firstMarch = false;
  }
  if (outOfBound)
    return;
  else if (firstMarch && sharedOccupancyField_->get(x, y))
    return;
  else if (sharedOccupancyField_->get(x, y)) {
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added occupied node at " << x << ", " << ny_-1-y << std::endl;
    }
    openSet_->push(Node{4, gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y), x, y, secondaryDir, primaryDir, secondaryDist+1, primaryDist, 0, false});
    if (blockCorners_(x, y) == pivot) {
      blockCorners_(x, y) = nullPoint_;
      if (move(x, y, primaryDir)) {
        blockCorners_(x, y) = nullPoint_;
      }
    }
  }
  // found a point on a different camefrom
  else {
    // if on a point that has not distance assigned yet
    if (gScore_(x, y) == infinity || cameFrom_(x, y) == pivot) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added first switched primary at " << x << ", " << ny_-1-y << std::endl;
      }
      openSet_->push(Node{5, std::nextafter(gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y), infinity), x, y, secondaryDir, primaryDir, secondaryDist, primaryDist, 0, firstMarch});
    }
    // if on a point that belongs to another pivot
    else if (!firstMarch && cameFrom_(x, y) != nullPoint_) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> started a second switched primary at " << x << ", " << ny_-1-y << std::endl;
      }
      addInitialSwitchedPrimary(pivot, x, y, secondaryDir, primaryDir, secondaryDist, primaryDist);
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::addNextSwitchedPrimary(int x, int y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist, const int& secondaryDist, bool firstMarch) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  // if the current point is on a boundary from the pivot current pivot
  if (blockCorners_(blockCorners_(x, y)) == pivot) {
    float blockSlope = calcBlockSlope(pivotDir_(pivot).first, pivotDir_(pivot).second, pivot, blockCorners_(x, y));
    // only adapt to the pivot search when the next primary is non visible
    if (!onVisibleSide(secondaryDir, primaryDist+1, secondaryDist, blockSlope)) {
      // remove the boundary points
      int x_block = x, y_block = y;
      blockCorners_(x_block, y_block) = nullPoint_;
      if (advance(x_block, y_block, primaryDir)) {
        blockCorners_(x_block, y_block) = nullPoint_;
        if (advance(x_block, y_block, secondaryDir)) {
          blockCorners_(x_block, y_block) = nullPoint_;
        }
      }
      x_block = x, y_block = y;
      if (advance(x_block, y_block, secondaryDir)) {
        blockCorners_(x_block, y_block) = nullPoint_;
      }
      // chane into pivot search
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> Switched primary on it's own boundary, thus changed the point into a pivot search " << std::endl;
      }
      addNextSlopePrimary(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, blockSlope, false, false);
      return false;
    }
  }
  // check if current point belongs to the pivot
  if (cameFrom_(x, y) != pivot) {
    if (!firstMarch) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> started a new switched primary because it hit another pivot zone at " << x << ", " << ny_-1-y << std::endl;
      }
      addInitialSwitchedPrimary(pivot, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
    }
    return false;
  }
  // move to the next point
  bool outOfBounds = false;
  if (advance(x, y, primaryDir, outOfBounds)) {
    double distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
    // if not on a boundary or on the visible side of the boundary
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added switched primary at " << x << ", " << ny_-1-y << std::endl;
    }
    openSet_->push(Node{5, std::nextafter(distance, infinity), x, y, primaryDir, secondaryDir, primaryDist+1, secondaryDist, 0, false});
    return true;
  }
  else if (!outOfBounds) {
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added two occupied nodes at " << x << ", " << ny_-1-y << std::endl;
    }
    double distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
    openSet_->push(Node{4, distance, x, y, primaryDir, secondaryDir, primaryDist+1, secondaryDist, 0, false});
    openSet_->push(Node{4, distance, x, y, secondaryDir, primaryDir, secondaryDist, primaryDist+1, 0, false});
    if (blockCorners_(x, y) == pivot) {
      blockCorners_(x, y) = nullPoint_;
      if (move(x, y, secondaryDir)) {
        blockCorners_(x, y) = nullPoint_;
      }
    }
  }
  return true;
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
} // namespace vbd

#endif // SOLVER_PIVOTS_HPP