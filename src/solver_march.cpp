#ifndef SOLVER_PIVOTS_CPP
#define SOLVER_PIVOTS_CPP

#include "solver/solver.hpp"
#include "solver_utils.cpp"
#include <iostream>

namespace vbd {
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::createNewPivot(const point pivot, const point parent, const cardir primaryDir, const cardir secondaryDir, const float slope) {
// 1) Add pivot to the pivot list
    pivots_[nb_of_pivots_] = pivot;
    ++nb_of_pivots_;
    secondaryDir_(pivot) = secondaryDir;
// 2) Add first primary straight point to the heap
//--------------------------------------------------------------------------------
  int x = pivot.first;
  int y = pivot.second;
  // if you can move to the correct position
  if (move(x, y, secondaryDir)) {
    // add distance and parent
    gScore_(x, y) = gScore_(pivot.first, pivot.second)+1;
    cameFrom_(x, y) = pivot;
    // add node to heap
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added first primary straight point at: " << x << ", " << ny_-1-y << std::endl;
    }
    openSet_->push(Node{1, gScore_(pivot.first, pivot.second)+1, x, y, secondaryDir, oppDirection(primaryDir) , 1, 0, 0, false});
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
// 3a) in case of a positive slope -> add first primary slope point to the heap
//--------------------------------------------------------------------------------
  if (slope >= 0) {
    x = pivot.first; y = pivot.second;
    bool outOfBound = false;
    if(advance(x, y, primaryDir, outOfBound)) {
      double distance = gScore_(parent) + evaluateDistance(parent.first, parent.second, x, y);
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "    looking for first parent primary " << std::endl;
      }
      addNextParentSlopePrimary(distance, x, y, primaryDir, secondaryDir, 1, 0, slope, false);
    }
    else if (outOfBound) {
      return;
    }
    else {
      double distance = gScore_(parent) + evaluateDistance(parent.first, parent.second, x, y);
      openSet_->push(Node{4, distance, x, y, primaryDir, secondaryDir, 1, 0, slope, false});
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
      }
    }
  }
// 3b) in case of a negative slope -> add another straight search 
//--------------------------------------------------------------------------------
  else {
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "    looking for other first secondary at pivot with negative slope " << std::endl;
    }
    x = pivot.first; y = pivot.second;
    Node node = {5, gScore_(pivot), x, y, primaryDir, oppDirection(secondaryDir), 0, 0, -slope, false};
    if (advancePivotSearch(node.distance, node.x, node.y, node.primaryDir, node.secondaryDir, node.primaryDist, node.secondaryDist, node.slope, node.state)) {
      openSet_->push(node);
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::addNextStraightPrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, 0);
  bool outOfBound = false;
// if you can advance on primary
//---------------------------------------------------------------
  if (advance(x, y, primaryDir, outOfBound)) {
    // if the point already has a distance assigned
    if (gScore_(x, y) != infinity) {
      if (closerToCurrent(x, y, pivot, cameFrom_(x, y))) {
        gScore_(x, y) = distance + 1;
        cameFrom_(x, y) = pivot;
      }
      return;
    }
    // if new point has no distance yet
    gScore_(x, y) = distance + 1;
    cameFrom_(x, y) = pivot;
    // add new node to the heap
    openSet_->push(Node{1, distance+1, x, y, primaryDir, secondaryDir, primaryDist + 1, 0, 0, false});
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added new primary straight point at: " << x << ", " << ny_-1-y << std::endl;
    }
  }
// if the advance on primary failed
//---------------------------------------------------------------
  // if advance failed because out of bounds
  else if (outOfBound) {
    return;
  }
  // if advance failed because of an object
  else {
    // create occupied node to search in secondary dir
    cameFrom_(x, y) = pivot;
    openSet_->push(Node{4, distance+1, x, y, primaryDir, secondaryDir, primaryDist + 1, 0, 0, false});
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
    }
    // set block corner for search in opp direction of secondary dir
    if (reverse(x, y, secondaryDir)) {
      blockCorners_(x, y) = getNeighbour(x, y, secondaryDir);
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
void Solver::addNextPivotSlopePrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, int secondaryDist, const float& slope, bool onVisiblePoint) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  // ----- advance in correct direction -----
  if (onVisiblePoint) {
    if (move(x, y, primaryDir))
      primaryDist += 1;
    else
      return;
  }
  else {
    if (move(x, y, secondaryDir))
      secondaryDist += 1;
    else
      return;
  }
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - pivot: " << pivot.first << ", " << ny_-1-pivot.second << std::endl;
    std::cout << "  - slope: " << slope << std::endl;
    if (onVisiblePoint) {
      std::cout << "  - This node was on the visible side" << std::endl;
    }
    else {
      std::cout << "  - This node was on the non visible side" << std::endl;
    }
  }
  // ----- process the new point -----
  while (true) {
    double newDistance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
  // if new point is occupied + the point above is occupied-> create occupied node search
    if (sharedOccupancyField_->get(x, y)) {
      if (!checkBackwards(x, y, primaryDir)) {
        cameFrom_(x, y) = pivot;
        openSet_->push(Node{4, newDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, false});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
        }
      }
      break;
    }
  // if the point is not visible
    else if (primaryDist*slope >= secondaryDist) {
      // if the non visible point came from a march on primary distance -> also march one in secondary
      if (onVisiblePoint) {
        if (move(x, y, secondaryDir)) {
            secondaryDist += 1;
            onVisiblePoint = false;
            if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
              std::cout << "-> the new point was on the parent side and free, thus the algorithm looked one cell further" << std::endl;
            }
            continue;
        }   
        else
          return;
      }
      // if the non visible point has no cameFrom or cameFrom the parent
      if (cameFrom_(x, y) == nullPoint_ || cameFrom_(cameFrom_(x, y)) == pivot) {
        openSet_->push(Node{2, newDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, false});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new non visible primary point at: " << x << ", " << ny_-1-y << std::endl;}
        break;
      }
      // if the non visible point came from a differen point
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> the new point was on the non visible side, but hit the zone of a other pivot thus the march stopped here" << std::endl;
        }
        return;
      }
    }
  // if the point is visible and has no distance assigned yet
    else if (gScore_(x, y) == infinity) {
      // update distance and relation to the parent
      gScore_(x, y) = newDistance;
      cameFrom_(x, y) = pivot;
      // add new node to the heap
      openSet_->push(Node{2, newDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, true});
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added new visible primary point at: " << x << ", " << ny_-1-y << std::endl;
      }
      break;
    }
  // if the point is visible and has a distance assigned
    else {
      // if closer to current pivot then to other
      if (closerToCurrent(x, y, pivot, cameFrom_(x, y))) {
        gScore_(x, y) = newDistance;
        cameFrom_(x, y) = pivot;
      }
      break;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::addNextParentSlopePrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, int secondaryDist, const float& slope, bool onVisiblePoint) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  point parent;
  // if on a visible point
  if (onVisiblePoint) {
    // if the starting point changed ownership
    if (cameFrom_(x, y) != nullPoint_ && cameFrom_(x, y) != pivot) {
      //if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> the beginning of a switched search was set at: " << x << ", " << ny_-1-y << std::endl;
      //}
      addNextSwitchedPrimary(distance, x, y, secondaryDir, primaryDir, secondaryDist, primaryDist, false, true);
      return false;
    }
    // else march
    else {
      parent = cameFrom_(pivot);
      if (move(x, y, primaryDir))
        primaryDist += 1;
      else
        return true;
    }
  }
  // if on a non visible point
  else {
    parent = cameFrom_(pivot);
    if (move(x, y, secondaryDir))
      secondaryDist += 1;
    else
      return true;
  }
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - pivot: " << pivot.first << ", " << ny_-1-pivot.second << std::endl;
    std::cout << "  - parent: " << parent.first << ", " << ny_-1-parent.second << std::endl;
    std::cout << "  - slope: " << slope << std::endl;
    if (onVisiblePoint) {
      std::cout << "  - This node was on the visible / pivot side" << std::endl;
    }
    else {
      std::cout << "  - This node was on the non visible / parent side" << std::endl;
    }
  }
  // ----- process the new point -----
  while (true) {
  // if the new point is on the parent side
    if ( evaluateCardinalDistance(primaryDir, parent.first, parent.second, x, y)*slope 
                >= evaluateCardinalDistance(secondaryDir, parent.first, parent.second, x, y))
    {
      double parentDistance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
      // parent / non visible side and occupied
      if (sharedOccupancyField_->get(x, y)) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> the new point was on the parent side and occupied" << std::endl;
        }
        cameFrom_(x, y) = pivot;
        openSet_->push(Node{4, parentDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, false});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
        }
        return true;
      }
      // parent / non visible side and free
      else {
        // if the non visible point came from a march on primary distance -> also march one in secondary
        if (onVisiblePoint) {
          if (move(x, y, secondaryDir)) {
            secondaryDist += 1;
            onVisiblePoint = false;
            if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
              std::cout << "-> the new point was on the parent side and free, so the algorithm looked one cell further" << std::endl;
            }
            continue;
          }
          else
            return true;
        }
        // if the non visible point has no cameFrom or cameFrom the parent
        if (cameFrom_(x, y) == nullPoint_ || cameFrom_(x, y) == parent) {
            openSet_->push(Node{3, parentDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, false});
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> added new non visible primary point at: " << x << ", " << ny_-1-y << std::endl;
          }
          return true;
        }
        // if the non visible point came from a differen point
        else {
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> the new point was on the non visible side, but hit the zone of a other pivot thus the march stopped here" << std::endl;
          }
          return true;
        }
      }
    }
  // if the new point is on the pivot side
    else {
      double pivotDistance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
      // pivot side and occupied
      if (sharedOccupancyField_->get(x, y)) {
        cameFrom_(x, y) = pivot;
        openSet_->push(Node{4, pivotDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, false});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> the new point was: on pivot side + occupied = new occupied node at: " << x << ", " << ny_-1-y << std::endl;
        }
        return true;
      }
      // pivot side and free cell that has no distance assigned yet
      else if (gScore_(x, y) == infinity) {
        // update distance and relation to the parent
        gScore_(x, y) = pivotDistance;
        cameFrom_(x, y) = pivot;
        // add new node to the heap
        openSet_->push(Node{3, pivotDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, slope, true});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> the new point at (" << x << ", " << ny_-1-y << ") was: on pivot side + free = new visible primary point at: " << x << ", " << ny_-1-y << std::endl;
        }
        return true;
      }
      // pivot side and free cell that has a distance assigned
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> the new point (" << x << ", " << ny_-1-y << ") was on the pivot side but already had a distance assigned" << std::endl;
        }
        // if closer to the current position then to other
        if (closerToCurrent(x, y, pivot, cameFrom_(x, y))) {
          gScore_(x, y) = pivotDistance;
          cameFrom_(x, y) = pivot;
          return true;
        }
        // if closer to other and other has same secondary direction
        else if (secondaryDir_(cameFrom_(x, y)) == secondaryDir) {
          forceMove(x, y, primaryDir, -1);
          pivotDistance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> the beginning of a switched search was set at: " << x << ", " << ny_-1-y << std::endl;
          }
          addNextSwitchedPrimary(pivotDistance, x, y, secondaryDir, primaryDir, secondaryDist, primaryDist-1, false, true);
          return false;
        }
        else {
          return true;
        }
      }
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::advanceSecondaryNode(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist,  int& secondaryDist, bool& moveBoundary, bool checkCFstart) {
  bool outOfBound = false;
  point parent = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  point block;
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - parent: " << parent.first << ", " << ny_-1-parent.second << std::endl;
    std::cout << "  - moveBoundary: " << moveBoundary << std::endl;
  }
  // if start changed ownership
  if (checkCFstart && cameFrom_(x, y) != nullPoint_ && cameFrom_(x, y) != parent) {
    return false;
  }
  // read in boundary from starting point if moveboundary is true
  if (moveBoundary) {
    block = blockCorners_(x, y);
    // blockCorners_(x, y) = nullPoint_; -> optional -> clears irrelavant boundary point
  }
  if (advance(x, y, secondaryDir, outOfBound)) {
  // if the advance was succesfull
  //****************************************************************************************************************
    distance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
    secondaryDist += 1;
    // read in boundary from advanced point if moveboundary is false
    bool onBoundary = false;
    if (!moveBoundary) {
      block = blockCorners_(x, y);
      // if the corner value is not null, and the blockSlope is from the parent
      if (block != nullPoint_ ) {
        if (blockCorners_(block) == parent) {
          onBoundary = true;
        }
        else if (blockCorners_(block) == cameFrom_(block)) {
          blockCorners_(x, y) = nullPoint_;
        }
      }
    }
    // effectivly move the boundary if moveboundary is true
    else {
      blockCorners_(x, y) = block; 
    }
    // process boundary point if on boundary
    //------------------------------------------------------------------------------------------------------
    if (moveBoundary || onBoundary) { 
      float blockSlope = calcBlockSlope(primaryDir, secondaryDir, parent, block);
      // if the point is visible
      if (primaryDist*blockSlope >= secondaryDist) {
        // if it is on the first primary after the corner
        if (evaluateCardinalDistance(primaryDir, block, {x, y}) == 1) {
          // if the point back on primary is free and the one below on secondary is occupied
          if (!checkBackwards(x, y, primaryDir) && checkBackwards(x, y, primaryDir, secondaryDir)) {
            if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
              std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
            }
            openSet_->push(Node{7, distance, x, y, secondaryDir, oppDirection(primaryDir), 0, 0, -1/blockSlope, false});
          }
        }
      }
      // if the current point is not visible
      else {
        // if it is on the first primary after the corner
        if (evaluateCardinalDistance(primaryDir, block, {x, y}) == 1) {
          if (checkBackwards(x, y, primaryDir) || checkBackwards(x, y, primaryDir, secondaryDir)) {
            forceMove(x, y, secondaryDir, -1);
            if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
              std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
            }
            openSet_->push(Node{7, distance, x, y, primaryDir, secondaryDir, 0, 0, blockSlope, false});
            forceMove(x, y, secondaryDir);
          }
        }
        // set block for march on next primary
        if (move(x, y, primaryDir)) {
          // if the boundary is on a null point and the corner does 
          if (cameFrom_(x, y) == nullPoint_ || cameFrom_(cameFrom_(x, y)) == parent) {
            blockCorners_(x, y) = block;
          }
        }
        return false;
      }
      moveBoundary = true;
    }
    // update information of the new visible point
    //------------------------------------------------------------------------------------------------------
    // if the new point has no distance yet
    if (gScore_(x, y) == infinity) {
      if (moveBoundary) {
        blockCorners_(x, y) = block;
      }
      gScore_(x, y) = distance;
      cameFrom_(x, y) = parent;
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
        cameFrom_(x, y) = parent;
      }
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
    bool alreadyOnBoundary = blockCorners_(x, y) != nullPoint_;
    // set block if next primary element is free and there is a path to it. 
    if (advance(x_adj, y_adj, primaryDir) && !checkBackwards(x_adj, y_adj, secondaryDir)) {
      // if moveBoundary was true and the slope coming from the moveblock was smaller than the slope of the new occupied point
      if (moveBoundary && calcBlockSlope(primaryDir, secondaryDir, parent, {x, y}) >= calcBlockSlope(primaryDir, secondaryDir, parent, block)) {
        blockCorners_(x_adj, y_adj) = block;
      }
      else {
        blockCorners_(x_adj, y_adj) = {x, y};
        blockCorners_(x, y) = parent;
      }
    }
  // determine if you need to continue as an occupied node
    // if on moveboundary or a boundary was discovered inside the object
    if (moveBoundary || alreadyOnBoundary) {
      return false;
    }
    // if on the first secondary of the edge search
    else if (primaryDist == 1 && secondaryDir_(parent) != secondaryDir) {
      return false;
    }
    // if the prev point on primary is occupied
    else if (checkBackwards(x, y, primaryDir)) {
      return false;
    }
    // else continue secondary search as occupied node
    else {
      cameFrom_(x, y) = parent;
      distance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
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
bool Solver::advanceOccupiedNode(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, const int& primaryDist,  int& secondaryDist) {
  point parent = cameFrom_(x, y);
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "  - parent: " << parent.first << ", " << ny_-1-parent.second << std::endl;
    std::cout << "  - primaryDir: " << cardir_to_string(primaryDir) << std::endl;
    std::cout << "  - secondaryDir: " << cardir_to_string(secondaryDir) << std::endl;
    std::cout << "  - primaryDist: " << primaryDist << std::endl;
    std::cout << "  - secondaryDist: " << secondaryDist << std::endl;
  }
  // remove camefrom from inside object
  cameFrom_(x, y) = nullPoint_;
  // move to next point
  if (!move(x, y, secondaryDir)) {
    return false;
  }
  else {
    // stop the occupied search if new point is on a boundary
    if (blockCorners_(x, y) != nullPoint_) {
      // cleanup boundary if inside a object 
      if (sharedOccupancyField_->get(x, y)) {
        blockCorners_(x, y) = nullPoint_;
      }
      return false;
    }
    // if not on a boundary
    else {
      // if the point after the move is occupied
      if (sharedOccupancyField_->get(x, y)) {
        // if the prev point on primary is occupied -> stop
        if (checkBackwards(x, y, primaryDir)) {
          return false;
        }
        // if the occupied point is not next to a sibling -> stop
        if (!nextToSibling(x, y, primaryDir, parent, true)) {
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> the new point was occupied, but not next to a sibling" << std::endl;
          }
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
      // if the point after the move is free, but there is an object blocking the path arround the corner
      else if (checkBackwards(x, y, primaryDir)) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> the new point was free, but the path around the corner was blocked" << std::endl;
        }
        return false;
      }
      // if the point after the move is free, and there is a path to it
      else {
        secondaryDist += 1;
        distance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
        float blockSlope = (secondaryDist-0.5)/(primaryDist-0.5);
        // if the current point is visible
        if (primaryDist*blockSlope <= secondaryDist) {
          // if there is already a distance assined to the point behind the object
          if (gScore_(x, y) != infinity) {
            if (closerToCurrent(x, y, parent, cameFrom_(x, y))) {
              gScore_(x, y) = distance;
              cameFrom_(x, y) = parent;
            }
            if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
              std::cout << "-> the point behind the object was visible, but already had a distance assigned" << std::endl;
            }
            return false;
          }
          // if there is no distance assigned continue
          gScore_(x, y) = distance;
          cameFrom_(x, y) = parent;
          // add new primary point of parent at the current location
          openSet_->push(Node{2, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, blockSlope, true});
          // add node to start looking for the pivot location
          openSet_->push(Node{5, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, blockSlope, true});
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> added new visible primary point at: " << x << ", " << ny_-1-y << std::endl;
            std::cout << "-> added new pivot search at the back at : " << x << ", " << ny_-1-y << std::endl;
          }
          return false;
        }
        // if the current point is not visible
        else {
          // create new pivot
          forceMove(x, y, primaryDir, -1);
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
          }
          openSet_->push(Node{7, distance, x, y, secondaryDir, primaryDir, 0, 0, 1/blockSlope, true});
          forceMove(x, y, primaryDir);
          // add the node to start looking for the new primary point to the parent
          openSet_->push(Node{2, distance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, blockSlope, false});
          if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
            std::cout << "-> added new non visible primary point at: " << x << ", " << ny_-1-y << std::endl;
          }
          return false;
        }
      }
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::advancePivotSearch(double& distance, int& x, int& y, const cardir& primaryDir, const cardir& secondaryDir, int& primaryDist, const int& secondaryDist, float& slope, bool& nextToObject) {
  // set pivot and parent
  point parent = cameFrom_(x, y);
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist); // in cases where secondaryDist == 0 -> this will be the parent, but this does (almost) not matter
  // if node is not next to object expand secondary search
  if (!nextToObject) {
    cameFrom_(x, y) = pivot;
    Node node = {0, distance, x, y, primaryDir, oppDirection(secondaryDir), primaryDist, 0, 0, false};
    if (advanceSecondaryNode(node.distance, node.x, node.y, node.primaryDir, node.secondaryDir, node.primaryDist, node.secondaryDist, node.state)) {
      openSet_->push(node);
    }
    cameFrom_(x, y) = parent;
  }
  // Move to the next primary point
  if (!move(x, y, primaryDir)) {
    return false;
  }
  // If the move resulted in a point that is still on the parent side
  //-------------------------------------------------------------------------------------------------
  else if (evaluateCardinalDistance(primaryDir, parent.first, parent.second, x, y)*slope 
              <= evaluateCardinalDistance(secondaryDir, parent.first, parent.second, x, y)) {
    distance = gScore_(parent.first, parent.second) + evaluateDistance(parent.first, parent.second, x, y);
    primaryDist += 1;
    // next point already had a distance assigned
    if (gScore_(x, y) != infinity && cameFrom_(x, y) != parent) {
      return false; //determining which one is closer will be done by the parent secondary search
    }
    // next point is occupied
    if (sharedOccupancyField_->get(x, y)) {
      if (!nextToObject) {
        cameFrom_(x, y) = pivot;
        openSet_->push(Node{4, distance, x, y, primaryDir, oppDirection(secondaryDir), primaryDist, 0, 0, false});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
        }
      }
      return false;
    }
    // previous point was next to a object + next point is free
    if (nextToObject) {
      // if new point is next to object
      if (checkBackwards(x, y, secondaryDir)) {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added pivot search next to object at: " << x << ", " << ny_-1-y << std::endl;
        }
        return true;
      }
      // if new point is next to a void
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> created a node to create a pivot at: " << x << ", " << ny_-1-y << std::endl;
        }
        openSet_->push(Node{7, distance, x, y, primaryDir, oppDirection(secondaryDir), 0, 0, -slope, false});
        return false;
      }
    }
    // previous point was next to a void + next point is free
    else {
      // if new point is next to object
      if (checkBackwards(x, y, secondaryDir)) {
        nextToObject = true;
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added pivot search next to object at: " << x << ", " << ny_-1-y << std::endl;
        }
        return true;
      }
      // if new point is next to void
      else {
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added pivot search next to void at: " << x << ", " << ny_-1-y << std::endl;
        }
        return true;
      }
    }
  }
  // If the move resulted in a point that is on the pivot side
  //-------------------------------------------------------------------------------------------------
  else {
    distance = gScore_(pivot.first, pivot.second) + evaluateDistance(pivot.first, pivot.second, x, y);
    // next point already had a distance assigned
    if (gScore_(x, y) != infinity) {
      if (secondaryDist != 0 && closerToCurrent(x, y, pivot, cameFrom_(x, y))) {
        gScore_(x, y) = distance;
        cameFrom_(x, y) = pivot;
      }
      return false;
    }
    // next point is occupied 
    if (sharedOccupancyField_->get(x, y)) {
      if (!nextToObject) {
        cameFrom_(x, y) = pivot;
        openSet_->push(Node{4, distance, x, y, primaryDir, oppDirection(secondaryDir), primaryDist+1, 0, 0, false});
        if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
          std::cout << "-> added new occupied node at: " << x << ", " << ny_-1-y << std::endl;
        }
      }
      return false;
    }
    // if beginning of pivot side is next to object or one step before a object
    if (nextToObject || (!nextToObject && checkBackwards(x, y, secondaryDir))) {
      forceMove(x, y, primaryDir, -1);
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> creating pivot at: " << x << ", " << ny_-1-y << std::endl;
      }
      createNewPivot({x, y}, pivot, secondaryDir, primaryDir, 1/slope);
      return false;
    }
    // previous point and next point are free and next to void
    else {
      cameFrom_(x, y) = pivot;
      gScore_(x, y) = distance;
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> added new primary straight point at: " << x << ", " << ny_-1-y << std::endl;
        std::cout << "-> added new primary parent slope point at: " << x << ", " << ny_-1-y << std::endl;
      }
      // add next straight primary point to the heap
      openSet_->push(Node{1, distance, x, y, primaryDir, oppDirection(secondaryDir), primaryDist+1, 0, 0, false});
      // start looking for the next parent primary point
      openSet_->push(Node{3, distance, x, y, secondaryDir, primaryDir, 0, primaryDist+1, 1/slope, true});
      return false;
    }
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
bool Solver::addNextSwitchedPrimary(const double& distance, int x, int y, const cardir& primaryDir, const cardir& secondaryDir, int primaryDist, const int& secondaryDist, bool moveBoundary, bool initialCall) {
  point pivot = getPivot(x, y, primaryDir, secondaryDir, primaryDist, secondaryDist);
  // add the next primary point
  double newDistance = distance; 
  if (advanceSecondaryNode(newDistance, x, y, secondaryDir, primaryDir, secondaryDist, primaryDist, moveBoundary, !initialCall)) {
    openSet_->push(Node{6, newDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, moveBoundary});
    return true;
  }
  if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
    std::cout << "-> primary switched march faled at: " << x << ", " << ny_-1-y << std::endl;
  } 
  // if the march failed because it went out of bounds
  if (x < 0 || x >= nx_ || y < 0 || y >= ny_) {
    return true;
  }
  // if the march ended on a occupied cell
  if (sharedOccupancyField_->get(x, y)) {
    if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
      std::cout << "-> the new point was occupied" << std::endl;
    }
    cameFrom_(x, y) = pivot;
    openSet_->push(Node{4, newDistance, x, y, primaryDir, secondaryDir, primaryDist+1, secondaryDist, 0, false});
    return true;
  }
  // if the march ended on a free cell
  else {
    // if the primary march ended overwrithing a camefrom to the pivot
    if (cameFrom_(x, y) == pivot && distanceToEdge(primaryDir, x, y) != 0) {
      openSet_->push(Node{6, newDistance, x, y, primaryDir, secondaryDir, primaryDist, secondaryDist, 0, moveBoundary});
      return true;
    }
    // if the primaymarch failed because of a different pivot zone check and it was not the first call of swithed primary
    else if (cameFrom_(x, y) != nullPoint_ && !initialCall) {
      if (sharedConfig_->debugPivotSearch && nb_of_iterations_ == max_nb_of_iter_) {
        std::cout << "-> switched search switch directions again at: " << x << ", " << ny_-1-y << std::endl;
      }
      forceMove(x, y, primaryDir, -1);
      addNextSwitchedPrimary(distance, x, y, secondaryDir, primaryDir, secondaryDist, primaryDist-1, false, true);
      return false;
    }
    else
      return true;
  }
}
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

} // namespace vbd

#endif // SOLVER_PIVOTS_HPP