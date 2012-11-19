/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors:  Ana Huaman <ahuaman3@gatech.edu>, Tobias Kunz <tobias@gatech.edu>
 * Date: 10/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#include "PathPlanner.h"
#define PRINT(x) std::cout << #x << " = " << x << std::endl;

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner() {
  copyWorld = false;
  world = NULL;
}

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner( robotics::World &_world,
                          bool _copyWorld, double _stepSize ) {

  copyWorld = _copyWorld;

  if( copyWorld ) {
    printf( "Do not use this option yet \n" );
  } else {
    world = &_world;
  }

  stepSize = _stepSize;
}

/**
 * @function ~PathPlanner
 * @brief Destructor
 */
PathPlanner::~PathPlanner() {

  if( copyWorld ) {
    delete world;
  }
}

/**
 * @function planPath
 * @brief Main function
 */
bool PathPlanner::planPath( int _robotId,
        		    const Eigen::VectorXi &_links,
                            const Eigen::VectorXd &_start,
                            const Eigen::VectorXd &_goal,
                            bool _bidirectional,
                            bool _connect,
                            bool _greedy,
                            bool _smooth,
                            unsigned int _maxNodes ) {


  //world->mRobots[_robotId]->setQuickDofs( _start ); // Other quick way
  world->getRobot(_robotId)->setDofs( _start, _links );
  if( world->checkCollision() )
    return false;

  world->getRobot(_robotId)->setDofs( _goal, _links );
  if( world->checkCollision() )
    return false;

  bool result;
  if( _bidirectional ) {
    result = planBidirectionalRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes );
  } else {
    result = planSingleTreeRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes );
  }

  if( result && _smooth ) {
    shortenAndSmoothPath( _robotId, _links, path );
  }

  return result;
}

int randomNumber(int min, int max){
  int t = rand() % max + min;
  std::cout << "Rand: " << t << std::endl;
  return t;
}
/**
 * @function planSingleRRT
 * @brief Finds a plan using a standard RRT
 */
bool PathPlanner::planSingleTreeRrt( int _robotId,
                                     const Eigen::VectorXi &_links,
                                     const Eigen::VectorXd &_start,
                                     const Eigen::VectorXd &_goal,
                                     bool _connect,
                                     bool _greedy,
                                     unsigned int _maxNodes ) {

  RRT rrt( world, _robotId, _links, _start, stepSize );
  RRT::StepResult result = RRT::STEP_PROGRESS;
  
  double smallestGap = DBL_MAX;
  while ( result != RRT::STEP_REACHED && smallestGap > stepSize ) {

    /** greedy section */
    if( _greedy ) {

      /** greedy and connect */
      if( _connect ) {

        if(randomNumber(0,7) == 1){
          rrt.connect(_goal);
          std::cout << "Connect greedy!" << std::endl;
        }
        else{
          rrt.connect();
          std::cout << "Connect!" << std::endl;
        }
      } else {
        if(randomNumber(0,7) == 1){
          rrt.tryStep(_goal);
          std::cout << "greedy!" << std::endl;
        }
        else{
          rrt.tryStep();
          std::cout << "Simple!" << std::endl;
        }
      }

      /** NO greedy section */
    } else {

      /** NO greedy and Connect */
      if( _connect ) {
        rrt.connect();

        /** No greedy and No connect -- PLAIN RRT */
      } else {
        rrt.tryStep();
      }

    }
    
    if( _maxNodes > 0 && rrt.getSize() > _maxNodes*2 ) {
      printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes*2 );
      return false;
    }

    double gap = rrt.getGap( _goal );
    if( gap < smallestGap ) {
      smallestGap = gap;
      std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << rrt.configVector.size() << std::endl;
      std::cout <<"--RRT Size: " << rrt.getSize() << std::endl;
    }
  } // End of while

  /// Save path
  printf(" --> Reached goal! : Gap: %.3f \n", rrt.getGap( _goal ) );
  rrt.tracePath( rrt.activeNode, path, false );

  return true;
}

/**
 * @function planBidirectionalRRT
 * @brief Grows 2 RRT (Start and Goal)
 */
bool PathPlanner::planBidirectionalRrt( int _robotId,
                                        const Eigen::VectorXi &_links,
                                        const Eigen::VectorXd &_start,
                                        const Eigen::VectorXd &_goal,
                                        bool _connect,
                                        bool _greedy, // no effect here
                                        unsigned int _maxNodes ) {

  RRT rrt_start( world, _robotId, _links, _start, stepSize );
  RRT rrt_goal( world, _robotId, _links, _goal, stepSize );
  
  double smallestGap = DBL_MAX;
  bool b = false;
  while ( smallestGap > stepSize ) {
    b ^=1;
    RRT &rrt_1 = b ? rrt_start : rrt_goal;
    RRT &rrt_2 = b ? rrt_goal  : rrt_start;

    /** and connect */
    if( _connect ) {
      rrt_1.connect();
      rrt_2.connect(rrt_1.configVector[rrt_1.activeNode]);
    } else { // !_connect
      rrt_1.tryStep();
      rrt_2.tryStep(rrt_1.configVector[rrt_1.activeNode]);
    }

    if( _maxNodes > 0 && rrt_start.getSize()*2 > _maxNodes ) {
      printf("--(!) Exceeded maximum of %d nodes. No path found (!)--\n", _maxNodes );
      return false;
    }

    double gap = rrt_2.getGap( rrt_1.configVector[rrt_1.activeNode] );

    if( gap < smallestGap ) {
      smallestGap = gap;
      std::cout << "--> [planner] Gap: " << smallestGap << "  Tree size: " << rrt_2.configVector.size() << std::endl;
    }
  } // End of while

  /// Save path
  printf(" --> Trees met! : Gap: %.3f \n", smallestGap );
  rrt_start.tracePath( rrt_start.activeNode, path, false );
  rrt_goal.tracePath( rrt_goal.activeNode, path, true );

  return true;
}


/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool PathPlanner::checkPathSegment( int _robotId,
                                    const Eigen::VectorXi &_links,
                                    const Eigen::VectorXd &_config1,
                                    const Eigen::VectorXd &_config2 ) const {

  int n = (int)((_config2 - _config1).norm() / stepSize );

  for( int i = 0; i < n; i++ ) {
    Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
    world->getRobot(_robotId)->setDofs( conf, _links );
    if( world->checkCollision() ) {
      return false;
    }
  }

  return true;
}

void PathPlanner::shortenPath( int _robotId,
    const Eigen::VectorXi &_links,
    std::list<Eigen::VectorXd> &_path ) {

  std::cout << "Path size: "<< _path.size() << std::endl;
  std::list<Eigen::VectorXd>::iterator start;
  std::list<Eigen::VectorXd>::iterator next;
  
  start = _path.begin();
  
  next = _path.begin();
  next++;
  double stepSize = (*next - *start).norm() ; // Ok this assumes 2 elements but whatever

  int num_inserted = 0; // Just for debug
  int num_deleted = 0;  // Just for debug
  
  while(next != _path.end()){
    // NOTE: When entering this while loop, next should be one step ahead of
    // start. Not two!
    	std::list<Eigen::VectorXd>::iterator mid = next;
    	if(++next == _path.end()){
    	  break;
    	}
    	bool check = checkPathSegment(_robotId, _links, *start, *next);
    	if(check){
    	  _path.erase(mid);
        num_deleted++;
    	}
    	else{
        // Remember how we assumed we can actually reach mid? Well lets
        // add some intermediete points between start and mid!
        const Eigen::VectorXd diff = *mid - *start;
        const double dist = diff.norm();
        const Eigen::VectorXd direction = diff*(stepSize/dist);
        double i = 1;
        while((*start+direction*(i+1e-6)).norm() < dist){
          Eigen::VectorXd newVector = *start+direction*i;
          _path.insert(mid, newVector);
          i++;
          num_inserted++;
        }
    	  start = mid;

        PRINT(num_inserted);
        PRINT(num_deleted);
        std::cout << "Note, num_inserted should be *almost* equal to num_deleted" << std::endl;
        num_inserted = 0;
        num_deleted = 0;
    	}
  }
  std::cout << "Path size after: "<< _path.size() << std::endl;
  return;
}

void PathPlanner::smoothPath( int _robotId,
    const Eigen::VectorXi &_links,
    std::list<Eigen::VectorXd> &_path ) {

  const int SPAN_SIZE = 9;

  if(SPAN_SIZE%2>0){
    PRINT(SPAN_SIZE);
    std::cout << "SPAN_SIZE must be odd!!" << std::endl;
  }

  std::list<Eigen::VectorXd>::iterator beg_local = _path.begin();
  std::list<Eigen::VectorXd>::iterator end_local = _path.begin();

  {
    int i = 0;
    while(end_local != _path.end() && i < SPAN_SIZE){
      end_local++;
      i++;
    }
  }

  const Eigen::VectorXd IDENTITY_VECTOR((*beg_local).size()); // Identity under summation

  while(end_local != _path.end()) {
    // First setup three new useful vectors
    std::list<Eigen::VectorXd>::iterator before_mid, mid, after_mid;
    before_mid = beg_local;
    for(int i = 0; i < SPAN_SIZE/2; i++) {
      before_mid++;
    }
    mid = before_mid;
    mid++;
    after_mid = mid;
    after_mid++;

    // Ok, can we replace mid with the average instead?
    /* Eigen::VectorXd candidate_new_mid = */
    /*   accumulate(beg_local, end_local, IDENTITY_VECTOR)/double(SPAN_SIZE); // average */
    Eigen::VectorXd candidate_new_mid = IDENTITY_VECTOR;
    std::list<Eigen::VectorXd>::iterator looper = beg_local;
    while ( looper!=end_local )
      candidate_new_mid += *looper++;
    candidate_new_mid /= double(SPAN_SIZE);

    bool check_left = checkPathSegment(_robotId, _links, *before_mid, candidate_new_mid);
    bool check_right = checkPathSegment(_robotId, _links, candidate_new_mid, *after_mid);
    if(check_left && check_right) {
      *mid = candidate_new_mid; // Replace original by average, if possible
      std::cout << "Yay, one node got smoothed" << std::endl;
    }
    else{
      std::cout << "Doh, one node didn't get smoothed (object was in the way)" << std::endl;
      PRINT(check_left);
      PRINT(check_right);

    }

    // Move ahead the local zoom
    beg_local++;
    end_local++;
  }

}

/**
 * @function shortenAndSmoothPath
 */
void PathPlanner::shortenAndSmoothPath( int _robotId,
    const Eigen::VectorXi &_links,
    std::list<Eigen::VectorXd> &_path ) {
  shortenPath(_robotId, _links, _path);
  smoothPath(_robotId, _links, _path);
}


