/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 * @file RRT.cpp
 *
 * Authors: Tobias Kunz <tobias@gatech.edu>, Ana Huaman <ahuaman3@gatech.edu>
 * Date: 10/2012
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 */

#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "RRT.h"

/**
 * @function RRT
 * @brief Constructor
 */
RRT::RRT( robotics::World *_world,
          int _robotId,
          const Eigen::VectorXi &_links,
          const Eigen::VectorXd &_root,
          double _stepSize ) {

  /// Initialize some member variables
  world = _world;
  robotId = _robotId;
  links = _links;
  ndim = links.size();
  stepSize = _stepSize;

  /// Initialize random generator
  srand( time(NULL) );

  /// Create kdtree and add the first node (start)
  kdTree = kd_create( ndim );
  addNode( _root, -1 );
}

/**
 * @function ~RRT()
 * @brief Destructor
 */
RRT::~RRT() {
    kd_free( kdTree );
}

/**
 * @function connect
 * @brief Connect the closest node with random target, stop until it is reached or until it collides
 */
bool RRT::connect() {
  Eigen::VectorXd qtry = getRandomConfig();
  return connect( qtry );
}

/**
 * @function connect
 * @brief Connect the closest node with _target, stop until it is reached or until it collides
 */
bool RRT::connect( const Eigen::VectorXd &_target ) {

  int NNIdx = getNearestNeighbor( _target );
  StepResult result = STEP_PROGRESS;
  int i = 0;
  while( result == STEP_PROGRESS ) {

    result = tryStepFromNode( _target, NNIdx );
    NNIdx = configVector.size() -1;
    i++;
  }
  return ( result == STEP_REACHED );

}

/**
 * @function tryStep
 * @brief Try to advance one stepSize towards a random target
 */
RRT::StepResult RRT::tryStep() {
    Eigen::VectorXd qtry = getRandomConfig();
    return tryStep( qtry );
}

/**
 * @function tryStep
 * @brief Try to advance one stepSize towards _qtry
 */
RRT::StepResult RRT::tryStep( const Eigen::VectorXd &_qtry ) {
  int NNIdx = getNearestNeighbor( _qtry );
  return tryStepFromNode( _qtry, NNIdx );
}

/**
 * @function tryStepFromNode
 * @brief Tries to extend tree towards provided sample (must be overridden for MBP )
 */
RRT::StepResult RRT::tryStepFromNode( const Eigen::VectorXd &_qtry, int _NNIdx ) {

  /// Calculates a new node to grow towards _qtry, check for collisions and adds to tree
  Eigen::VectorXd qnear = configVector[_NNIdx];

  /// Compute direction and magnitude
  Eigen::VectorXd diff = _qtry - qnear;
  double dist = diff.norm();

  if( dist < stepSize ) {
    return STEP_REACHED;
  }

  /// Scale this vector to stepSize and add to end of qnear
  Eigen::VectorXd qnew = qnear + diff*(stepSize/dist);

  if( !checkCollisions(qnew) ) {
    addNode( qnew, _NNIdx );
    return STEP_PROGRESS;
  } else {
    return STEP_COLLISION;
  }

}

/**
 * @function addNode
 * @brief Add _qnew to tree with parent _parentId
 * @return id of node added
 */
int RRT::addNode( const Eigen::VectorXd &_qnew, int _parentId )
{
  /// Update graph vectors -- what does this mean?
  configVector.push_back( _qnew );
  parentVector.push_back( _parentId );

  uintptr_t id = configVector.size() - 1;
  kd_insert( kdTree, _qnew.data(), (void*) id ); //&idVector[id];  /// WTH? -- ahq

  activeNode = id;
  return id;
}

/**
 * @function getRandomConfig
 * @brief Samples a random point for qtmp in the configuration space,
 * bounded by the provided configuration vectors (and returns ref to it)
 */
Eigen::VectorXd RRT::getRandomConfig() {
  Eigen::VectorXd config( ndim );
  for( unsigned int i = 0; i < ndim; i++ ) {
    double minVal = world->getRobot(robotId)->getDof(links[i])->getMin();
    double maxVal = world->getRobot(robotId)->getDof(links[i])->getMax();
    config[i] = randomInRange( minVal, maxVal );
  }

  return config;
}

/**
 * @function getNearestNeighbor
 * @brief Returns Nearest Neighbor index to query point
 */
int RRT::getNearestNeighbor( const Eigen::VectorXd &_qsamp ) {

    struct kdres* result = kd_nearest( kdTree, _qsamp.data() );
    uintptr_t nearest = (uintptr_t)kd_res_item_data(result);

    activeNode = nearest;
    return nearest;

}

/**
 * @function getGap
 * @brief Get the gap (Distance) between the closest node in the tree to the _target
 */
double RRT::getGap( const Eigen::VectorXd &_target ) {
    return ( _target - configVector[activeNode] ).norm();
}

/**
 * @function tracePath
 * @brief Traces the path from some node to the initConfig node
 */
void RRT::tracePath( int _node,
                     std::list<Eigen::VectorXd> &_path,
                     bool _reverse ) {

    int x = _node;

    while( x != -1 ) {
      if( !_reverse ) {
	_path.push_front( configVector[x] );
      } else {
	_path.push_back( configVector[x] );
      }
      x = parentVector[x];
    }
}

/**
 * @function checkCollisions
 * @brief Returns true if collisions. If it is false, we are cool.
 */
bool RRT::checkCollisions( const Eigen::VectorXd &_config ) {
  world->getRobot(robotId)->setDofs( _config, links );
  world->getRobot(robotId)->update();
  return world->checkCollision();
}

/**
 * @function getSize
 * @brief Returns size of the tree
 */
unsigned int RRT::getSize() {
    return configVector.size();
}

/**
 * @function randomInRange
 * @brief Get random number between min and max
 */
double RRT::randomInRange( double _min, double _max ) {

    return _min + ((_max - _min) * ((double)rand() / ((double)RAND_MAX + 1)));
}
