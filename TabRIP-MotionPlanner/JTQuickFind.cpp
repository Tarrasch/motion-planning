/**
 * @file JTQuickFind.cpp
 * @brief Returns joint angle information for a desired goal position given in X,Y and Z
 * @author Juan C. Garcia made minor modifications to code provided by Ana C. Huaman Quispe.
 */

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/TrfmTranslate.h>
#include <kinematics/Transformation.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include "JTQuickFind.h"

#include <Eigen/LU>


JTQuickFind::JTQuickFind( robotics::World &_world, 
			double _configStep ) {
    mWorld = &_world;
    mConfigStep = _configStep;
}

JTQuickFind::~JTQuickFind() {
  
}


void JTQuickFind::init( int _robotId,
		       const Eigen::VectorXi &_links,
		       std::string _EEName,
		       int _EEId,
		       double _res ) {

  mRobotId = _robotId;
  mLinks = _links;
  
  mMaxIter = 1000;
  mWorkspaceThresh = _res; // An error of half the resolution
  mEENode = (dynamics::BodyNodeDynamics*) mWorld->getRobot(mRobotId)->getNode( _EEName.c_str() );
  mEEId = _EEId;  
}


Eigen::MatrixXd JTQuickFind::GetPseudoInvJac() {
  //printf("Num Dependent DOF minus 6D0F is : %d \n", mEENode->getNumDependentDofs() - 6 );
  Eigen::MatrixXd Jaclin = mEENode->getJacobianLinear().topRightCorner( 3, mLinks.size() );
  //std::cout<< "Jaclin: \n"<<Jaclin << std::endl;
  Eigen::MatrixXd JaclinT = Jaclin.transpose();
  Eigen::MatrixXd Jt;
  Eigen::MatrixXd JJt = (Jaclin*JaclinT);
  Eigen::FullPivLU<Eigen::MatrixXd> lu(JJt);
  Jt = JaclinT*( lu.inverse() );
  //std::cout<< "Jaclin pseudo inverse: \n"<<Jt << std::endl;  
  return Jt;
}


bool JTQuickFind::GoToXYZ( Eigen::VectorXd &_q, 
			  Eigen::VectorXd _targetXYZ) {

  Eigen::VectorXd dXYZ;
  Eigen::VectorXd dConfig;
  int iter;
  mWorld->getRobot(mRobotId)->update();
  
  //-- Initialize
  dXYZ = ( _targetXYZ - GetXYZ(_q) ); // GetXYZ also updates the config to _q, so Jaclin use an updated value
  iter = 0;
  //printf("New call to GoToXYZ: dXYZ: %f  \n", dXYZ.norm() );
  while( dXYZ.norm() > mWorkspaceThresh && iter < mMaxIter ) {
    //printf("XYZ Error: %f \n", dXYZ.norm() );
    Eigen::MatrixXd Jt = GetPseudoInvJac();
    dConfig = Jt*dXYZ;
    //printf("dConfig : %.3f \n", dConfig.norm() );
    if( dConfig.norm() > mConfigStep ) {
      double n = dConfig.norm();
      dConfig = dConfig *(mConfigStep/n);
      //printf("NEW dConfig : %.3f \n", dConfig.norm() );
    }
    _q = _q + dConfig;
    
    dXYZ = (_targetXYZ - GetXYZ(_q) );
    iter++;
  }
  
  if( iter >= mMaxIter ) { return false; }
  else { return true; }
  
}

Eigen::VectorXd JTQuickFind::GetXYZ( Eigen::VectorXd _q ) {
  // Get current XYZ position
  mWorld->getRobot(mRobotId)->setDofs( _q, mLinks );
  mWorld->getRobot(mRobotId)->update();
  
  Eigen::MatrixXd qTransform = mEENode->getWorldTransform();
  Eigen::VectorXd qXYZ(3); qXYZ << qTransform(0,3), qTransform(1,3), qTransform(2,3);
  
  return qXYZ;
}
 
