/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef MANIPULATION_TAB
#define MANIPULATION_TAB

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Joint.h>
#include "PathPlanner.h"
#include <kinematics/Joint.h> 
#include <kinematics/Dof.h> 

#include <iostream>

/**
 * @class Manipulation Tab
 * @brief Implements RTT algorithms + ability to manipulate objects
 */
class ManipulationTab : public GRIPTab
{
public:
    Eigen::VectorXd mStartConf;
    
    double start_x,start_y,start_z;
    double goal_x,goal_y,goal_z;
    robotics::Object* object;
    
    int mRobotId;
    Eigen::VectorXi mLinks;

    int mRrtStyle;
    bool mGreedyMode;
    bool mConnectMode;
    bool mSmooth;
    PathPlanner *mPlanner;

    wxTextCtrl *mTimeText;

    // public vars to capture external selection stuff
    robotics::Object* mSelectedObject;
    robotics::Robot* mSelectedRobot;
    dynamics::BodyNodeDynamics* mSelectedNode;

    /// Functions

    ManipulationTab(){};
    ManipulationTab( wxWindow * parent, wxWindowID id = -1,
                   const wxPoint & pos = wxDefaultPosition,
                   const wxSize & size = wxDefaultSize,
                   long style = wxTAB_TRAVERSAL);
    virtual ~ManipulationTab(){}

    void OnSlider(wxCommandEvent &evt);
    void OnRadio(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnCheckBox(wxCommandEvent &evt);
    void SetTimeline();
    void GRIPStateChange();
    void AttachObject();
    
    // Thread specific
    // GRIPThread* thread;

    // Your Thread routine
    // call GRIPThread::CheckPoint() regularly
    // void Thread();
    // void onCompleteThread();

    DECLARE_DYNAMIC_CLASS( RipTabPlannerTab )
    DECLARE_EVENT_TABLE()
};

#endif /** MANIPULATION_TAB */
