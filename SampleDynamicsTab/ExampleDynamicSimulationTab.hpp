/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#ifndef DYNAMIC_SIMULATION_TAB
#define DYNAMIC_SIMULATION_TAB

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>

#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <robotics/World.h>
#include <integration/EulerIntegrator.h>
#include <integration/RK4Integrator.h>

#include "WorldIntegrator.hpp"

#include <iostream>
#include <time.h>

/**
 * @class ExampleDynamicSimulationTab
 * @brief Uses DART's dynamic simulation capabilities
 */
class ExampleDynamicSimulationTab : public GRIPTab
{
public:
    std::vector< WorldState* > mSimHistory;

    // sizer for whole tab
    wxBoxSizer* sizerFull;

    // public vars to capture external selection stuff 
    robotics::Object* mSelectedObject;
    robotics::Robot* mSelectedRobot;
    dynamics::BodyNodeDynamics* mSelectedNode;

    /// Functions
    ExampleDynamicSimulationTab(){};
    ExampleDynamicSimulationTab( wxWindow * parent, wxWindowID id = -1,
                          const wxPoint & pos = wxDefaultPosition,
                          const wxSize & size = wxDefaultSize,
                          long style = wxTAB_TRAVERSAL);
    virtual ~ExampleDynamicSimulationTab(){}

    void SimulateFrame();
    void OnSlider(wxCommandEvent &evt);
    void OnButton(wxCommandEvent &evt);
    void OnTimer(wxTimerEvent &evt);
    void PopulateTimeline();
    void GRIPStateChange();

    // wx events
    DECLARE_DYNAMIC_CLASS( ExampleDynamicSimulationTab )
    DECLARE_EVENT_TABLE()
    
private:
    integration::EulerIntegrator mEuIntegrator;
    integration::RK4Integrator mRK4Integrator;

    wxTimer* mSimTimer;
    
    WorldState* mCurrentSimState;
};

#endif /** DYNAMIC_SIMULATION_TAB */
