/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "GRIPApp.h"
#include "ExampleDynamicSimulationTab.hpp"

extern wxNotebook* tabView;

/**
 * @class RipPlannerTabApp
 */
class RipPlannerTabApp : public GRIPApp {
    virtual void AddTabs() {
        tabView->AddPage(new ExampleDynamicSimulationTab(tabView), wxT("Dynamic Sim Example"));
    }
};

IMPLEMENT_APP(RipPlannerTabApp)
