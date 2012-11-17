/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 */

#include "GRIPApp.h"
#include "tabRipPlanner.h"
#include "tabManipulation.h"

extern wxNotebook* tabView;

/**
	* @class RipPlannerTabApp
	*/
class RipPlannerTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new RipPlannerTab(tabView), wxT("RIP Planner"));
		tabView->AddPage(new ManipulationTab(tabView), wxT("Manipulation"));
		
	}
};

IMPLEMENT_APP(RipPlannerTabApp)
