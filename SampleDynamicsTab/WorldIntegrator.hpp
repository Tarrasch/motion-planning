/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Saul Reynolds-Haertle <saulrh@gatech.edu>
 * Date: 21/10/2012
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 **
 *   @file WorldIntegrator.h
 *   @brief Class that hooks a world and state up to an integrator for
 *   use DART simulations.
 */


// how this all works, theoretically:

// create a WorldIntegrator and point it at your world.
// use readFromWorld to get a WorldState. Maybe several
//     worldstates. organize them how you like.
// point the WorldIntegrator at the world state
// tell the worldintegrator to integrate
// the world state has now been changed to be one simulation step into
//     the future.
// use writeToWorld to write the new changes back into the
//     world. Done.

#ifndef __WORLD_INTEGRATOR_H__
#define __WORLD_INTEGRATOR_H__

#include <Tabs/GRIPTab.h>
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <robotics/World.h>
#include <iostream>
#include <integration/EulerIntegrator.h>

/**
 * @class WorldState

 * @brief Convenient intermedia representation for holding the state
 *        of a world. Can convert back and forth from
 *        serialized/compact/whatever VectorXd form (for simulation)
 *        and an actual world.
 */

class WorldState
{
public:
    WorldState();
    WorldState(robotics::World* w);
    WorldState(robotics::World* w, Eigen::VectorXd& serState);
    WorldState(WorldState& other);
    WorldState(WorldState* other);

    ~WorldState();

    void writeToWorld(robotics::World* w, bool updateDynamics=false);
    void readFromWorld(robotics::World* w);
    void writeToVector(Eigen::VectorXd& serState);
    void readFromVector(robotics::World* w, Eigen::VectorXd& serState);

    std::vector<Eigen::VectorXd> mPosVects;
    std::vector<Eigen::VectorXd> mVelVects;
    double mT;
    int mId;

    static int getUID() { static int sUID = 0; return sUID++; }
    static int getNumberOfDoFs(robotics::World* w);


    void printToStdout();
    static void printVectToStdout(Eigen::VectorXd& v);
    static void printWorldToStdout(robotics::World* w);
};

/**
 * @class WorldIntegrator
 * @brief Hooks a DART World up to a DART Integrator to allow dynamic
 * simulation.
 */
class WorldIntegrator : public integration::IntegrableSystem
{
public:
    WorldIntegrator();
    WorldIntegrator(robotics::World* w);
    ~WorldIntegrator();
    
    virtual Eigen::VectorXd getState();
    virtual Eigen::VectorXd evalDeriv();
    virtual void setState(Eigen::VectorXd state);

    robotics::World* mWorld;
    WorldState* mWorldState;
};


#endif  /* __WORLD_INTEGRATOR_H__ */
