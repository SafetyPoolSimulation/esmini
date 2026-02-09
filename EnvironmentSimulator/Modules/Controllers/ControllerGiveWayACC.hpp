/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <string>
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_GIVE_WAY_ACC_TYPE_NAME "GiveWayACCController"

namespace scenarioengine
{
    class ControllerGiveWayACC : public Controller
    {
    public:
        ControllerGiveWayACC(InitArgs* args);

        virtual const char* GetTypeName()
        {
            return CONTROLLER_GIVE_WAY_ACC_TYPE_NAME;
        }
        virtual int GetType()
        {
            return CONTROLLER_TYPE_GIVE_WAY_ACC;
        }

        void Init();
        void InitPostPlayer();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);
        void SetSetSpeed(double setSpeed)
        {
            setSpeed_ = setSpeed;
        }
        void SetTimeGap(double setTimeGap)
        {
            timeGap_ = setTimeGap;
        }

        bool HasRightSidePriorityConflict(Object* ego, Object* other);

        static constexpr double GIVEWAY_RADIUS    = 25.0;
        static constexpr double GIVEWAY_ANGLE_MIN = -M_PI_2;
        static constexpr double GIVEWAY_ANGLE_MAX = M_PI / 8.0;
        static constexpr double GIVEWAY_MIN_SPEED = 0.5;

    private:
        vehicle::Vehicle vehicle_;
        bool             active_;
        double           timeGap_;  // target headway time
        double           setSpeed_;
        double           lateralDist_;
        double           currentSpeed_;
        bool             setSpeedSet_;
        bool             virtual_;
    };

    Controller* InstantiateControllerGiveWayACC(void* args);
}  // namespace scenarioengine