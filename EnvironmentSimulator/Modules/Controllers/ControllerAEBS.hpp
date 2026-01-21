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
#include "ControllerALKS_R157SM.hpp"

#define CONTROLLER_AEBS_TYPE_NAME "AEBSController"

namespace scenarioengine
{
    class ControllerAEBS : public Controller
    {
    public:
        ControllerAEBS(InitArgs* args);

        virtual const char* GetTypeName()
        {
            return CONTROLLER_AEBS_TYPE_NAME;
        }
        virtual int GetType()
        {
            return CONTROLLER_TYPE_ACC;
        }

        void Init();
        void InitPostPlayer();
        void LinkObject(Object* object);
        void Step(double timeStep);
        int Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);

    private:
        bool    active_;
        double  ttc_;
        double  deceleration_;
        bool    available_;
        double  setSpeed_;
        double  currentSpeed_;
        bool    setSpeedSet_;
        bool    virtual_;
    };

    inline ControllerALKS_R157SM::ReferenceDriver aeb_driver_; // obtain ReferenceDriver to get AEB functionality

    Controller* InstantiateControllerAEBS(void* args);
}

