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

        ~ControllerAEBS();

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

        void UpdateDynamicParams(double egoSpeed);

        double speed_at_impact_ = NAN;
        bool end_logged_ = false;

    private:
        ControllerALKS_R157SM::ReferenceDriver aeb_driver_;  // obtain ReferenceDriver to get AEB functionality

        /* Input parameters */
        bool    available_;
        double  ttc_;
        double  deceleration_; // OpenSCENARIO max.
        double  fcw_audio_ttc_;
        double  fcw_visual_ttc_;
        double  lon_lookahead_dist_;
        double  lat_lookahead_dist_;

        /* Internal calculation parameters */
        double  setSpeed_;
        double  currentSpeed_;
        bool    setSpeedSet_;
        bool    virtual_;

        bool fcw_audio_logged_ = false;
        bool fcw_visual_logged_ = false;
        bool aeb_logged_        = false;
        double aeb_start_time_     = NAN;

        /* Dynamic Braking Paremeters */
        double brake_delay_ = 0.0;
        double ramp_time_   = 0.5;
        double max_decel_   = 10.0; // AEB requested peak deceleration
        
        bool brake_started_ = false;
        double brake_start_time_ = 0.0;
        double current_decel_    = 0.0;

        double v_trigger_ = 0.0;  // speed at AEB trigger moment, used to scale decel

        double min_gap_ever_ = LARGE_NUMBER;
    };

    Controller* InstantiateControllerAEBS(void* args);
}

