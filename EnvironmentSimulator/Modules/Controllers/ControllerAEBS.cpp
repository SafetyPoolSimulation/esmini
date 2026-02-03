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

/*
 * This controller simulates a simple Automated Emergency Braking System
 */

#include "ControllerAEBS.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerAEBS(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerAEBS(initArgs);
}

ControllerAEBS::ControllerAEBS(InitArgs* args)
    : Controller(args),
      available_(true),
      ttc_(1.4),
      deceleration_(8.3385),
      fcw_audio_ttc_(2.3),
      fcw_visual_ttc_(2.1),
      setSpeed_(0),
      lon_lookahead_dist_(50.0),
      lat_lookahead_dist_(5.0),
      currentSpeed_(0),
      setSpeedSet_(false),
      virtual_(false)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG);

    if (args && args->properties && args->properties->ValueExists("AEBAvailable"))
    {
        available_ = args->properties->GetValueStr("AEBAvailable") == "true" ? true : false;
    }
    if (args && args->properties && args->properties->ValueExists("AEBTTC"))
    {
        ttc_ = strtod(args->properties->GetValueStr("AEBTTC"));
    }
    if (args && args->properties && args->properties->ValueExists("AEBDeceleration"))
    {
        deceleration_ = strtod(args->properties->GetValueStr("AEBDeceleration"));
    }
    if (args && args->properties && args->properties->ValueExists("FCWAudioTTC"))
    {
        fcw_audio_ttc_ = strtod(args->properties->GetValueStr("FCWAudioTTC"));
    }
    if (args && args->properties && args->properties->ValueExists("FCWVisualTTC"))
    {
        fcw_visual_ttc_ = strtod(args->properties->GetValueStr("FCWVisualTTC"));
    }
    if (args && args->properties && args->properties->ValueExists("LookaheadDistanceLon"))
    {
        lon_lookahead_dist_ = strtod(args->properties->GetValueStr("LookaheadDistanceLon"));
    }
    if (args && args->properties && args->properties->ValueExists("LookaheadDistanceLat"))
    {
        lat_lookahead_dist_ = strtod(args->properties->GetValueStr("LookaheadDistanceLat"));
    }
}

ControllerAEBS::~ControllerAEBS()
{
    // Log END_STATE if not already logged
    if (!end_logged_)
    {
        LOG_INFO("[AEBS_EVENT] END_STATE (destructor) "
                 "minGap={:.2f}m impact=YES speedAtImpact={:.2f}m/s",
                 min_gap_ever_,
                 currentSpeed_);
        end_logged_ = true;
    }
}

void ControllerAEBS::Init()
{
    Controller::Init();
}

void ControllerAEBS::InitPostPlayer()
{
    // Uncomment line below to enable example how to add sensors. Press 'r' to visualize sensor frustum.
    // player_->AddObjectSensor(object_, 4.0, 0.0, 0.5, 0.0, 1.0, 50.0, 1.2, 100);
}

void ControllerAEBS::LinkObject(Object* object)
{
    if (!object)
        return;

    if (object->type_ != Object::Type::VEHICLE)
    {
        LOG_ERROR("Cannot assign AEBS controller to a non vehicle object {}", object->GetName());
        return;
    }

    Controller::LinkObject(object);

    // Link aeb_driver_ from ALKS_R157SM to Ego and set params
    Vehicle* egoVeh = static_cast<Vehicle*>(object);
    aeb_driver_.SetVehicle(egoVeh);
    aeb_driver_.aeb_.ttc_critical_aeb_ = ttc_;
    aeb_driver_.aeb_.max_dec_          = deceleration_;
    aeb_driver_.aeb_.available_        = available_;
}

void ControllerAEBS::Step(double timeStep)
{
    // ACC params
    double minGapLength = LARGE_NUMBER;
    int minObjIndex = -1;
    const double minDist = 3.0;

    // First check if speed has been set from somewhere else (another action or controller), respect it and update setSpeed
    if (virtual_)
    {
        currentSpeed_ = object_->GetSpeed();
        //LOG_INFO("[AEBS] Virtual mode, currentSpeed set to {:.2f}", currentSpeed_);
    }
    else if (abs(object_->GetSpeed() - currentSpeed_) > 1e-3)
    {
        //LOG_INFO("[AEBS] New setspeed detected: {:.2f}", setSpeed_);
        setSpeed_ = object_->GetSpeed();
    }

    //double lookaheadDist = MAX(50.0, 2 * minDist - pow(currentSpeed_, 2) / -object_->GetMaxDeceleration());
    double lookaheadDist = lon_lookahead_dist_; // set from .xosc parameter

    // Loop over all entities
    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        Object* pivot_obj = entities_->object_[i];
        if (!pivot_obj || pivot_obj == object_)
            continue;

        // Compute bounding-box distances in ego-local frame
        double x_local, y_local;
        object_->FreeSpaceDistance(pivot_obj, &y_local, &x_local);

        // Skip vehicles outside longitudinal or lateral lookahead
        if (x_local > lon_lookahead_dist_ || y_local < -lat_lookahead_dist_ || y_local > lat_lookahead_dist_)
            continue;

        // Candidate lead vehicle: smallest longitudinal distance
        if (x_local > 0 && x_local < minGapLength)
        {
            minGapLength = x_local;
            minObjIndex  = static_cast<int>(i);
        }
    }

    double acc = 0.0;

    if (minObjIndex > -1)
    {
        Object* lead = entities_->object_[static_cast<unsigned int>(minObjIndex)];
        if (!lead)
        {
            LOG_WARN("[AEBS] Lead object pointer is NULL!");
        }
        else
        {
            //LOG_INFO("[AEBS] Lead vehicle {} at index {}", lead->GetName(), minObjIndex);

            // --- AEBS integration ---
            ControllerALKS_R157SM::Model::ObjectInfo obj_info;
            obj_info.obj = lead;

            double relSpeed = currentSpeed_ - lead->GetSpeed();
            if (fabs(relSpeed) < 1e-6)
            {
                //LOG_WARN("[AEBS] relSpeed very small ({:.6f}), clamping to 1e-6 to avoid division by zero", relSpeed);
                relSpeed = 1e-6;
            }

            obj_info.ttc = (relSpeed > 0.0) ? (minGapLength / relSpeed) : LARGE_NUMBER; // this will need refinement for non-stationary lead vehicles

            //LOG_INFO("[AEBS] AEBS update: TTC = {:.2f}, lead speed = {:.2f}, currentSpeed = {:.2f}", obj_info.ttc, lead->GetSpeed(), currentSpeed_);
      
            // Log FCWs
            if (!aeb_logged_)
            {
                if (!fcw_audio_logged_ && obj_info.ttc <= fcw_audio_ttc_)
                {
                    LOG_INFO("[AEBS_EVENT] FCW_AUDIO "
                                "speed={:.2f}m/s gap={:.2f}m TTC={:.2f}s",
                                currentSpeed_,
                                minGapLength,
                                obj_info.ttc);
                    fcw_audio_logged_ = true;
                }

                if (!fcw_visual_logged_ && obj_info.ttc <= fcw_visual_ttc_)
                {
                    LOG_INFO("[AEBS_EVENT] FCW_VISUAL "
                                "speed={:.2f}m/s gap={:.2f}m TTC={:.2f}s",
                                currentSpeed_,
                                minGapLength,
                                obj_info.ttc);
                    fcw_visual_logged_ = true;
                }
            }

            try
            {
                aeb_driver_.UpdateAEB(static_cast<Vehicle*>(object_), &obj_info);

                if (aeb_driver_.aeb_.active_ && !aeb_logged_)
                {
                    aeb_start_time_ = scenario_engine_->getSimulationTime();
                    LOG_INFO("[AEBS_EVENT] AEB_ACTIVATION "
                                "time {:.3f}s speed={:.2f}m/s gap={:.2f}m TTC={:.2f}s maxDecel={:.2f}m/s²",
                                aeb_start_time_,
                                currentSpeed_,
                                minGapLength,
                                obj_info.ttc,
                                aeb_driver_.aeb_.max_dec_);
                    aeb_logged_ = true;
                }
            }
            catch (...)
            {
                LOG_ERROR("[AEBS] Crash occurred in aeb_driver_.UpdateAEB!");
                throw;
            }

            if (aeb_driver_.aeb_.active_)
            {
                double aebDec = -aeb_driver_.aeb_.max_dec_;
                currentSpeed_ = std::max(0.0, currentSpeed_ + aebDec * timeStep);
                LOG_INFO("[AEBS] AEBS ACTIVE! Deceleration {:.2f}m/s², currentSpeed = {:.2f}m/s", aebDec, currentSpeed_);
            }

            object_->SetSensorPosition(lead->pos_.GetX(), lead->pos_.GetY(), lead->pos_.GetZ());
        }
    }
    else
    {
        //LOG_INFO("[AEBS] No lead vehicle detected.");

        object_->SetSensorPosition(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ());
    }

    if (mode_ == ControlOperationMode::MODE_OVERRIDE && !virtual_)
    {
        object_->MoveAlongS(currentSpeed_ * timeStep);
        gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    }

    if (virtual_)
    {
        double acc_v[2] = {0.0, 0.0};
        RotateVec2D(acc, 0.0, object_->pos_.GetH(), acc_v[0], acc_v[1]);
        gateway_->updateObjectAcc(object_->GetId(), 0.0, acc_v[0], acc_v[1], 0.0);
    }
    else
    {
        gateway_->updateObjectSpeed(object_->GetId(), 0.0, currentSpeed_);
    }

    min_gap_ever_ = std::min(min_gap_ever_, minGapLength);

    // Get speed at impact
    if (std::isnan(speed_at_impact_) && minGapLength <= 0.0)
    {
        speed_at_impact_ = currentSpeed_;
    }

    // Log end state
    if (!end_logged_)
    {
        bool stopped = currentSpeed_ < 0.1;
        bool interaction_complete = aeb_logged_ || fcw_audio_logged_ || fcw_visual_logged_;

        if (stopped && interaction_complete)
        {
            LOG_INFO("[AEBS_EVENT] END_STATE "
                     "minGap={:.2f}m impact={} speedAtImpact={:.2f}m/s",
                     min_gap_ever_,
                     (min_gap_ever_ <= 0.0 ? "YES" : "NO"),
                     speed_at_impact_);

            end_logged_ = true;
        }
    }

    Controller::Step(timeStep);

    //LOG_INFO("[AEBS] Step end, currentSpeed = {:.2f}", currentSpeed_);
}


int ControllerAEBS::Activate(const ControlActivationMode(&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    currentSpeed_ = object_->GetSpeed();
    if (mode_ == ControlOperationMode::MODE_ADDITIVE || setSpeedSet_ == false)
    {
        setSpeed_ = object_->GetSpeed();
    }

    Controller::Activate(mode);

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        // Make sure heading is aligned with road driving direction
        object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    }

    if (player_)
    {
        player_->SteeringSensorSetVisible(object_->GetId(), true);
    }

    return 0;
}

void ControllerAEBS::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}