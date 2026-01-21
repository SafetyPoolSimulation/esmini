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

#include "ControllerAEBS.h"
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
    active_(false),
    ttc_(1.4),
    deceleration_(8.3385),
    available_(true),
    setSpeed_(0),
    currentSpeed_(0),
    setSpeedSet_(false),
    virtual_(false)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG);

    if (args && args->properties && args->properties->ValueExists("AEBTTC"))
    {
        ttc_ = strtod(args->properties->GetValueStr("AEBTTC"));
    }
    if (args && args->properties && args->properties->ValueExists("AEBDeceleration"))
    {
        deceleration_ = strtod(args->properties->GetValueStr("AEBDeceleration"));
    }
    if (args && args->properties && args->properties->ValueExists("AEBAvailable"))
    {
        available_ = args->properties->GetValueStr("AEBAvailable") == "true" ? true : false;
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
        LOG_INFO("[AEBS] Virtual mode, currentSpeed set to {:.2f}", currentSpeed_);
    }
    else if (abs(object_->GetSpeed() - currentSpeed_) > 1e-3)
    {
        LOG_INFO("[AEBS] New setspeed detected: {:.2f}", setSpeed_);
        setSpeed_ = object_->GetSpeed();
    }

    double lookaheadDist = MAX(50.0, 2 * minDist - pow(currentSpeed_, 2) / -object_->GetMaxDeceleration());

    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        Object* pivot_obj = entities_->object_[i];
        if (pivot_obj == nullptr || pivot_obj == object_)
        {
            continue;
        }

        roadmanager::PositionDiff diff;
        if (object_->pos_.Delta(&pivot_obj->pos_, diff, false, lookaheadDist))
        {
            double adjustedGapLength = diff.ds;
            double dHeading          = GetAbsAngleDifference(object_->pos_.GetH(), pivot_obj->pos_.GetH());

            if (dHeading < M_PI_2)
            {
                adjustedGapLength -=
                    (static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
                    (static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) / 2.0 -
                     static_cast<double>(pivot_obj->boundingbox_.center_.x_));
            }
            else
            {
                adjustedGapLength -=
                    (static_cast<double>(object_->boundingbox_.dimensions_.length_) / 2.0 + static_cast<double>(object_->boundingbox_.center_.x_)) +
                    (static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) / 2.0 +
                     static_cast<double>(pivot_obj->boundingbox_.center_.x_));
            }

            if (diff.dLaneId == 0 && adjustedGapLength > 0 && adjustedGapLength < minGapLength && abs(diff.dt) < lateralDist_)
            {
                minGapLength = adjustedGapLength;
                minObjIndex  = static_cast<int>(i);
                LOG_INFO("[AEBS] Lead candidate found at index {}: gapLength = {:.2f}", i, minGapLength);
            }
        }

        double x_local, y_local;
        object_->FreeSpaceDistance(pivot_obj, &y_local, &x_local);

        if (static_cast<unsigned int>(minObjIndex) != i && x_local > 0 &&
            x_local <
                1.0 + static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) + 0.5 * MAX(0.0, currentSpeed_ - pivot_obj->GetSpeed()) &&
            y_local < 0.2 && y_local > -0.5)
        {
            minGapLength = x_local;
            minObjIndex  = static_cast<int>(i);
            LOG_INFO("[AEBS] Close object detected at index {}: x_local = {:.2f}", i, x_local);
        }
    }

    double x_local, y_local;
    object_->FreeSpaceDistance(pivot_obj, &y_local, &x_local);

    if (static_cast<unsigned int>(minObjIndex) != i && x_local > 0 &&
        x_local < 1.0 + static_cast<double>(pivot_obj->boundingbox_.dimensions_.length_) + 0.5 * MAX(0.0, currentSpeed_ - pivot_obj->GetSpeed()) &&
        y_local < 0.2 && y_local > -0.5)
    {
        minGapLength = x_local;
        minObjIndex  = static_cast<int>(i);
        LOG_INFO("[AEBS] Close object detected at index {}: x_local = {:.2f}", i, x_local);
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
            LOG_INFO("[AEBS] Lead vehicle {} at index {}", lead->GetName(), minObjIndex);

            // --- AEBS integration ---
            ControllerALKS_R157SM::Model::ObjectInfo obj_info;
            obj_info.obj = lead;

            double relSpeed = currentSpeed_ - lead->GetSpeed();
            if (fabs(relSpeed) < 1e-6)
            {
                LOG_WARN("[AEBS] relSpeed very small ({:.6f}), clamping to 1e-6 to avoid division by zero", relSpeed);
                relSpeed = 1e-6;
            }

            obj_info.ttc = minGapLength / relSpeed;
            LOG_INFO("[AEBS] AEBS update: TTC = {:.2f}, lead speed = {:.2f}, currentSpeed = {:.2f}", obj_info.ttc, lead->GetSpeed(), currentSpeed_);

            try
            {
                aeb_driver_.UpdateAEB(static_cast<Vehicle*>(object_), &obj_info);
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
                LOG_INFO("[AEBS] AEBS ACTIVE! Deceleration {:.2f}, currentSpeed = {:.2f}", aebDec, currentSpeed_);
            }

            if (minGapLength < 1)
            {
                currentSpeed_ = 0.0;
            }

            object_->SetSensorPosition(lead->pos_.GetX(), lead->pos_.GetY(), lead->pos_.GetZ());
        }
    }
    else
    {
        LOG_INFO("[AEBS] No lead vehicle detected.");

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

    Controller::Step(timeStep);

    LOG_INFO("[AEBS] Step end, currentSpeed = {:.2f}", currentSpeed_);
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