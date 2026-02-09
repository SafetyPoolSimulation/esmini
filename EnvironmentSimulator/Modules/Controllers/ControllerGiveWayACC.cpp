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
 * This controller simulates a simple Adaptive Cruise Control
 */

#include "ControllerGiveWayACC.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "Roadmanager.hpp"
#include "playerbase.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerGiveWayACC(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerGiveWayACC(initArgs);
}

ControllerGiveWayACC::ControllerGiveWayACC(InitArgs* args)
    : Controller(args),
      active_(false),
      timeGap_(1.5),
      setSpeed_(0),
      lateralDist_(5.0),
      currentSpeed_(0),
      setSpeedSet_(false),
      virtual_(false)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG);

    if (args && args->properties && args->properties->ValueExists("timeGap"))
    {
        timeGap_ = strtod(args->properties->GetValueStr("timeGap"));
    }
    if (args && args->properties && args->properties->ValueExists("setSpeed"))
    {
        setSpeed_    = strtod(args->properties->GetValueStr("setSpeed"));
        setSpeedSet_ = true;
    }
    if (args && args->properties && args->properties->ValueExists("lateralDist"))
    {
        lateralDist_ = strtod(args->properties->GetValueStr("lateralDist"));
    }
    if (args && args->properties && !args->properties->ValueExists("mode"))
    {
        // Default mode for this controller is additive
        // which will use speed set by other actions as setSpeed
        // in override mode setSpeed is set explicitly (if missing
        // the current speed when controller is activated will be
        // used as setSpeed)
        mode_ = ControlOperationMode::MODE_ADDITIVE;
    }
    if (args && args->properties && args->properties->ValueExists("virtual"))
    {
        virtual_ = args->properties->GetValueStr("virtual") == "true" ? true : false;
    }
}

void ControllerGiveWayACC::Init()
{
    Controller::Init();
}

void ControllerGiveWayACC::InitPostPlayer()
{
    // Uncomment line below to enable example how to add sensors. Press 'r' to visualize sensor frustum.
    // player_->AddObjectSensor(object_, 4.0, 0.0, 0.5, 0.0, 1.0, 50.0, 1.2, 100);
}

void ControllerGiveWayACC::Step(double timeStep)
{
    const double minDist            = 3.0;
    const double accelerationFactor = 0.7;

    // Base desired speed
    double targetSpeed = setSpeed_;

    // ----------------------------------------------------
    // 1. Detect if Ego is approaching a junction
    // ----------------------------------------------------
    bool   approachingJunction = false;
    double distToJunction      = LARGE_NUMBER;

    int                     egoRoadId = object_->pos_.GetTrackId();
    roadmanager::Road*      egoRoad   = object_->pos_.GetRoadById(egoRoadId);
    roadmanager::OpenDrive* od        = object_->pos_.GetOpenDrive();

    if (egoRoad && od)
    {
        for (int i = 0; i < od->GetNumOfRoads(); i++)
        {
            roadmanager::Road* r = od->GetRoadByIdx(i);
            if (!r)
                continue;

            if (r->GetJunction() >= 0 && egoRoad->IsSuccessor(r))
            {
                approachingJunction = true;
                distToJunction      = egoRoad->GetLength() - object_->pos_.GetS();
                break;
            }
        }
    }

    // ----------------------------------------------------
    // 2. RIGHT-OF-WAY CHECK (FRONT-RIGHT BOX ONLY)
    // ----------------------------------------------------
    bool mustStop = false;

    constexpr double GIVEWAY_START_DIST = 30.0;
    constexpr double X_MAX              = 20.0;  // forward
    constexpr double Y_MAX              = 10.0;  // right

    if (approachingJunction && distToJunction < GIVEWAY_START_DIST)
    {
        for (auto other : entities_->object_)
        {
            if (!other || other == object_)
                continue;

            double x_local, y_local;
            object_->FreeSpaceDistance(other, &y_local, &x_local);

            // STRICT front-right box
            if (x_local > 0.0 && x_local < X_MAX && y_local > 0.0 && y_local < Y_MAX && other->pos_.GetVelLong() > GIVEWAY_MIN_SPEED)
            {
                mustStop = true;
                break;
            }
        }
    }

    // ----------------------------------------------------
    // 3. HARD SAFETY: VEHICLE DIRECTLY IN FRONT
    // ----------------------------------------------------
    if (!mustStop)
    {
        // braking distance + buffer
        double emergencyDist = minDist + 0.5 * currentSpeed_ * currentSpeed_ / object_->GetMaxDeceleration() + 1.0;  // buffer

        for (auto other : entities_->object_)
        {
            if (!other || other == object_)
                continue;

            double x_local, y_local;
            object_->FreeSpaceDistance(other, &y_local, &x_local);

            // STRICTLY in front lane
            if (x_local > 0.0 && x_local < emergencyDist && fabs(y_local) < lateralDist_ * 0.4)
            {
                mustStop = true;
                break;
            }
        }
    }

    // ----------------------------------------------------
    // 4. APPLY STOP LOGIC
    // ----------------------------------------------------
    if (mustStop)
    {
        targetSpeed = 0.0;
    }
    else
    {
        // ------------------------------------------------
        // 5. ACC FOLLOWING (optional lead vehicle)
        // ------------------------------------------------
        double  minGap = LARGE_NUMBER;
        Object* lead   = nullptr;

        double lookaheadDist = MAX(50.0, 2 * minDist - pow(currentSpeed_, 2) / -object_->GetMaxDeceleration());

        for (auto other : entities_->object_)
        {
            if (!other || other == object_)
                continue;

            roadmanager::PositionDiff diff;
            if (!object_->pos_.Delta(&other->pos_, diff, false, lookaheadDist))
                continue;

            if (diff.dLaneId != 0 || diff.ds <= 0 || fabs(diff.dt) > lateralDist_)
                continue;

            if (diff.ds < minGap)
            {
                minGap = diff.ds;
                lead   = other;
            }
        }

        if (lead)
        {
            double followDist = minDist + timeGap_ * fabs(MAX(currentSpeed_, lead->GetSpeed()));

            if (minGap < followDist)
            {
                targetSpeed = MIN(targetSpeed, lead->GetSpeed());
            }
        }
    }

    // ----------------------------------------------------
    // 6. FINAL SPEED CONTROLLER (THIS IS THE FIX)
    // ----------------------------------------------------
    double acc = (targetSpeed - currentSpeed_) * accelerationFactor * object_->GetMaxAcceleration();

    acc = CLAMP(acc, -object_->GetMaxDeceleration(), object_->GetMaxAcceleration());

    currentSpeed_ += acc * timeStep;
    currentSpeed_ = CLAMP(currentSpeed_, 0.0, setSpeed_);

    // ----------------------------------------------------
    // 7. APPLY MOTION
    // ----------------------------------------------------
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
}

int ControllerGiveWayACC::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
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

void ControllerGiveWayACC::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}
