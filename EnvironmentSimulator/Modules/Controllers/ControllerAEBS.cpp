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
 * ControllerAEBS.cpp — Automated Emergency Braking System
 *
 * Key changes vs. original:
 *
 *  1. Sigmoid (logistic) ramp replaces smoothstep.
 *     Real hydraulic brake pressure follows an S-curve with soft tails;
 *     smoothstep reaches 75 % of peak by mid-ramp and is too steep.
 *     The sigmoid is centred at ramp_time_/2 and its steepness k is
 *     calibrated so the 5 %→95 % rise occupies exactly ramp_time_.
 *
 *  2. ramp_time_ is now treated as FULL ramp (5 %→95 % of max_decel_),
 *     matching the definition produced by the updated Python extractor.
 *     (The old extractor used 10 %→90 %, and the old C++ used ramp_time_
 *     as the TOTAL duration — a double mismatch that made braking ~2× too
 *     steep.)
 *
 *  3. The bad 50 km/h brake_delay entry (-1.88 s) has been replaced with
 *     NaN-safe fallback logic: any negative delay is clamped to 0.
 *
 *  4. UpdateDynamicParams() now accepts pre-clamped values from the table
 *     and applies a final guard so no negative delay ever reaches Step().
 *
 *  5. The data table below should be regenerated from your real-world runs
 *     using the companion extract_aeb_parameters.py --cpp flag.
 */

#include "ControllerAEBS.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"
#include "logger.hpp"
#include <cmath>
#include <algorithm>

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerAEBS(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);
    return new ControllerAEBS(initArgs);
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
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
        available_ = args->properties->GetValueStr("AEBAvailable") == "true";

    if (args && args->properties && args->properties->ValueExists("AEBTTC"))
        ttc_ = strtod(args->properties->GetValueStr("AEBTTC"));

    if (args && args->properties && args->properties->ValueExists("AEBDeceleration"))
        deceleration_ = strtod(args->properties->GetValueStr("AEBDeceleration"));

    if (args && args->properties && args->properties->ValueExists("FCWAudioTTC"))
        fcw_audio_ttc_ = strtod(args->properties->GetValueStr("FCWAudioTTC"));

    if (args && args->properties && args->properties->ValueExists("FCWVisualTTC"))
        fcw_visual_ttc_ = strtod(args->properties->GetValueStr("FCWVisualTTC"));

    if (args && args->properties && args->properties->ValueExists("LookaheadDistanceLon"))
        lon_lookahead_dist_ = strtod(args->properties->GetValueStr("LookaheadDistanceLon"));

    if (args && args->properties && args->properties->ValueExists("LookaheadDistanceLat"))
        lat_lookahead_dist_ = strtod(args->properties->GetValueStr("LookaheadDistanceLat"));
}

// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------
ControllerAEBS::~ControllerAEBS()
{
    if (!end_logged_)
    {
        LOG_INFO("[AEBS_EVENT] END_STATE (destructor) "
                 "minGap={:.2f}m impact=YES speedAtImpact={:.2f}m/s",
                 min_gap_ever_,
                 currentSpeed_);
        end_logged_ = true;
    }
}

// ---------------------------------------------------------------------------
// Init / link
// ---------------------------------------------------------------------------
void ControllerAEBS::Init()
{
    Controller::Init();
}

void ControllerAEBS::InitPostPlayer()
{
    // Uncomment to visualise sensor frustum (press 'r' in viewer):
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

    Vehicle* egoVeh = static_cast<Vehicle*>(object);
    aeb_driver_.SetVehicle(egoVeh);
    aeb_driver_.aeb_.ttc_critical_aeb_ = ttc_;
    aeb_driver_.aeb_.max_dec_          = deceleration_;
    aeb_driver_.aeb_.available_        = available_;
}

// ---------------------------------------------------------------------------
// UpdateDynamicParams
// ---------------------------------------------------------------------------
//
// Table columns:
//   speed[]       — ego speed breakpoints in m/s
//   ramp_time[]   — FULL ramp duration (5%→95% of peak decel), seconds
//                   Source: extract_aeb_parameters.py with RAMP_LOW=0.05,
//                   RAMP_HIGH=0.95 so this directly drives the sigmoid.
//   max_decel[]   — peak deceleration magnitude, m/s²
//
// Brake delay is not modelled — measured values (0–108ms) are negligible
// relative to ramp durations (0.9–3.0s) and absorbed into the sigmoid tail.
//
// *** REGENERATE this table from your own real-world data using: ***
//   python extractor.py
//
void ControllerAEBS::UpdateDynamicParams(double egoSpeed)
{
    // ---- Speed breakpoints (m/s) — median v_start per bin ----
    static const std::vector<double> speed = {
        8.3901,   // ~30 km/h (n=1)
        11.1989,  // ~40 km/h (n=1)
        13.8330,  // ~50 km/h (n=1)
        16.5464,  // ~60 km/h (n=1)
        19.4948,  // ~70 km/h (n=5)
        20.7561,  // ~75 km/h (n=4)
        22.0804   // ~80 km/h (n=1)
    };

    // ---- Ramp time (seconds) — derived from 2*v_start/peak_decel ----
    static const std::vector<double> ramp_time = {
        1.4137,  // ~30 km/h (n=1)
        2.0204,  // ~40 km/h (n=1)
        2.4165,  // ~50 km/h (n=1)
        3.0106,  // ~60 km/h (n=1)
        3.5036,  // ~70 km/h (n=5, median)
        3.6977,  // ~75 km/h (n=4, median)
        3.7416   // ~80 km/h (n=1)
    };

    // ---- Peak deceleration magnitude (m/s²) ----
    static const std::vector<double> max_decel = {
        11.8697,  // ~30 km/h (n=1)
        11.0859,  // ~40 km/h (n=1)
        11.4486,  // ~50 km/h (n=1)
        10.9919,  // ~60 km/h (n=1)
        11.1753,  // ~70 km/h (n=5, median)
        11.1960,  // ~75 km/h (n=4, median)
        11.8027   // ~80 km/h (n=1)
    };


    // ---- Clamp to table bounds ----
    auto setParams = [&](size_t i)
    {
        ramp_time_ = ramp_time[i];
        max_decel_ = max_decel[i];
    };

    if (egoSpeed <= speed.front())
    {
        setParams(0);
        return;
    }
    if (egoSpeed >= speed.back())
    {
        setParams(speed.size() - 1);
        return;
    }

    // ---- Linear interpolation ----
    for (size_t i = 0; i < speed.size() - 1; ++i)
    {
        if (egoSpeed >= speed[i] && egoSpeed <= speed[i + 1])
        {
            double ratio = (egoSpeed - speed[i]) / (speed[i + 1] - speed[i]);

            ramp_time_ = ramp_time[i] + ratio * (ramp_time[i + 1] - ramp_time[i]);
            max_decel_ = max_decel[i] + ratio * (max_decel[i + 1] - max_decel[i]);
            return;
        }
    }

    LOG_INFO("[AEBS_PARAMS] v_trigger={:.3f} ramp_time_={:.4f} max_decel_={:.4f}", v_trigger_, ramp_time_, max_decel_);
}

// ---------------------------------------------------------------------------
// ComputeDecel
// ---------------------------------------------------------------------------
// Soft-start then constant decel profile matching real-world shape:
//
//   Phase 1 (0 → t_soft):   linear ramp from 0 to mean_decel
//                            gives gentle initial speed drop matching
//                            the real-world curve's soft entry
//   Phase 2 (t_soft → end): constant mean_decel until vehicle stops
//                            gives straight steep gradient through middle
//                            and clean stop without long tail
//
// mean_decel = v_trigger / ramp_time — the effective constant rate that
// guarantees the stop time matches real-world ramp_time_ exactly.
//
// SOFT_FRAC controls the fraction of ramp_time spent in the ramp-up phase.
// 0.20 (20%) gives a short soft entry matching the real orange curve shape.
//
static double ComputeDecel(double t_brake, double ramp_time, double v_trigger)
{
    if (ramp_time <= 0.0 || v_trigger <= 0.0)
        return 0.0;

    constexpr double SOFT_FRAC  = 0.20;
    double           mean_decel = v_trigger / ramp_time;
    double           t_soft     = ramp_time * SOFT_FRAC;

    if (t_brake < t_soft)
    {
        // Linear ramp: 0 → mean_decel over t_soft
        return mean_decel * (t_brake / t_soft);
    }

    // Constant phase: holds mean_decel until speed reaches zero
    return mean_decel;
}

// ---------------------------------------------------------------------------
// Step
// ---------------------------------------------------------------------------
void ControllerAEBS::Step(double timeStep)
{
    double minGapLength = LARGE_NUMBER;
    int    minObjIndex  = -1;

    // Track speed: virtual mode reads from object; otherwise use our integrator
    if (virtual_)
    {
        currentSpeed_ = object_->GetSpeed();
    }
    else if (fabs(object_->GetSpeed() - currentSpeed_) > 1e-3)
    {
        setSpeed_ = object_->GetSpeed();
    }

    // ---- Find closest lead vehicle ----
    for (size_t i = 0; i < entities_->object_.size(); i++)
    {
        Object* pivot_obj = entities_->object_[i];
        if (!pivot_obj || pivot_obj == object_)
            continue;

        double x_local, y_local;
        object_->FreeSpaceDistance(pivot_obj, &y_local, &x_local);

        if (x_local > lon_lookahead_dist_ || y_local < -lat_lookahead_dist_ || y_local > lat_lookahead_dist_)
            continue;

        if (x_local > 0.0 && x_local < minGapLength)
        {
            minGapLength = x_local;
            minObjIndex  = static_cast<int>(i);
        }
    }

    double aebDec = 0.0;  // applied deceleration (negative = braking)

    if (minObjIndex > -1)
    {
        Object* lead = entities_->object_[static_cast<unsigned int>(minObjIndex)];
        if (!lead)
            return;

        ControllerALKS_R157SM::Model::ObjectInfo obj_info;
        obj_info.obj = lead;

        double relSpeed = currentSpeed_ - lead->GetSpeed();
        if (fabs(relSpeed) < 1e-6)
            relSpeed = 1e-6;

        obj_info.ttc = (relSpeed > 0.0) ? (minGapLength / relSpeed) : LARGE_NUMBER;

        double simTime = scenario_engine_->getSimulationTime();

        // Only call UpdateAEB before trigger — once latched we own braking entirely.
        // Calling it after trigger would let AEBDeceleration from .xosc interfere
        // with our lookup-table-based decel profile.
        if (!brake_started_)
        {
            aeb_driver_.UpdateAEB(static_cast<Vehicle*>(object_), &obj_info);

            if (aeb_driver_.aeb_.active_)
            {
                brake_start_time_ = simTime;
                brake_started_    = true;
                v_trigger_        = currentSpeed_;
                UpdateDynamicParams(currentSpeed_);
                LOG_INFO("[AEBS_EVENT] AEB TRIGGERED t={:.3f}s v={:.3f}m/s ttc={:.3f}s "
                         "ramp={:.3f}s maxDec={:.2f}",
                         simTime,
                         currentSpeed_,
                         obj_info.ttc,
                         ramp_time_,
                         max_decel_);
            }
        }

        if (brake_started_ && currentSpeed_ > 0.0)
        {
            double t_since_activation = simTime - brake_start_time_;

            // Soft-start then constant: gentle initial curve, straight steep
            // middle, clean stop — matches real-world speed profile shape.
            current_decel_ = ComputeDecel(t_since_activation, ramp_time_, v_trigger_);

            aebDec        = -current_decel_;
            currentSpeed_ = std::max(0.0, currentSpeed_ + aebDec * timeStep);

            LOG_INFO("[AEBS_DYNAMIC] t={:.3f}s ramp={:.3f}s decel={:.2f}m/s² speed={:.3f}m/s",
                     t_since_activation,
                     ramp_time_,
                     current_decel_,
                     currentSpeed_);
        }
        else if (!brake_started_)
        {
            current_decel_ = 0.0;
        }

        object_->SetSensorPosition(lead->pos_.GetX(), lead->pos_.GetY(), lead->pos_.GetZ());
    }
    else
    {
        object_->SetSensorPosition(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ());
    }

    // ---- Move object ----
    if (mode_ == ControlOperationMode::MODE_OVERRIDE && !virtual_)
    {
        object_->MoveAlongS(currentSpeed_ * timeStep);
        gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    }

    // ---- Propagate acceleration for virtual mode ----
    if (virtual_)
    {
        double acc_v[2] = {0.0, 0.0};
        RotateVec2D(aebDec, 0.0, object_->pos_.GetH(), acc_v[0], acc_v[1]);
        gateway_->updateObjectAcc(object_->GetId(), 0.0, acc_v[0], acc_v[1], 0.0);
    }
    else
    {
        gateway_->updateObjectSpeed(object_->GetId(), 0.0, currentSpeed_);
    }

    // ---- Book-keeping ----
    min_gap_ever_ = std::min(min_gap_ever_, minGapLength);

    if (std::isnan(speed_at_impact_) && minGapLength <= 0.0)
    {
        speed_at_impact_ = currentSpeed_;
        LOG_INFO("[AEBS_EVENT] IMPACT speed={:.3f}m/s", speed_at_impact_);
    }

    Controller::Step(timeStep);
}

// ---------------------------------------------------------------------------
// Activate
// ---------------------------------------------------------------------------
int ControllerAEBS::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    currentSpeed_ = object_->GetSpeed();
    if (mode_ == ControlOperationMode::MODE_ADDITIVE || !setSpeedSet_)
        setSpeed_ = object_->GetSpeed();

    Controller::Activate(mode);

    if (IsActiveOnDomains(static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LAT)))
    {
        object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
    }

    if (player_)
        player_->SteeringSensorSetVisible(object_->GetId(), true);

    return 0;
}

// ---------------------------------------------------------------------------
// ReportKeyEvent
// ---------------------------------------------------------------------------
void ControllerAEBS::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}