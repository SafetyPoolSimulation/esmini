#include "ControllerSpeedProfile.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "playerbase.hpp"
#include "logger.hpp"

using namespace scenarioengine;

Controller* scenarioengine::InstantiateControllerSpeedProfile(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);
    return new ControllerSpeedProfile(initArgs);
}

ControllerSpeedProfile::ControllerSpeedProfile(InitArgs* args)
    : Controller(args), 
    t_trigger_(-1.0), 
    currentSpeed_(0),
    setSpeed_(0), 
    setSpeedSet_(false)
{
    operating_domains_ = static_cast<unsigned int>(ControlDomainMasks::DOMAIN_MASK_LONG);

    if (args && args->properties && args->properties->ValueExists("csvPath"))
    {
        csv_path_ = args->properties->GetValueStr("csvPath");
    }
    else
    {
        LOG_WARN("[SPEED_PROFILE] No csvPath property found in .xosc");
    }
}

void ControllerSpeedProfile::Init()
{
    if (csv_path_.empty())
    {
        LOG_ERROR("[SPEED_PROFILE] csvPath is empty, cannot load speed profile");
        return;
    }

    std::ifstream file(csv_path_);
    if (!file.is_open())
    {
        LOG_ERROR("[SPEED_PROFILE] Could not open CSV: {}", csv_path_);
        return;
    }

    std::string line;
    std::getline(file, line);  // skip header row
    std::getline(file, line);  // skip units row

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;
        std::stringstream ss(line);
        std::string       t_str, skip_str, v_str;
        std::getline(ss, t_str, ',');  // column 1: time (s)
        std::getline(ss, skip_str, ',');  // column 2: acceleration (m/s^2) - ignored
        std::getline(ss, v_str, ',');  // column 3: speed (km/h)
        try
        {
            times_.push_back(std::stod(t_str));
            speeds_.push_back(std::stod(v_str));
        }
        catch (...)
        {
            continue;
        }  // skip malformed rows
    }

    LOG_INFO("[SPEED_PROFILE] Loaded {} rows from {}", times_.size(), csv_path_);
    LOG_INFO("[SPEED_PROFILE] Duration: {:.3f}s  v_start: {:.3f} m/s", times_.back(), speeds_.front());

    Controller::Init();
}

void ControllerSpeedProfile::InitPostPlayer()
{
}

double ControllerSpeedProfile::LookupSpeed(double t) const
{
    if (times_.empty())
        return 0.0;
    if (t <= times_.front())
        return speeds_.front();
    if (t >= times_.back())
        return 0.0;

    auto   it   = std::lower_bound(times_.begin(), times_.end(), t);
    int    idx  = static_cast<int>(std::distance(times_.begin(), it)) - 1;
    double frac = (t - times_[idx]) / (times_[idx + 1] - times_[idx]);
    double v    = speeds_[idx] + frac * (speeds_[idx + 1] - speeds_[idx]);

    return std::max(0.0, v);
}

void ControllerSpeedProfile::Step(double timeStep)
{
    if (object_ == nullptr || times_.empty())
        return;

    double simTime = scenario_engine_->getSimulationTime();

    // Record trigger time on first active step
    if (t_trigger_ < 0.0)
    {
        t_trigger_ = simTime;
        LOG_INFO("[SPEED_PROFILE] Activated at t={:.3f}s", t_trigger_);
    }

    double t_rel        = simTime - t_trigger_;
    double target_speed = LookupSpeed(t_rel) / 3.6; // convert km/h to m/s

    LOG_INFO("[SPEED_PROFILE] simTime={:.3f}  t_rel={:.3f}  target_speed={:.3f}", simTime, t_rel, target_speed);

    object_->MoveAlongS(target_speed * timeStep);
    gateway_->updateObjectPos(object_->GetId(), 0.0, &object_->pos_);
    gateway_->updateObjectSpeed(object_->GetId(), 0.0, target_speed);

    Controller::Step(timeStep);
}

int ControllerSpeedProfile::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
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
    
    LOG_INFO("[SPEED_PROFILE] Activate called");

    return 0;

    //LOG_INFO("[SPEED_PROFILE] Activate called");
    //return Controller::Activate(mode);
}

void ControllerSpeedProfile::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}
