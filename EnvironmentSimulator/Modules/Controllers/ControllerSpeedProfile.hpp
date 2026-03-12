#pragma once

#include <string>
#include "Controller.hpp"
#include "Entities.hpp"
#include "vehicle.hpp"

#define CONTROLLER_SPEED_PROFILE_TYPE_NAME "SpeedProfileController"

namespace scenarioengine
{
    class ControllerSpeedProfile : public Controller
    {
    public:
        ControllerSpeedProfile(InitArgs* args);

        virtual const char* GetTypeName()
        {
            return CONTROLLER_SPEED_PROFILE_TYPE_NAME;
        }
        virtual int GetType()
        {
            return CONTROLLER_TYPE_SPEED_PROFILE;
        }

        void Init();
        void InitPostPlayer();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);

        private:
        double LookupSpeed(double t) const;

        std::string csv_path_;
        std::vector<double> times_;
        std::vector<double> speeds_;
        double              t_trigger_;

        double currentSpeed_;
        double setSpeed_;
        bool setSpeedSet_;
    };

    Controller* InstantiateControllerSpeedProfile(void* args);
}
