#include <iostream>
#include <cmath>
#include "esminiLib.hpp"

// --- 1. Minimal 3-DOF Fossen Kinetics Class ---
class MarinePhysics {
public:
    // Earth-fixed position (Surge X, Sway Y) and Heading (Yaw H)
    double global_x = 0.0;
    double global_y = 0.0;
    double global_h = 0.0;

    // Body-fixed velocities (Surge u, Sway v, Yaw r)
    double u = 0.0;
    double v = 0.0;
    double r = 0.0;

    void Update(double target_speed, double target_heading_rate, double dt) {
        // Simplified Added Mass & Damping approximation for demonstration
        double mass = 5000.0; 
        double added_mass_surge = 1000.0;
        double surge_inertia = mass + added_mass_surge;
        double linear_damping = 200.0;
        
        // Calculate driving force (Thrust) based on target vs current speed
        double thrust = (target_speed - u) * 5000.0; // P-controller for thrust
        
        // Apply Fossen Surge kinetics (simplified: M*u_dot + D*u = Tau)
        double u_dot = (thrust - (linear_damping * u)) / surge_inertia;
        
        // Update body velocities
        u += u_dot * dt;
        
        // Simplified Yaw kinetics (snapping to rate for this MVP)
        r = target_heading_rate; 

        // Kinematic Transformation: Body-fixed to Earth-fixed (J(psi) * nu)
        global_x += (u * cos(global_h) - v * sin(global_h)) * dt;
        global_y += (u * sin(global_h) + v * cos(global_h)) * dt;
        global_h += r * dt;
    }
};

int main(int argc, char* argv[]) {
    // --- 2. Headless Initialization of esmini ---
    // We pass mock arguments to start esmini without the 3D viewer.
    // Replace "scenario.xosc" with your actual OpenSCENARIO test file.
    const char* esmini_args[] = {
        "marine_sim",
        "--headless",
        "--osc", "scenario.xosc"
    };
    int esmini_argc = sizeof(esmini_args) / sizeof(esmini_args[0]);

    if (SE_InitWithArgs(esmini_argc, esmini_args) != 0) {
        std::cerr << "Failed to initialize esmini." << std::endl;
        return -1;
    }
    
    std::cout << "esmini initialized successfully in headless mode." << std::endl;

    // --- 3. Simulation Setup ---
    MarinePhysics ego_vessel;
    const int EGO_ID = 0; // Assuming Ego is defined first in your .xosc
    const double FIXED_DT = 0.016; // Deterministic 60Hz timestep
    const double SIM_DURATION = 10.0; // Run for 10 seconds

    // Initialize starting position based on esmini's scenario setup
    SE_ScenarioObjectState initial_state;
    if (SE_GetObjectState(EGO_ID, &initial_state) == 0) {
        ego_vessel.global_x = initial_state.x;
        ego_vessel.global_y = initial_state.y;
        ego_vessel.global_h = initial_state.h;
    }

    // --- 4. The Deterministic Execution Loop ---
    int steps = SIM_DURATION / FIXED_DT;
    
    for (int i = 0; i < steps; ++i) {
        // Step A: Extract Target Commands from OpenSCENARIO
        // Here we query esmini for what the scenario WANTS the vessel to do
        SE_ScenarioObjectState current_intent;
        SE_GetObjectState(EGO_ID, &current_intent);
        
        // For Option 1, we pull the target speed from the SpeedAction.
        double target_speed = current_intent.speed;
        
        // (Optional: Extract target steering/yaw rate using SE_GetObjectAngularVelocity if defined)
        double target_yaw_rate = 0.0; 

        // Step B: Calculate Physics
        // Pass the requested rates into the Fossen 3-DOF model
        ego_vessel.Update(target_speed, target_yaw_rate, FIXED_DT);

        // Step C: Override the Trajectory
        // Force esmini to accept the physically calculated drift positions
        // Parameters: ID, X, Y, Z, Heading, Pitch, Roll
        SE_ReportObjectPos(EGO_ID, ego_vessel.global_x, ego_vessel.global_y, 0.0, ego_vessel.global_h, 0.0, 0.0);

        // Step D: Step the Scenario Engine Deterministically
        SE_StepDT(FIXED_DT);

        // Print telemetry to console to prove it is running
        if (i % 60 == 0) { // Print once every simulation second
            std::cout << "Time: " << i * FIXED_DT 
                      << "s | Surge: " << ego_vessel.u 
                      << " m/s | Pos X: " << ego_vessel.global_x 
                      << " Y: " << ego_vessel.global_y << std::endl;
        }
    }

    // Clean up memory and close the simulator
    SE_Close();
    std::cout << "Simulation completed." << std::endl;

    return 0;
}