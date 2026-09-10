#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/thruster_allocation.hpp"
#include <string>
#include <vector>
#include <map>

namespace nav_dynamics {

/**
 * @brief Global NED environment parameters
 */
struct NedEnvironment {
    double gravity = 9.80665;                          ///< Gravity acceleration [m/s^2] (+z down)
    double fluid_density = 1025.0;                     ///< Water density [kg/m^3] (seawater ~1025, freshwater ~1000)
    double surface_atmospheric_pressure = 101325.0;    ///< Atmospheric pressure at water surface [Pa]
    Vector3d nominal_ocean_current = Vector3d::Zero(); ///< Current velocity in NED frame [m/s]
};

/**
 * @brief ArduSub specific parameters (AP_Motors6DOF & Multi-IMU lever arms)
 */
struct ArduSubConfig {
    std::string frame_type = "CUSTOM_3THRUSTER";
    double pwm_min = 1100.0;
    double pwm_max = 1900.0;
    double pwm_trim = 1500.0;
    double pwm_deadband = 25.0;

    /// Lever arm vectors of up to 5 IMUs relative to CoM in body frame [m]
    std::map<std::string, Vector3d> imu_lever_arms;
};

/**
 * @brief Complete loaded ROV configuration container
 */
struct RovConfig {
    std::string vehicle_name = "Custom_Team_ROV";
    VehicleParameters vehicle_params;
    ThrusterAllocation thruster_allocation;
    DofTransformer dof_transformer;
    NedEnvironment ned_env;
    ArduSubConfig ardusub_cfg;
};

/**
 * @brief Configuration loader that parses YAML files (such as rov_params.yaml)
 * and populates ROV physical parameters, ArduSub settings, and DOF transformer.
 */
class ConfigLoader {
public:
    /// Load entire ROV configuration from a YAML file
    static RovConfig load_from_yaml(const std::string& filepath);

    /// Save ROV configuration to a YAML file
    static void save_to_yaml(const RovConfig& config, const std::string& filepath);
};

} // namespace nav_dynamics
