#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/thruster_allocation.hpp"
#include <string>
#include <vector>
#include <map>

namespace nav_dynamics {

/**
 * @brief Các thông số môi trường toàn cục trong hệ quy chiếu NED
 */
struct NedEnvironment {
    double gravity = 9.80665;                          ///< Gia tốc trọng trường [m/s^2] (+z hướng xuống)
    double fluid_density = 1025.0;                     ///< Khối lượng riêng của nước [kg/m^3] (nước biển ~1025, nước ngọt ~1000)
    double surface_atmospheric_pressure = 101325.0;    ///< Áp suất khí quyển tại mặt nước [Pa]
    Vector3d nominal_ocean_current = Vector3d::Zero(); ///< Vận tốc dòng chảy trong hệ quy chiếu NED [m/s]
};

/**
 * @brief Các thông số đặc thù của ArduSub (AP_Motors6DOF & cánh tay đòn các IMU)
 */
struct ArduSubConfig {
    std::string frame_type = "CUSTOM_3THRUSTER";
    double pwm_min = 1100.0;
    double pwm_max = 1900.0;
    double pwm_trim = 1500.0;
    double pwm_deadband = 25.0;

    /// Các véc-tơ cánh tay đòn của tối đa 5 IMU tương đối so với CoM trong hệ thân tàu [m]
    std::map<std::string, Vector3d> imu_lever_arms;
};

/**
 * @brief Cấu trúc chứa toàn bộ cấu hình đã nạp của ROV
 */
struct RovConfig {
    std::string vehicle_name = "Custom_Team_ROV";
    VehicleParameters vehicle_params;
    ThrusterAllocation thruster_allocation;
    DofTransformer dof_transformer;
    DofConfig dof_config;
    NedEnvironment ned_env;
    ArduSubConfig ardusub_cfg;
};

/**
 * @brief Bộ nạp cấu hình phân tích cú pháp tệp YAML (như rov_params.yaml)
 * và điền các thông số vật lý của ROV, cài đặt ArduSub, và bộ biến đổi bậc tự do (DOF transformer).
 */
class ConfigLoader {
public:
    /// Nạp toàn bộ cấu hình ROV từ một tệp YAML
    static RovConfig load_from_yaml(const std::string& filepath);

    /// Nạp nhanh chỉ các thông số vật lý và thủy động học (VehicleParameters) từ tệp YAML
    static VehicleParameters load_vehicle_parameters(const std::string& filepath);

    /// Nạp nhanh bộ biến đổi bậc tự do (DofTransformer) từ tệp YAML
    static DofTransformer load_dof_transformer(const std::string& filepath);

    /// Nạp nhanh cấu hình bậc tự do (DofConfig) từ tệp YAML
    static DofConfig load_dof_config(const std::string& filepath);

    /// Lưu cấu hình ROV ra một tệp YAML
    static void save_to_yaml(const RovConfig& config, const std::string& filepath);
};

} // namespace nav_dynamics
