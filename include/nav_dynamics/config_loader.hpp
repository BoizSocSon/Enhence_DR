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
    double gravity = 9.80665;                          ///< Gia tốc trọng trường [m/s^2]
    double fluid_density = 1025.0;                     ///< Khối lượng riêng nước [kg/m^3]
    double surface_atmospheric_pressure = 101325.0;    ///< Áp suất khí quyển [Pa]
    Vector3d nominal_ocean_current = Vector3d::Zero(); ///< Vận tốc dòng chảy NED [m/s]
};

/**
 * @brief Các thông số đặc thù của ArduSub
 */
struct ArduSubConfig {
    std::string frame_type = "CUSTOM_3THRUSTER";
    double pwm_min = 1100.0;
    double pwm_max = 1900.0;
    double pwm_trim = 1500.0;
    double pwm_deadband = 25.0;

    /// Cánh tay đòn IMU so với CoM [m]
    std::map<std::string, Vector3d> imu_lever_arms;
};

/**
 * @brief Cấu trúc chứa toàn bộ cấu hình ROV đã nạp
 */
struct RovConfig {
    std::string vehicle_name = "Custom_Team_ROV";
    VehicleParameters vehicle_params;
    ThrusterAllocation thruster_allocation;
    DofTransformer dof_transformer;
    NedEnvironment ned_env;
    ArduSubConfig ardusub_cfg;

    /// Tương thích ngược: lấy DofConfig từ transformer
    [[nodiscard]] DofConfig dof_config() const { return DofConfig(dof_transformer); }
};

/**
 * @brief Bộ nạp cấu hình phân tích tệp YAML và điền các thông số ROV.
 */
class ConfigLoader {
public:
    /// Nạp toàn bộ cấu hình ROV
    static RovConfig load_from_yaml(const std::string& filepath);

    /// Nạp nhanh VehicleParameters
    static VehicleParameters load_vehicle_parameters(const std::string& filepath);

    /// Nạp nhanh DofTransformer
    static DofTransformer load_dof_transformer(const std::string& filepath);

    /// Nạp nhanh DofConfig
    static DofConfig load_dof_config(const std::string& filepath);

    /// Lưu cấu hình ra YAML
    static void save_to_yaml(const RovConfig& config, const std::string& filepath);
};

} // namespace nav_dynamics
