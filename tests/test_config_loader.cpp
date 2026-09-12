#include "nav_dynamics/config_loader.hpp"
#include "test_common.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_config_loader..." << std::endl;

    std::string config_path = "config/rov_params.yaml";
    if (!std::ifstream(config_path).good()) {
        config_path = "../config/rov_params.yaml";
    }
    if (!std::ifstream(config_path).good()) {
        config_path = "../../config/rov_params.yaml";
    }
    if (!std::ifstream(config_path).good()) {
        config_path = "/home/stevehoang/Navigation_System_Library/config/rov_params.yaml";
    }
    nav_dynamics::RovConfig cfg = nav_dynamics::ConfigLoader::load_from_yaml(config_path);

    // Kiểm tra nạp nhanh chỉ VehicleParameters (1 lần gọi)
    nav_dynamics::VehicleParameters v_params = nav_dynamics::ConfigLoader::load_vehicle_parameters(config_path);
    NAV_TEST_ASSERT(std::abs(v_params.mass - 11.5) < 1e-9, "load_vehicle_parameters mass mismatch!");
    NAV_TEST_ASSERT(std::abs(v_params.M_A(0, 0) - 5.5) < 1e-9, "load_vehicle_parameters M_A(0,0) mismatch!");

    // 1. Kiểm tra các thông số vật rắn của phương tiện
    NAV_TEST_ASSERT(cfg.vehicle_name == "Custom_Team_ROV", "Vehicle name mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.mass - 11.5) < 1e-9, "Mass mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.volume - 0.0116) < 1e-9, "Volume mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.r_G.z() - 0.02) < 1e-9, "r_G.z mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.r_B.z() - (-0.02)) < 1e-9, "r_B.z mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.I_b(0, 0) - 0.16) < 1e-9, "Ixx mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.I_b(1, 1) - 0.35) < 1e-9, "Iyy mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.I_b(2, 2) - 0.35) < 1e-9, "Izz mismatch!");

    // 2. Kiểm tra các đạo hàm thủy động học
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.M_A(0, 0) - 5.5) < 1e-9, "Added mass X_udot mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.M_A(2, 2) - 14.6) < 1e-9, "Added mass Z_wdot mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.M_A(5, 5) - 0.12) < 1e-9, "Added mass N_rdot mismatch!");

    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.D_l(0, 0) - 4.03) < 1e-9, "Linear damping Xu mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.D_l(2, 2) - 11.17) < 1e-9, "Linear damping Zw mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.D_l(5, 5) - 0.07) < 1e-9, "Linear damping Nr mismatch!");

    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.D_q(0, 0) - 18.18) < 1e-9, "Quadratic damping Xuu mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.D_q(2, 2) - 36.99) < 1e-9, "Quadratic damping Zww mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.vehicle_params.D_q(5, 5) - 1.55) < 1e-9, "Quadratic damping Nrr mismatch!");

    // 3. Kiểm tra các thông số tương thích ArduSub
    NAV_TEST_ASSERT(cfg.ardusub_cfg.frame_type == "CUSTOM_3THRUSTER", "Frame type mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.ardusub_cfg.pwm_min - 1100.0) < 1e-9, "PWM min mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.ardusub_cfg.pwm_max - 1900.0) < 1e-9, "PWM max mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.ardusub_cfg.pwm_trim - 1500.0) < 1e-9, "PWM trim mismatch!");
    NAV_TEST_ASSERT(cfg.ardusub_cfg.imu_lever_arms.size() == 5, "Must have 5 IMU lever arms!");
    NAV_TEST_ASSERT(cfg.ardusub_cfg.imu_lever_arms.count("imu1") == 1, "Must contain imu1!");
    NAV_TEST_ASSERT(cfg.ardusub_cfg.imu_lever_arms.count("imu5") == 1, "Must contain imu5 (CoM)!");

    // 4. Kiểm tra phân bổ lực đẩy động cơ
    NAV_TEST_ASSERT(cfg.thruster_allocation.num_thrusters() == 3, "Must have 3 thrusters loaded!");

    // 5. Kiểm tra bộ biến đổi bậc tự do (DOF Transformer)
    NAV_TEST_ASSERT(cfg.dof_transformer.reduced_dim() == 3, "Loaded DOF transformer dim must be 3!");
    NAV_TEST_ASSERT(cfg.dof_transformer.is_dof_active(nav_dynamics::DofIndex::SURGE), "Surge must be active!");
    NAV_TEST_ASSERT(cfg.dof_transformer.is_dof_active(nav_dynamics::DofIndex::HEAVE), "Heave must be active!");
    NAV_TEST_ASSERT(cfg.dof_transformer.is_dof_active(nav_dynamics::DofIndex::YAW), "Yaw must be active!");

    // Kiểm tra chính xác các phần tử ma trận transform_matrix_3DOF_ được nạp từ YAML
    const auto& T = cfg.dof_transformer.T_matrix();
    NAV_TEST_ASSERT(T.rows() == 3 && T.cols() == 6, "Matrix T size must be 3x6!");
    NAV_TEST_ASSERT(std::abs(T(0, 0) - 1.0) < 1e-9, "T(0,0) must be 1.0 for Surge!");
    NAV_TEST_ASSERT(std::abs(T(1, 2) - 1.0) < 1e-9, "T(1,2) must be 1.0 for Heave!");
    NAV_TEST_ASSERT(std::abs(T(2, 5) - 1.0) < 1e-9, "T(2,5) must be 1.0 for Yaw!");
    // Các phần tử khác bằng 0
    NAV_TEST_ASSERT(std::abs(T(0, 1)) < 1e-9 && std::abs(T(0, 2)) < 1e-9, "T row 0 non-surge must be 0!");
    NAV_TEST_ASSERT(std::abs(T(1, 0)) < 1e-9 && std::abs(T(1, 1)) < 1e-9, "T row 1 non-heave must be 0!");
    NAV_TEST_ASSERT(std::abs(T(2, 0)) < 1e-9 && std::abs(T(2, 4)) < 1e-9, "T row 2 non-yaw must be 0!");

    // Kiểm tra nạp độc lập DofTransformer và DofConfig từ file YAML
    auto t_standalone = nav_dynamics::ConfigLoader::load_dof_transformer(config_path);
    NAV_TEST_ASSERT(t_standalone.reduced_dim() == 3, "Standalone load_dof_transformer dim must be 3!");
    NAV_TEST_ASSERT(t_standalone.is_orthogonal(), "Standalone transformer must be orthogonal!");

    auto c_standalone = nav_dynamics::ConfigLoader::load_dof_config(config_path);
    NAV_TEST_ASSERT(c_standalone.dim() == 3, "Standalone load_dof_config dim must be 3!");
    NAV_TEST_ASSERT(c_standalone.is_active(nav_dynamics::DofIndex::SURGE), "Surge active in standalone config!");

    // 6. Kiểm tra các thông số môi trường NED
    NAV_TEST_ASSERT(std::abs(cfg.ned_env.gravity - 9.80665) < 1e-9, "Gravity mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.ned_env.fluid_density - 1025.0) < 1e-9, "Fluid density mismatch!");
    NAV_TEST_ASSERT(std::abs(cfg.ned_env.surface_atmospheric_pressure - 101325.0) < 1e-9, "P_atm mismatch!");

    std::cout << "[TEST] test_config_loader PASSED!" << std::endl;
    return 0;
}
