#include "nav_dynamics/config_loader.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/dof_config.hpp"
#include "nav_dynamics/mass_matrix.hpp"
#include "nav_dynamics/coriolis_matrix.hpp"
#include "nav_dynamics/damping_matrix.hpp"
#include "nav_dynamics/restoring_force.hpp"
#include "nav_dynamics/thruster_allocation.hpp"
#include "nav_dynamics/dynamic_model.hpp"
#include "nav_dynamics/ros_adapter.hpp"
#include "test_common.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_dof_transformation_integration..." << std::endl;

    // 1. Nạp file cấu hình rov_params.yaml
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
    const auto& transformer = cfg.dof_transformer;
    const auto P_cfg = cfg.dof_config();

    std::cout << "[TEST] 1. Kiểm tra ma trận chuyển đổi nạp từ YAML..." << std::endl;
    NAV_TEST_ASSERT(transformer.reduced_dim() == 3, "Transformer reduced_dim must be 3!");
    NAV_TEST_ASSERT(P_cfg.dim() == 3, "DofConfig dim must be 3!");
    NAV_TEST_ASSERT(transformer.is_orthogonal(), "Transformer T must be orthogonal (T * T^T = I_3)!");

    // Ma trận T từ file YAML:
    // Hàng 0: Surge (u) -> [1, 0, 0, 0, 0, 0]
    // Hàng 1: Heave (w) -> [0, 0, 1, 0, 0, 0]
    // Hàng 2: Yaw   (r) -> [0, 0, 0, 0, 0, 1]
    const auto& T = transformer.T_matrix();
    NAV_TEST_ASSERT(std::abs(T(0, 0) - 1.0) < 1e-9, "T(0,0) must map Surge (u)");
    NAV_TEST_ASSERT(std::abs(T(1, 2) - 1.0) < 1e-9, "T(1,2) must map Heave (w)");
    NAV_TEST_ASSERT(std::abs(T(2, 5) - 1.0) < 1e-9, "T(2,5) must map Yaw (r)");

    NAV_TEST_ASSERT(transformer.is_dof_active(nav_dynamics::DofIndex::SURGE), "Surge must be active");
    NAV_TEST_ASSERT(transformer.is_dof_active(nav_dynamics::DofIndex::HEAVE), "Heave must be active");
    NAV_TEST_ASSERT(transformer.is_dof_active(nav_dynamics::DofIndex::YAW), "Yaw must be active");
    NAV_TEST_ASSERT(!transformer.is_dof_active(nav_dynamics::DofIndex::SWAY), "Sway must be inactive");

    // 2. Kiểm tra mô-đun MassMatrixEvaluator với DofTransformer
    std::cout << "[TEST] 2. Kiểm tra mô-đun MassMatrixEvaluator..." << std::endl;
    nav_dynamics::MassMatrixEvaluator mass_eval(cfg.vehicle_params);
    nav_dynamics::MatrixNd M_r = mass_eval.compute_reduced(transformer);
    NAV_TEST_ASSERT(M_r.rows() == 3 && M_r.cols() == 3, "M_r must be 3x3");

    // M_r đường chéo: (m + X_udot), (m + Z_wdot), (Izz + N_rdot)
    double expected_M00 = cfg.vehicle_params.mass + cfg.vehicle_params.M_A(0, 0);
    double expected_M11 = cfg.vehicle_params.mass + cfg.vehicle_params.M_A(2, 2);
    double expected_M22 = cfg.vehicle_params.I_b(2, 2) + cfg.vehicle_params.M_A(5, 5);
    NAV_TEST_ASSERT(std::abs(M_r(0, 0) - expected_M00) < 1e-6, "M_r(0,0) mismatch");
    NAV_TEST_ASSERT(std::abs(M_r(1, 1) - expected_M11) < 1e-6, "M_r(1,1) mismatch");
    NAV_TEST_ASSERT(std::abs(M_r(2, 2) - expected_M22) < 1e-6, "M_r(2,2) mismatch");

    nav_dynamics::VectorNd b_r(3);
    b_r << 10.0, -15.0, 5.0;
    nav_dynamics::VectorNd acc_sol = mass_eval.solve_reduced(transformer, b_r);
    NAV_TEST_ASSERT((M_r * acc_sol).isApprox(b_r, 1e-6), "solve_reduced must solve M_r * x = b_r");

    // 3. Kiểm tra mô-đun CoriolisMatrixEvaluator với DofTransformer
    std::cout << "[TEST] 3. Kiểm tra mô-đun CoriolisMatrixEvaluator..." << std::endl;
    nav_dynamics::CoriolisMatrixEvaluator coriolis_eval(cfg.vehicle_params);
    nav_dynamics::Vector6d nu_6d;
    nu_6d << 0.5, 0.0, -0.2, 0.0, 0.0, 0.1;
    nav_dynamics::MatrixNd C_r = coriolis_eval.compute_reduced(transformer, nu_6d, nu_6d);
    NAV_TEST_ASSERT(C_r.rows() == 3 && C_r.cols() == 3, "C_r must be 3x3");
    nav_dynamics::MatrixNd C_skew = C_r + C_r.transpose();
    NAV_TEST_ASSERT(C_skew.isZero(1e-9), "C_r must be skew-symmetric");

    nav_dynamics::VectorNd tau_C_r = coriolis_eval.compute_coriolis_force_reduced(transformer, nu_6d, nu_6d);
    nav_dynamics::Vector6d tau_C_6d = coriolis_eval.compute_coriolis_force(nu_6d, nu_6d);
    NAV_TEST_ASSERT(tau_C_r.isApprox(transformer.transform_vector(tau_C_6d), 1e-9), "tau_C_r must equal T * tau_C_6d");

    // 4. Kiểm tra mô-đun DampingMatrixEvaluator với DofTransformer
    std::cout << "[TEST] 4. Kiểm tra mô-đun DampingMatrixEvaluator..." << std::endl;
    nav_dynamics::DampingMatrixEvaluator damping_eval(cfg.vehicle_params);
    nav_dynamics::MatrixNd D_r = damping_eval.compute_reduced(transformer, nu_6d);
    NAV_TEST_ASSERT(D_r.rows() == 3 && D_r.cols() == 3, "D_r must be 3x3");
    NAV_TEST_ASSERT(nu_6d.head<3>().dot(D_r * transformer.transform_vector(nu_6d)) > 0.0, "D_r must be dissipative");

    nav_dynamics::VectorNd tau_D_r = damping_eval.compute_damping_force_reduced(transformer, nu_6d);
    nav_dynamics::Vector6d tau_D_6d = damping_eval.compute_damping_force(nu_6d);
    NAV_TEST_ASSERT(tau_D_r.isApprox(transformer.transform_vector(tau_D_6d), 1e-9), "tau_D_r must equal T * tau_D_6d");

    // 5. Kiểm tra mô-đun RestoringForceEvaluator với DofTransformer
    std::cout << "[TEST] 5. Kiểm tra mô-đun RestoringForceEvaluator..." << std::endl;
    nav_dynamics::RestoringForceEvaluator restoring_eval(cfg.vehicle_params);
    nav_dynamics::KinematicState state;
    state.pos_ned << 0.0, 0.0, 5.0; // sâu 5m
    state.euler_rpy << 0.05, -0.02, 0.3; // góc nghiêng nhỏ
    state.update_quaternion_from_euler();

    nav_dynamics::VectorNd g_r = restoring_eval.compute_reduced(transformer, state);
    nav_dynamics::Vector6d g_6d = restoring_eval.compute_g_state(state);
    NAV_TEST_ASSERT(g_r.isApprox(transformer.transform_restoring(g_6d), 1e-9), "g_r must equal T * g_6d");

    // 6. Kiểm tra mô-đun ThrusterAllocation với DofTransformer
    std::cout << "[TEST] 6. Kiểm tra mô-đun ThrusterAllocation..." << std::endl;
    nav_dynamics::MatrixNd B_r = cfg.thruster_allocation.compute_reduced_B(transformer);
    NAV_TEST_ASSERT(B_r.rows() == 3 && B_r.cols() == 3, "B_r must be 3x3 (3 DOFs x 3 Thrusters)");

    nav_dynamics::VectorNd u_thrust(3);
    u_thrust << 15.0, 15.0, -10.0; // [TL, TR, TV]
    nav_dynamics::VectorNd tau_alloc_r = cfg.thruster_allocation.forward_allocation_reduced(transformer, u_thrust);
    NAV_TEST_ASSERT(tau_alloc_r.size() == 3, "tau_alloc_r must have 3 elements");
    NAV_TEST_ASSERT(tau_alloc_r(0) > 0.0, "Surge thrust should be positive for forward thrusts");

    nav_dynamics::VectorNd u_inversed = cfg.thruster_allocation.inverse_allocation_reduced(transformer, tau_alloc_r);
    NAV_TEST_ASSERT(u_inversed.isApprox(u_thrust, 1e-6), "Inverse allocation must reproduce thrust vector");

    // 7. Kiểm tra mô-đun DynamicModel với DofTransformer đã nạp
    std::cout << "[TEST] 7. Kiểm tra mô-đun DynamicModel..." << std::endl;
    nav_dynamics::DynamicModel model(cfg.vehicle_params, transformer, cfg.thruster_allocation);
    NAV_TEST_ASSERT(model.reduced_dim() == 3, "Model reduced_dim must be 3");

    nav_dynamics::Vector6d tau_cmd;
    tau_cmd << 25.0, 0.0, 12.0, 0.0, 0.0, 1.5; // Điều khiển trong Surge, Heave, Yaw

    nav_dynamics::Vector6d acc_6d = model.compute_forward_dynamics(state, tau_cmd);
    NAV_TEST_ASSERT(std::abs(acc_6d(1)) < 1e-9, "Inactive Sway acceleration must be 0");
    NAV_TEST_ASSERT(std::abs(acc_6d(3)) < 1e-9, "Inactive Roll acceleration must be 0");
    NAV_TEST_ASSERT(std::abs(acc_6d(4)) < 1e-9, "Inactive Pitch acceleration must be 0");

    nav_dynamics::VectorNd tau_cmd_r = transformer.transform_wrench(tau_cmd);
    nav_dynamics::VectorNd acc_r = model.compute_forward_dynamics_reduced(state, tau_cmd_r);
    NAV_TEST_ASSERT(std::abs(acc_r(0) - acc_6d(0)) < 1e-9, "Surge acceleration must match");
    NAV_TEST_ASSERT(std::abs(acc_r(1) - acc_6d(2)) < 1e-9, "Heave acceleration must match");
    NAV_TEST_ASSERT(std::abs(acc_r(2) - acc_6d(5)) < 1e-9, "Yaw acceleration must match");

    // Động lực học nghịch trong 3DOF
    nav_dynamics::VectorNd tau_recomputed = model.compute_inverse_dynamics_reduced(state, acc_r);
    NAV_TEST_ASSERT(tau_recomputed.isApprox(tau_cmd_r, 1e-6), "Inverse dynamics 3DOF must reconstruct tau_r");

    // Bước tích phân RK4
    nav_dynamics::KinematicState sim_state = state;
    model.step_rk4(sim_state, tau_cmd, 0.05);
    NAV_TEST_ASSERT(sim_state.nu(0) != 0.0, "State nu(0) should advance after RK4 step");

    // 8. Kiểm tra mô-đun RosAdapter chuyển đổi 6DOF <-> 3DOF
    std::cout << "[TEST] 8. Kiểm tra mô-đun RosAdapter..." << std::endl;
    nav_dynamics::VectorNd twist_3d = nav_dynamics::RosAdapter::to_reduced_twist(nu_6d, transformer);
    NAV_TEST_ASSERT(twist_3d.size() == 3, "Twist 3D size must be 3");
    NAV_TEST_ASSERT(std::abs(twist_3d(0) - nu_6d(0)) < 1e-9, "Twist surge match");
    NAV_TEST_ASSERT(std::abs(twist_3d(1) - nu_6d(2)) < 1e-9, "Twist heave match");
    NAV_TEST_ASSERT(std::abs(twist_3d(2) - nu_6d(5)) < 1e-9, "Twist yaw match");

    nav_dynamics::Vector6d nu_reconstructed = nav_dynamics::RosAdapter::from_reduced_twist(twist_3d, transformer);
    NAV_TEST_ASSERT(std::abs(nu_reconstructed(0) - nu_6d(0)) < 1e-9, "Reconstructed surge match");
    NAV_TEST_ASSERT(std::abs(nu_reconstructed(1)) < 1e-9, "Reconstructed sway must be 0");
    NAV_TEST_ASSERT(std::abs(nu_reconstructed(2) - nu_6d(2)) < 1e-9, "Reconstructed heave match");

    nav_dynamics::TwistPOD pod = nav_dynamics::RosAdapter::to_rov_3dof_twist_pod(1.2, -0.4, 0.15);
    NAV_TEST_ASSERT(std::abs(pod.linear.x - 1.2) < 1e-9, "POD linear.x match");
    NAV_TEST_ASSERT(std::abs(pod.linear.y - 0.0) < 1e-9, "POD linear.y must be 0");
    NAV_TEST_ASSERT(std::abs(pod.linear.z - (-0.4)) < 1e-9, "POD linear.z match");
    NAV_TEST_ASSERT(std::abs(pod.angular.z - 0.15) < 1e-9, "POD angular.z match");

    double u_out = 0, w_out = 0, r_out = 0;
    nav_dynamics::RosAdapter::from_rov_3dof_twist_pod(pod, u_out, w_out, r_out);
    NAV_TEST_ASSERT(std::abs(u_out - 1.2) < 1e-9, "Extracted u match");
    NAV_TEST_ASSERT(std::abs(w_out - (-0.4)) < 1e-9, "Extracted w match");
    NAV_TEST_ASSERT(std::abs(r_out - 0.15) < 1e-9, "Extracted r match");

    std::cout << "[TEST] test_dof_transformation_integration PASSED SUCCESSFUL!" << std::endl;
    return 0;
}
