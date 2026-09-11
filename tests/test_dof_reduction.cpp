#include "nav_dynamics/dof_config.hpp"
#include "nav_dynamics/dynamic_model.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_dof_reduction..." << std::endl;

    // 1. Kiểm tra các cấu hình đặt sẵn (Presets)
    auto cfg_6 = nav_dynamics::DofConfig::make_6dof();
    NAV_TEST_ASSERT(cfg_6.dim() == 6, "6DOF dimension must be 6!");

    auto cfg_3_rov = nav_dynamics::DofConfig::make_rov_3dof();
    NAV_TEST_ASSERT(cfg_3_rov.dim() == 3, "3DOF ROV dimension must be 3!");
    NAV_TEST_ASSERT(cfg_3_rov.is_active(nav_dynamics::DofIndex::SURGE), "Surge must be active!");
    NAV_TEST_ASSERT(cfg_3_rov.is_active(nav_dynamics::DofIndex::HEAVE), "Heave must be active!");
    NAV_TEST_ASSERT(cfg_3_rov.is_active(nav_dynamics::DofIndex::YAW), "Yaw must be active!");
    NAV_TEST_ASSERT(!cfg_3_rov.is_active(nav_dynamics::DofIndex::SWAY), "Sway must not be active!");
    NAV_TEST_ASSERT(!cfg_3_rov.is_active(nav_dynamics::DofIndex::ROLL), "Roll must not be active!");
    NAV_TEST_ASSERT(!cfg_3_rov.is_active(nav_dynamics::DofIndex::PITCH), "Pitch must not be active!");

    auto cfg_3_planar = nav_dynamics::DofConfig::make_planar_3dof();
    NAV_TEST_ASSERT(cfg_3_planar.dim() == 3, "3DOF Planar dimension must be 3!");
    NAV_TEST_ASSERT(cfg_3_planar.is_active(nav_dynamics::DofIndex::SURGE), "Surge must be active!");
    NAV_TEST_ASSERT(cfg_3_planar.is_active(nav_dynamics::DofIndex::SWAY), "Sway must be active!");
    NAV_TEST_ASSERT(cfg_3_planar.is_active(nav_dynamics::DofIndex::YAW), "Yaw must be active!");

    auto cfg_4 = nav_dynamics::DofConfig::make_rov_4dof();
    NAV_TEST_ASSERT(cfg_4.dim() == 4, "4DOF dimension must be 4!");

    // 2. Tính chất của ma trận chiếu: P * P^T = I_n
    const auto& P = cfg_3_rov.projection_matrix();
    nav_dynamics::MatrixNd P_Pt = P * P.transpose();
    NAV_TEST_ASSERT(P_Pt.isApprox(nav_dynamics::MatrixNd::Identity(3, 3), 1e-9), "P * P^T must equal Identity!");

    // 3. Thu giảm và mở rộng véc-tơ
    nav_dynamics::Vector6d v_full;
    v_full << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    nav_dynamics::VectorNd v_red = cfg_3_rov.reduce_vector(v_full);
    NAV_TEST_ASSERT(v_red.size() == 3, "v_red size must be 3!");
    NAV_TEST_ASSERT(std::abs(v_red(0) - 1.0) < 1e-9, "v_red(0) mismatch!"); // surge
    NAV_TEST_ASSERT(std::abs(v_red(1) - 3.0) < 1e-9, "v_red(1) mismatch!"); // heave
    NAV_TEST_ASSERT(std::abs(v_red(2) - 6.0) < 1e-9, "v_red(2) mismatch!"); // yaw

    nav_dynamics::Vector6d v_exp = cfg_3_rov.expand_vector(v_red, 0.0);
    NAV_TEST_ASSERT(std::abs(v_exp(0) - 1.0) < 1e-9, "v_exp surge mismatch!");
    NAV_TEST_ASSERT(std::abs(v_exp(1) - 0.0) < 1e-9, "v_exp sway mismatch!");
    NAV_TEST_ASSERT(std::abs(v_exp(2) - 3.0) < 1e-9, "v_exp heave mismatch!");
    NAV_TEST_ASSERT(std::abs(v_exp(3) - 0.0) < 1e-9, "v_exp roll mismatch!");
    NAV_TEST_ASSERT(std::abs(v_exp(4) - 0.0) < 1e-9, "v_exp pitch mismatch!");
    NAV_TEST_ASSERT(std::abs(v_exp(5) - 6.0) < 1e-9, "v_exp yaw mismatch!");

    // 4. Tính tương đương mô hình động lực học giữa 6-DOF không liên kết chéo và ROV 3-DOF thu giảm
    // Với phương tiện đối xứng không ghép kênh (r_G = 0, r_B = 0), chuyển động 6-DOF thuần túy trong {u, w, r}
    // giống hệt về mặt toán học với mô hình ROV 3-DOF.
    nav_dynamics::VehicleParameters params;
    params.mass = 11.5;
    params.volume = 11.5 / 1025.0; // độ nổi trung tính
    params.r_G.setZero();
    params.r_B.setZero();

    nav_dynamics::DynamicModel model_6dof(params, cfg_6);
    nav_dynamics::DynamicModel model_3dof(params, cfg_3_rov);

    nav_dynamics::KinematicState state;
    state.nu << 0.5, 0.0, -0.2, 0.0, 0.0, 0.1; // chỉ có surge, heave, yaw khác không

    nav_dynamics::Vector6d tau;
    tau << 20.0, 0.0, 10.0, 0.0, 0.0, 2.0; // chỉ có surge, heave, yaw khác không

    nav_dynamics::Vector6d acc_6dof = model_6dof.compute_forward_dynamics_6d(state, tau);
    nav_dynamics::Vector6d acc_from_3dof = model_3dof.compute_forward_dynamics(state, tau);

    // Các bậc tự do hoạt động {0, 2, 5} phải khớp giữa mô hình 6D không liên kết và mô hình 3D thu giảm
    NAV_TEST_ASSERT(std::abs(acc_6dof(0) - acc_from_3dof(0)) < 1e-6, "Surge acceleration must match!");
    NAV_TEST_ASSERT(std::abs(acc_6dof(2) - acc_from_3dof(2)) < 1e-6, "Heave acceleration must match!");
    NAV_TEST_ASSERT(std::abs(acc_6dof(5) - acc_from_3dof(5)) < 1e-6, "Yaw acceleration must match!");

    // Các bậc tự do không hoạt động {1, 3, 4} trong mô hình 3-DOF phải bằng 0
    NAV_TEST_ASSERT(std::abs(acc_from_3dof(1)) < 1e-9, "Inactive Sway acceleration must be 0!");
    NAV_TEST_ASSERT(std::abs(acc_from_3dof(3)) < 1e-9, "Inactive Roll acceleration must be 0!");
    NAV_TEST_ASSERT(std::abs(acc_from_3dof(4)) < 1e-9, "Inactive Pitch acceleration must be 0!");

    std::cout << "[TEST] test_dof_reduction PASSED!" << std::endl;
    return 0;
}
