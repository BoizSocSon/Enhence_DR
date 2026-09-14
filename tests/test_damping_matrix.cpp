#include "nav_dynamics/damping_matrix.hpp"
#include "nav_dynamics/dof_config.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_damping_matrix..." << std::endl;

    nav_dynamics::VehicleParameters params;
    params.set_linear_damping_diagonal(4.03, 6.22, 11.17, 0.07, 0.07, 0.07);
    params.set_quadratic_damping_diagonal(18.18, 21.66, 36.99, 1.55, 1.55, 1.55);

    nav_dynamics::DampingMatrixEvaluator evaluator(params);

    // 1. Kiểm tra tính tiêu tán năng lượng
    nav_dynamics::Vector6d nu_r;
    nu_r << 0.5, -0.2, 0.3, 0.05, -0.05, 0.4;
    NAV_TEST_ASSERT(evaluator.is_dissipative(nu_r), "Damping must be strictly dissipative!");

    // 2. Tính véc-tơ lực cản
    nav_dynamics::Vector6d f_d = evaluator.compute_damping_force(nu_r);

    // Kiểm tra lực cản chuyển động tiến (surge): (Xu + Xuu * |u|) * u
    double u = nu_r(0);
    double expected_f_u = (4.03 + 18.18 * std::abs(u)) * u;
    NAV_TEST_ASSERT(std::abs(f_d(0) - expected_f_u) < 1e-9, "Surge damping force mismatch!");

    // Kiểm tra lực cản chuyển động nâng/hạ (heave): (Zw + Zww * |w|) * w
    double w = nu_r(2);
    double expected_f_w = (11.17 + 36.99 * std::abs(w)) * w;
    NAV_TEST_ASSERT(std::abs(f_d(2) - expected_f_w) < 1e-9, "Heave damping force mismatch!");

    // Kiểm tra mô-men cản quay trở (yaw): (Nr + Nrr * |r|) * r
    double r = nu_r(5);
    double expected_tau_r = (0.07 + 1.55 * std::abs(r)) * r;
    NAV_TEST_ASSERT(std::abs(f_d(5) - expected_tau_r) < 1e-9, "Yaw damping torque mismatch!");

    // 3. Ma trận cản thu giảm 3-DOF
    auto config_3dof = nav_dynamics::DofConfig::make_rov_3dof();
    nav_dynamics::MatrixNd D_3 = evaluator.compute_reduced(config_3dof.transformer(), nu_r);
    NAV_TEST_ASSERT(D_3.rows() == 3 && D_3.cols() == 3, "D_3 dimensions mismatch!");
    NAV_TEST_ASSERT(std::abs(D_3(0, 0) - (4.03 + 18.18 * std::abs(u))) < 1e-9, "D_3(0,0) mismatch!");
    NAV_TEST_ASSERT(std::abs(D_3(1, 1) - (11.17 + 36.99 * std::abs(w))) < 1e-9, "D_3(1,1) mismatch!");
    NAV_TEST_ASSERT(std::abs(D_3(2, 2) - (0.07 + 1.55 * std::abs(r))) < 1e-9, "D_3(2,2) mismatch!");

    std::cout << "[TEST] test_damping_matrix PASSED!" << std::endl;
    return 0;
}
