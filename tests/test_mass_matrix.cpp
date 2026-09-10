#include "nav_dynamics/mass_matrix.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_mass_matrix..." << std::endl;

    nav_dynamics::VehicleParameters params;
    params.mass = 11.5;
    params.r_G = nav_dynamics::Vector3d(0.0, 0.0, 0.02); // CoG on z-axis
    params.I_b << 0.16, 0.0,  0.0,
                  0.0,  0.35, 0.0,
                  0.0,  0.0,  0.35;
    params.set_added_mass_diagonal(-5.5, -8.0, -14.6, -0.05, -0.12, -0.12);

    nav_dynamics::MassMatrixEvaluator evaluator(params);

    // 1. Test Symmetry of M_RB
    const auto& M_rb = evaluator.M_RB();
    NAV_TEST_ASSERT(M_rb.isApprox(M_rb.transpose(), 1e-9), "M_RB must be symmetric!");

    // 2. Test Symmetry of M_A
    const auto& M_a = evaluator.M_A();
    NAV_TEST_ASSERT(M_a.isApprox(M_a.transpose(), 1e-9), "M_A must be symmetric!");

    // 3. Test Symmetry of M_total
    const auto& M = evaluator.M_total();
    NAV_TEST_ASSERT(M.isApprox(M.transpose(), 1e-9), "M_total must be symmetric!");

    // 4. Test Positive Definiteness
    NAV_TEST_ASSERT(evaluator.is_positive_definite(), "M_total must be positive definite!");

    // 5. Test Linear Solver: M * x = b
    nav_dynamics::Vector6d b;
    b << 10.0, -5.0, 20.0, 1.0, -2.0, 3.0;
    nav_dynamics::Vector6d x = evaluator.solve(b);
    nav_dynamics::Vector6d b_reconstructed = M * x;
    NAV_TEST_ASSERT(b.isApprox(b_reconstructed, 1e-9), "Solver M*x = b must be accurate!");

    // 6. Test 3-DOF ROV Reduced Mass Matrix {u, w, r}
    auto config_3dof = nav_dynamics::DofConfig::make_rov_3dof();
    nav_dynamics::MatrixNd M_3 = evaluator.compute_reduced(config_3dof);
    NAV_TEST_ASSERT(M_3.rows() == 3 && M_3.cols() == 3, "M_3 must be 3x3!");

    // Check diagonal values: m - X_udot = 11.5 + 5.5 = 17.0
    NAV_TEST_ASSERT(std::abs(M_3(0, 0) - (11.5 + 5.5)) < 1e-9, "M_3(0,0) must equal m - X_udot!");
    // m - Z_wdot = 11.5 + 14.6 = 26.1
    NAV_TEST_ASSERT(std::abs(M_3(1, 1) - (11.5 + 14.6)) < 1e-9, "M_3(1,1) must equal m - Z_wdot!");
    // I_zz - N_rdot = 0.35 + 0.12 = 0.47
    NAV_TEST_ASSERT(std::abs(M_3(2, 2) - (0.35 + 0.12)) < 1e-9, "M_3(2,2) must equal I_zz - N_rdot!");

    // Coupling terms should be 0 because r_G_x = r_G_y = 0
    NAV_TEST_ASSERT(std::abs(M_3(0, 1)) < 1e-9, "M_3 off-diagonal must be zero!");
    NAV_TEST_ASSERT(std::abs(M_3(0, 2)) < 1e-9, "M_3 off-diagonal must be zero!");
    NAV_TEST_ASSERT(std::abs(M_3(1, 2)) < 1e-9, "M_3 off-diagonal must be zero!");

    std::cout << "[TEST] test_mass_matrix PASSED!" << std::endl;
    return 0;
}
