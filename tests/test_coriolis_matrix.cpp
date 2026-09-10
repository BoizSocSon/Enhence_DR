#include "nav_dynamics/coriolis_matrix.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_coriolis_matrix..." << std::endl;

    nav_dynamics::VehicleParameters params;
    params.mass = 11.5;
    params.r_G = nav_dynamics::Vector3d(0.0, 0.0, 0.02);
    params.I_b << 0.16, 0.0,  0.0,
                  0.0,  0.35, 0.0,
                  0.0,  0.0,  0.35;
    params.set_added_mass_diagonal(5.5, 8.0, 14.6, 0.05, 0.12, 0.12);

    nav_dynamics::CoriolisMatrixEvaluator evaluator(params);

    nav_dynamics::Vector6d nu;
    nu << 1.2, -0.4, 0.5, 0.1, -0.2, 0.8;
    nav_dynamics::Vector6d nu_r = nu; // assume zero current

    // 1. Check Skew-Symmetry of C_RB: C_RB + C_RB^T = 0
    nav_dynamics::Matrix6d C_rb = evaluator.compute_C_RB(nu);
    nav_dynamics::Matrix6d C_rb_sum = C_rb + C_rb.transpose();
    NAV_TEST_ASSERT(C_rb_sum.isZero(1e-9), "C_RB must be strictly skew-symmetric!");

    // 2. Check Skew-Symmetry of C_A: C_A + C_A^T = 0
    nav_dynamics::Matrix6d C_a = evaluator.compute_C_A(nu_r);
    nav_dynamics::Matrix6d C_a_sum = C_a + C_a.transpose();
    NAV_TEST_ASSERT(C_a_sum.isZero(1e-9), "C_A must be strictly skew-symmetric!");

    // 3. Check Energy Conservation: s^T * C(nu) * s = 0 for arbitrary vector s
    nav_dynamics::Matrix6d C_total = evaluator.compute_C(nu, nu_r);
    double work = nu.dot(C_total * nu);
    NAV_TEST_ASSERT(std::abs(work) < 1e-9, "Coriolis force must perform zero mechanical work!");

    // 4. Verify vanishing C_3 for 3-DOF ROV {u, w, r} when sway=roll=pitch=0
    nav_dynamics::Vector6d nu_3dof;
    nu_3dof << 1.5, 0.0, -0.8, 0.0, 0.0, 0.6; // only u, w, r non-zero
    auto config_3dof = nav_dynamics::DofConfig::make_rov_3dof();
    nav_dynamics::MatrixNd C_3 = evaluator.compute_reduced(config_3dof, nu_3dof, nu_3dof);

    NAV_TEST_ASSERT(C_3.isZero(1e-9), "Reduced 3-DOF Coriolis matrix C_3 must vanish identically for {u, w, r}!");

    std::cout << "[TEST] test_coriolis_matrix PASSED!" << std::endl;
    return 0;
}
