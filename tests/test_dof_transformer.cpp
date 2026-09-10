#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/mass_matrix.hpp"
#include "nav_dynamics/coriolis_matrix.hpp"
#include "nav_dynamics/damping_matrix.hpp"
#include "nav_dynamics/restoring_force.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_dof_transformer..." << std::endl;

    // 1. Check Presets
    auto t_6dof = nav_dynamics::DofTransformer::make_6dof();
    NAV_TEST_ASSERT(t_6dof.reduced_dim() == 6, "6DOF reduced_dim must be 6!");
    NAV_TEST_ASSERT(t_6dof.is_orthogonal(), "6DOF T must be orthogonal!");

    auto t_3dof = nav_dynamics::DofTransformer::make_rov_3dof();
    NAV_TEST_ASSERT(t_3dof.reduced_dim() == 3, "3DOF ROV reduced_dim must be 3!");
    NAV_TEST_ASSERT(t_3dof.is_dof_active(nav_dynamics::DofIndex::SURGE), "Surge must be active!");
    NAV_TEST_ASSERT(t_3dof.is_dof_active(nav_dynamics::DofIndex::HEAVE), "Heave must be active!");
    NAV_TEST_ASSERT(t_3dof.is_dof_active(nav_dynamics::DofIndex::YAW), "Yaw must be active!");
    NAV_TEST_ASSERT(!t_3dof.is_dof_active(nav_dynamics::DofIndex::SWAY), "Sway must be inactive!");
    NAV_TEST_ASSERT(!t_3dof.is_dof_active(nav_dynamics::DofIndex::ROLL), "Roll must be inactive!");
    NAV_TEST_ASSERT(!t_3dof.is_dof_active(nav_dynamics::DofIndex::PITCH), "Pitch must be inactive!");

    // 2. Right-Orthogonality Check: T * T^T = I_n
    NAV_TEST_ASSERT(t_3dof.is_orthogonal(), "T_3dof * T_3dof^T must equal Identity_3!");

    // 3. Fossen Mass Matrix Reduction: M_r = T * M * T^T
    nav_dynamics::VehicleParameters params;
    params.mass = 11.5;
    params.r_G = nav_dynamics::Vector3d(0.0, 0.0, 0.02);
    params.I_b << 0.16, 0.0,  0.0,
                  0.0,  0.35, 0.0,
                  0.0,  0.0,  0.35;
    params.set_added_mass_diagonal(5.5, 8.0, 14.6, 0.05, 0.12, 0.12);

    nav_dynamics::MassMatrixEvaluator mass_eval(params);
    nav_dynamics::MatrixNd M_r = t_3dof.transform_mass(mass_eval.M_total());

    NAV_TEST_ASSERT(M_r.rows() == 3 && M_r.cols() == 3, "M_r dimensions must be 3x3!");
    // Check diagonal: m + X_udot = 17.0, m + Z_wdot = 26.1, Izz + N_rdot = 0.47
    NAV_TEST_ASSERT(std::abs(M_r(0, 0) - 17.0) < 1e-9, "M_r(0,0) mismatch!");
    NAV_TEST_ASSERT(std::abs(M_r(1, 1) - 26.1) < 1e-9, "M_r(1,1) mismatch!");
    NAV_TEST_ASSERT(std::abs(M_r(2, 2) - 0.47) < 1e-9, "M_r(2,2) mismatch!");
    // Off-diagonals are zero when r_G on z-axis
    NAV_TEST_ASSERT(std::abs(M_r(0, 1)) < 1e-9, "M_r off-diagonal must be 0!");
    NAV_TEST_ASSERT(std::abs(M_r(0, 2)) < 1e-9, "M_r off-diagonal must be 0!");
    NAV_TEST_ASSERT(std::abs(M_r(1, 2)) < 1e-9, "M_r off-diagonal must be 0!");

    // 4. Fossen Coriolis Matrix Reduction: C_r = T * C * T^T
    nav_dynamics::CoriolisMatrixEvaluator cor_eval(params);
    nav_dynamics::Vector6d nu_general;
    nu_general << 1.0, -0.5, 0.3, 0.1, -0.2, 0.4;
    nav_dynamics::Matrix6d C_full = cor_eval.compute_C(nu_general, nu_general);
    nav_dynamics::MatrixNd C_r = t_3dof.transform_coriolis(C_full);

    // Reduced Coriolis must maintain skew-symmetry: C_r + C_r^T = 0
    nav_dynamics::MatrixNd C_r_sum = C_r + C_r.transpose();
    NAV_TEST_ASSERT(C_r_sum.isZero(1e-9), "Reduced Coriolis matrix must be strictly skew-symmetric!");

    // For 3-DOF ROV with sway=roll=pitch=0, C_3 must vanish identically
    nav_dynamics::Vector6d nu_rov_planar;
    nu_rov_planar << 1.5, 0.0, -0.5, 0.0, 0.0, 0.8;
    nav_dynamics::Matrix6d C_rov = cor_eval.compute_C(nu_rov_planar, nu_rov_planar);
    nav_dynamics::MatrixNd C_rov_r = t_3dof.transform_coriolis(C_rov);
    NAV_TEST_ASSERT(C_rov_r.isZero(1e-9), "C_3 must vanish identically for 3-DOF ROV {u,w,r}!");

    // 5. Fossen Damping Matrix Reduction: D_r = T * D * T^T
    nav_dynamics::DampingMatrixEvaluator damp_eval(params);
    nav_dynamics::Matrix6d D_full = damp_eval.compute_D(nu_general);
    nav_dynamics::MatrixNd D_r = t_3dof.transform_damping(D_full);
    NAV_TEST_ASSERT(D_r.rows() == 3 && D_r.cols() == 3, "D_r dimensions must be 3x3!");
    // Check dissipativity of reduced damping: v_r^T * D_r * v_r > 0
    nav_dynamics::VectorNd v_r(3);
    v_r << 0.5, -0.3, 0.2;
    double diss = v_r.dot(D_r * v_r);
    NAV_TEST_ASSERT(diss > 0.0, "Reduced damping matrix must be dissipative!");

    // 6. Vector Transformation and Expansion
    nav_dynamics::Vector6d tau_6d;
    tau_6d << 20.0, -10.0, 15.0, 1.0, -2.0, 3.0;
    nav_dynamics::VectorNd tau_r = t_3dof.transform_wrench(tau_6d);
    NAV_TEST_ASSERT(tau_r.size() == 3, "tau_r size must be 3!");
    NAV_TEST_ASSERT(std::abs(tau_r(0) - 20.0) < 1e-9, "tau_r surge mismatch!");
    NAV_TEST_ASSERT(std::abs(tau_r(1) - 15.0) < 1e-9, "tau_r heave mismatch!");
    NAV_TEST_ASSERT(std::abs(tau_r(2) - 3.0) < 1e-9, "tau_r yaw mismatch!");

    nav_dynamics::Vector6d tau_expanded = t_3dof.expand_vector(tau_r);
    NAV_TEST_ASSERT(std::abs(tau_expanded(0) - 20.0) < 1e-9, "tau_expanded surge mismatch!");
    NAV_TEST_ASSERT(std::abs(tau_expanded(1) - 0.0) < 1e-9, "tau_expanded sway must be 0!");
    NAV_TEST_ASSERT(std::abs(tau_expanded(2) - 15.0) < 1e-9, "tau_expanded heave mismatch!");
    NAV_TEST_ASSERT(std::abs(tau_expanded(3) - 0.0) < 1e-9, "tau_expanded roll must be 0!");
    NAV_TEST_ASSERT(std::abs(tau_expanded(4) - 0.0) < 1e-9, "tau_expanded pitch must be 0!");
    NAV_TEST_ASSERT(std::abs(tau_expanded(5) - 3.0) < 1e-9, "tau_expanded yaw mismatch!");

    // 7. Creation from Names
    auto t_named = nav_dynamics::DofTransformer::from_dof_names({"SURGE", "SWAY", "YAW"});
    NAV_TEST_ASSERT(t_named.reduced_dim() == 3, "t_named dimension must be 3!");
    NAV_TEST_ASSERT(t_named.is_dof_active(nav_dynamics::DofIndex::SWAY), "Sway must be active in named planar!");

    std::cout << "[TEST] test_dof_transformer PASSED!" << std::endl;
    return 0;
}
