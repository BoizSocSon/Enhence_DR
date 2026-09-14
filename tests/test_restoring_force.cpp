#include "nav_dynamics/restoring_force.hpp"
#include "nav_dynamics/dof_config.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_restoring_force..." << std::endl;

    nav_dynamics::VehicleParameters params;
    params.mass = 11.5;            // W = 11.5 * 9.80665 = 112.776 N
    params.volume = 0.0116;        // B = 1025 * 9.80665 * 0.0116 = 116.601 N (hơi dương)
    params.fluid_density = 1025.0;
    params.gravity = 9.80665;
    params.r_G = nav_dynamics::Vector3d(0.0, 0.0, 0.02);  // Trọng tâm CoG ở dưới gốc tọa độ (NED +z)
    params.r_B = nav_dynamics::Vector3d(0.0, 0.0, -0.02); // Tâm nổi CoB ở trên gốc tọa độ (NED -z)

    nav_dynamics::RestoringForceEvaluator evaluator(params);

    double W = params.weight();
    double B = params.buoyancy();
    double W_minus_B = W - B;

    // 1. Tư thế thăng bằng (phi = theta = psi = 0)
    nav_dynamics::Vector6d g_level = evaluator.compute_g_euler(0.0, 0.0, 0.0);
    NAV_TEST_ASSERT(std::abs(g_level(0)) < 1e-9, "Surge restoring force must be zero when level!");
    NAV_TEST_ASSERT(std::abs(g_level(1)) < 1e-9, "Sway restoring force must be zero when level!");
    NAV_TEST_ASSERT(std::abs(g_level(2) - (-(W_minus_B))) < 1e-9, "Heave force must be -(W - B)!");
    NAV_TEST_ASSERT(std::abs(g_level(3)) < 1e-9, "Roll moment must be zero when level!");
    NAV_TEST_ASSERT(std::abs(g_level(4)) < 1e-9, "Pitch moment must be zero when level!");
    NAV_TEST_ASSERT(std::abs(g_level(5)) < 1e-9, "Yaw moment must be zero!");

    // 2. Véc-tơ g thu giảm cho ROV 3-DOF
    auto config_3dof = nav_dynamics::DofConfig::make_rov_3dof();
    nav_dynamics::KinematicState state_level;
    nav_dynamics::VectorNd g_3 = evaluator.compute_reduced(config_3dof.transformer(), state_level);
    NAV_TEST_ASSERT(g_3.size() == 3, "g_3 size must be 3!");
    NAV_TEST_ASSERT(std::abs(g_3(0)) < 1e-9, "g_3 surge must be 0!");
    NAV_TEST_ASSERT(std::abs(g_3(1) - (-(W_minus_B))) < 1e-9, "g_3 heave must match -(W-B)!");
    NAV_TEST_ASSERT(std::abs(g_3(2)) < 1e-9, "g_3 yaw must be 0!");

    // 3. Ổn định nghiêng lắc ngang Roll (phi = 10 độ)
    double phi = 10.0 * M_PI / 180.0;
    nav_dynamics::Vector6d g_roll = evaluator.compute_g_euler(phi, 0.0, 0.0);
    NAV_TEST_ASSERT(std::abs(g_roll(3)) > 1e-3, "Restoring roll moment must exist when tilted!");

    std::cout << "[TEST] test_restoring_force PASSED!" << std::endl;
    return 0;
}
