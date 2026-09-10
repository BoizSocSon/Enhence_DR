#include "nav_dynamics/dynamic_model.hpp"
#include "nav_dynamics/ros_adapter.hpp"
#include "test_common.hpp"
#include <iostream>
#include <cmath>

int main() {
    std::cout << "[TEST] Running test_dynamic_model..." << std::endl;

    nav_dynamics::VehicleParameters params;
    params.mass = 11.5;
    params.volume = 11.5 / 1025.0; // neutral buoyancy
    params.set_linear_damping_diagonal(4.03, 6.22, 11.17, 0.07, 0.07, 0.07);
    params.set_quadratic_damping_diagonal(18.18, 21.66, 36.99, 1.55, 1.55, 1.55);

    nav_dynamics::DynamicModel model_6dof(params, nav_dynamics::DofConfig::make_6dof());
    nav_dynamics::DynamicModel model_3dof(params, nav_dynamics::DofConfig::make_rov_3dof());

    // 1. Forward vs Inverse Dynamics Consistency
    nav_dynamics::KinematicState state;
    state.nu << 0.8, -0.1, 0.3, 0.02, -0.01, 0.15;
    state.euler_rpy << 0.05, -0.02, 0.5;
    state.update_quaternion_from_euler();

    nav_dynamics::Vector6d tau_in;
    tau_in << 25.0, 5.0, -10.0, 0.5, -1.0, 3.0;

    // Forward dynamics -> acceleration
    nav_dynamics::Vector6d acc = model_6dof.compute_forward_dynamics_6d(state, tau_in);

    // Inverse dynamics -> recovered wrench
    nav_dynamics::Vector6d tau_recovered = model_6dof.compute_inverse_dynamics(state, acc);
    NAV_TEST_ASSERT(tau_in.isApprox(tau_recovered, 1e-9), "Inverse dynamics must match forward dynamics!");

    // 2. Dynamic Breakdown consistency
    nav_dynamics::DynamicBreakdown bd = model_6dof.evaluate_breakdown(state, tau_in);
    NAV_TEST_ASSERT(bd.acceleration_6d.isApprox(acc, 1e-9), "Breakdown acceleration must match!");
    nav_dynamics::Vector6d net_recomputed = tau_in - bd.coriolis_force - bd.damping_force - bd.restoring_force;
    NAV_TEST_ASSERT(bd.net_wrench.isApprox(net_recomputed, 1e-9), "Net wrench mismatch!");

    // 3. Thruster Allocation Consistency (TL, TR, TV)
    auto thruster_alloc = nav_dynamics::ThrusterAllocation::make_project_rov_3thruster(0.15, 0.12, 0.0);
    nav_dynamics::VectorNd thrusts(3);
    thrusts << 15.0, 12.0, -8.0; // TL, TR, TV
    nav_dynamics::Vector6d tau_thrust = thruster_alloc.forward_allocation(thrusts);

    // Inverse allocation for 3-DOF ROV
    auto cfg_3dof = nav_dynamics::DofConfig::make_rov_3dof();
    nav_dynamics::VectorNd tau_3dof = cfg_3dof.reduce_vector(tau_thrust);
    nav_dynamics::VectorNd thrusts_recovered = thruster_alloc.inverse_allocation_reduced(cfg_3dof, tau_3dof);
    NAV_TEST_ASSERT(thrusts.isApprox(thrusts_recovered, 1e-6), "Inverse thruster allocation failed!");

    // 4. Numerical Integration (RK4) towards Terminal Velocity in 3-DOF ROV mode
    // Apply constant surge thrust tau_x = 20 N from rest.
    // Theoretical terminal velocity: 18.18 * u^2 + 4.03 * u - 20 = 0 -> u_term ~ 0.9439 m/s
    nav_dynamics::KinematicState sim_state;
    nav_dynamics::Vector6d tau_step = nav_dynamics::Vector6d::Zero();
    tau_step(0) = 20.0; // 20 N surge thrust

    double dt = 0.01;
    for (int step = 0; step < 1000; ++step) { // simulate 10 seconds
        model_3dof.step_rk4(sim_state, tau_step, dt);
    }

    double u_final = sim_state.nu(0);
    double u_expected = (-4.03 + std::sqrt(4.03 * 4.03 + 4.0 * 18.18 * 20.0)) / (2.0 * 18.18);
    NAV_TEST_ASSERT(std::abs(u_final - u_expected) < 0.01, "RK4 simulation failed to reach expected terminal velocity!");

    // 5. ROS Adapter POD Conversions
    nav_dynamics::TwistPOD twist_pod = nav_dynamics::RosAdapter::to_twist_pod(sim_state.nu);
    NAV_TEST_ASSERT(std::abs(twist_pod.linear.x - sim_state.nu(0)) < 1e-9, "TwistPOD mismatch!");
    nav_dynamics::Vector6d nu_from_pod = nav_dynamics::RosAdapter::from_twist_pod(twist_pod);
    NAV_TEST_ASSERT(nu_from_pod.isApprox(sim_state.nu, 1e-9), "nu_from_pod mismatch!");

    nav_dynamics::WrenchPOD wrench_pod = nav_dynamics::RosAdapter::to_wrench_pod(tau_in);
    nav_dynamics::Vector6d tau_from_pod = nav_dynamics::RosAdapter::from_wrench_pod(wrench_pod);
    NAV_TEST_ASSERT(tau_from_pod.isApprox(tau_in, 1e-9), "tau_from_pod mismatch!");

    nav_dynamics::OdometryPOD odom_pod = nav_dynamics::RosAdapter::to_odometry_pod(sim_state);
    nav_dynamics::KinematicState state_from_odom;
    nav_dynamics::RosAdapter::from_odometry_pod(odom_pod, state_from_odom);
    NAV_TEST_ASSERT(state_from_odom.pos_ned.isApprox(sim_state.pos_ned, 1e-9), "Odom pos mismatch!");
    NAV_TEST_ASSERT(state_from_odom.nu.isApprox(sim_state.nu, 1e-9), "Odom nu mismatch!");

    // Flat array copy
    double raw_buf[6];
    nav_dynamics::RosAdapter::to_flat_6d(sim_state.nu, raw_buf);
    nav_dynamics::Vector6d nu_from_flat = nav_dynamics::RosAdapter::from_flat_6d(raw_buf);
    NAV_TEST_ASSERT(nu_from_flat.isApprox(sim_state.nu, 1e-9), "Flat array mismatch!");

    std::cout << "[TEST] test_dynamic_model PASSED!" << std::endl;
    return 0;
}
