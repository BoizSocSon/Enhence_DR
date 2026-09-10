#include "nav_dynamics/dynamic_model.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/config_loader.hpp"
#include "nav_dynamics/ros_adapter.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "===============================================================\n";
    std::cout << "   ROV 6-DOF Dynamic Model & ConfigLoader Simulation Demo\n";
    std::cout << "===============================================================\n\n";

    // 1. Load ROV configuration from YAML file (contains ROV fixed & ArduSub params)
    std::string config_path = "/home/stevehoang/Navigation_System_Library/config/rov_params.yaml";
    std::cout << "[INFO] Loading configuration from: " << config_path << "\n";
    nav_dynamics::RovConfig rov_cfg = nav_dynamics::ConfigLoader::load_from_yaml(config_path);

    std::cout << "[INFO] Vehicle: " << rov_cfg.vehicle_name << ", Mass: " << rov_cfg.vehicle_params.mass << " kg\n";
    std::cout << "[INFO] ArduSub Frame: " << rov_cfg.ardusub_cfg.frame_type
              << ", Thrusters: " << rov_cfg.thruster_allocation.num_thrusters() << "\n";
    std::cout << "[INFO] Active DOFs from DofTransformer (dim=" << rov_cfg.dof_transformer.reduced_dim() << "):\n";
    for (const auto& name : rov_cfg.dof_transformer.active_dof_names()) {
        std::cout << "  - " << name << "\n";
    }
    std::cout << "[INFO] Transformer Orthogonality (T * T^T == I): "
              << (rov_cfg.dof_transformer.is_orthogonal() ? "VALID" : "INVALID") << "\n\n";

    // 2. Initialize Dynamic Model using loaded configuration
    nav_dynamics::DynamicModel rov_model(rov_cfg.vehicle_params,
                                         rov_cfg.dof_transformer,
                                         rov_cfg.thruster_allocation);

    // 3. Initial State & NED Environment current
    nav_dynamics::KinematicState state;
    nav_dynamics::FluidCurrent ocean_current(rov_cfg.ned_env.nominal_ocean_current);

    double dt = 0.05;       // 20 Hz simulation rate
    double sim_time = 5.0;  // 5 seconds
    int total_steps = static_cast<int>(sim_time / dt);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time [s] | Pos (X, Y, Z) [m]     | Vel (u, w, r)        | TL [N] TR [N] TV [N] | Surge Acc\n";
    std::cout << "-----------------------------------------------------------------------------------------\n";

    for (int step = 0; step <= total_steps; ++step) {
        double t = step * dt;

        // Commanded thruster forces
        nav_dynamics::VectorNd thrusts(3);
        if (t < 2.0) {
            // Maneuver 1: Dive down (TV = 8 N) and slow forward (TL=TR=5 N)
            thrusts << 5.0, 5.0, 8.0;
        } else {
            // Maneuver 2: Turn right (TL=15 N, TR=5 N) while maintaining depth
            thrusts << 15.0, 5.0, 4.0;
        }

        // Map thrusters to 6D wrench: tau = B * T
        nav_dynamics::Vector6d tau = rov_cfg.thruster_allocation.forward_allocation(thrusts);

        // Compute forward dynamics & breakdown
        nav_dynamics::DynamicBreakdown bd = rov_model.evaluate_breakdown(state, tau, ocean_current);

        // Step simulation using RK4
        rov_model.step_rk4(state, tau, dt, ocean_current);

        // Print telemetry every 0.5 seconds
        if (step % 10 == 0) {
            std::cout << std::setw(8) << t << " | "
                      << std::setw(6) << state.pos_ned.x() << " "
                      << std::setw(6) << state.pos_ned.y() << " "
                      << std::setw(6) << state.pos_ned.z() << " | "
                      << std::setw(6) << state.nu(0) << " "
                      << std::setw(6) << state.nu(2) << " "
                      << std::setw(6) << state.nu(5) << " | "
                      << std::setw(6) << thrusts(0) << " "
                      << std::setw(6) << thrusts(1) << " "
                      << std::setw(6) << thrusts(2) << " | "
                      << std::setw(9) << bd.acceleration_6d(0) << "\n";
        }
    }

    std::cout << "\n===============================================================\n";
    std::cout << "   ROS Message Standardization Output (Zero-ROS Dependency)\n";
    std::cout << "===============================================================\n";

    // Convert state to ROS POD types
    nav_dynamics::OdometryPOD odom = nav_dynamics::RosAdapter::to_odometry_pod(state);
    nav_dynamics::TwistPOD twist = nav_dynamics::RosAdapter::to_twist_pod(state.nu);
    nav_dynamics::WrenchPOD wrench = nav_dynamics::RosAdapter::to_wrench_pod(state.nu_dot);

    std::cout << "[ROS TwistPOD]  linear=(" << twist.linear.x << ", " << twist.linear.y << ", " << twist.linear.z
              << "), angular=(" << twist.angular.x << ", " << twist.angular.y << ", " << twist.angular.z << ")\n";
    std::cout << "[ROS WrenchPOD] force=(" << wrench.force.x << ", " << wrench.force.y << ", " << wrench.force.z
              << "), torque=(" << wrench.torque.x << ", " << wrench.torque.y << ", " << wrench.torque.z << ")\n";
    std::cout << "[ROS PosePOD]   position=(" << odom.pose.position.x << ", " << odom.pose.position.y << ", " << odom.pose.position.z
              << "), quat=(" << odom.pose.orientation.x << ", " << odom.pose.orientation.y << ", "
              << odom.pose.orientation.z << ", " << odom.pose.orientation.w << ")\n";
    std::cout << "\nSimulation demo finished successfully!\n";

    return 0;
}
