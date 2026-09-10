#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"
#include <vector>

namespace nav_dynamics {

/**
 * @brief Physical definition of a single thruster actuator
 */
struct ThrusterUnit {
    std::string name = "thruster";
    Vector3d position_body = Vector3d::Zero();  ///< Position relative to CoM in body frame [m]
    Vector3d direction_body = Vector3d::UnitX();///< Unit thrust direction vector in body frame

    // Thrust coefficients
    double K_t_fwd = 0.035;    ///< Forward thrust coefficient
    double K_t_rev = 0.025;    ///< Reverse thrust coefficient
    double prop_diameter = 0.076; ///< Propeller diameter [m] (e.g. BlueRobotics T200 ~76mm)
    double max_thrust_fwd = 50.0; ///< Maximum forward thrust [N]
    double max_thrust_rev = 40.0; ///< Maximum reverse thrust [N]
    double deadband_pwm = 25.0;   ///< PWM deadband around neutral [us] (e.g. 1500 +/- 25)
    double neutral_pwm = 1500.0;  ///< Neutral PWM [us]

    /// Compute generalized 6D wrench column vector: b_j = [e_j; r_j x e_j]
    [[nodiscard]] Vector6d wrench_column() const {
        Vector6d col;
        Vector3d e_unit = direction_body.normalized();
        col.head<3>() = e_unit;
        col.tail<3>() = position_body.cross(e_unit);
        return col;
    }

    /// Map PWM command (1100..1900 us) to thrust force [N]
    [[nodiscard]] double pwm_to_thrust(double pwm) const;

    /// Map desired thrust force [N] to PWM command (1100..1900 us)
    [[nodiscard]] double thrust_to_pwm(double thrust) const;
};

/**
 * @brief Manages thruster layout, thruster allocation matrix B (tau = B * T),
 * and inverse allocation (T = B_pinv * tau).
 */
class ThrusterAllocation {
public:
    ThrusterAllocation() = default;
    explicit ThrusterAllocation(const std::vector<ThrusterUnit>& thrusters);

    /// Set thruster configuration
    void set_thrusters(const std::vector<ThrusterUnit>& thrusters);

    /// Number of thrusters k
    [[nodiscard]] size_t num_thrusters() const { return thrusters_.size(); }

    /// List of thrusters
    [[nodiscard]] const std::vector<ThrusterUnit>& thrusters() const { return thrusters_; }

    /// 6 x k allocation matrix B
    [[nodiscard]] const MatrixNd& B_matrix() const { return B_; }

    /// Forward allocation: tau_6d = B * T
    [[nodiscard]] Vector6d forward_allocation(const VectorNd& thrusts) const;

    /// Forward allocation for reduced DOF: tau_r = P * B * T
    [[nodiscard]] VectorNd forward_allocation_reduced(const DofConfig& config, const VectorNd& thrusts) const;

    /// Inverse allocation: compute thruster forces T from desired 6D wrench tau
    [[nodiscard]] VectorNd inverse_allocation(const Vector6d& desired_tau) const;

    /// Inverse allocation for reduced DOF: compute thruster forces T from desired tau_r
    [[nodiscard]] VectorNd inverse_allocation_reduced(const DofConfig& config, const VectorNd& desired_tau_r) const;

    /// Convert vector of PWM commands to thrust forces
    [[nodiscard]] VectorNd pwm_to_thrusts(const VectorNd& pwms) const;

    /// Convert vector of thrust forces to PWM commands
    [[nodiscard]] VectorNd thrusts_to_pwm(const VectorNd& thrusts) const;

    /// Factory helper: create project standard 3-thruster ROV configuration (TL, TR, TV)
    static ThrusterAllocation make_project_rov_3thruster(double l_x = 0.15, double d_y = 0.12, double z_t = 0.0);

private:
    void rebuild_allocation_matrix();

    std::vector<ThrusterUnit> thrusters_;
    MatrixNd B_;      ///< 6 x k allocation matrix
    MatrixNd B_pinv_; ///< k x 6 pseudo-inverse matrix
};

} // namespace nav_dynamics
