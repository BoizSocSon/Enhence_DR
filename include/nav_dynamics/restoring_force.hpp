#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

/**
 * @brief Computes the restoring forces and moments vector g(eta) due to
 * gravity (W = mg) and buoyancy (B = rho * g * nabla).
 */
class RestoringForceEvaluator {
public:
    RestoringForceEvaluator() = default;
    explicit RestoringForceEvaluator(const VehicleParameters& params);

    /// Update vehicle parameters
    void set_parameters(const VehicleParameters& params);

    /// Compute 6D restoring vector g(eta) from rotation matrix R_nb
    [[nodiscard]] Vector6d compute_g(const Matrix3d& R_nb) const;

    /// Compute 6D restoring vector g(eta) from Euler angles (roll phi, pitch theta, yaw psi)
    [[nodiscard]] Vector6d compute_g_euler(double phi, double theta, double psi) const;

    /// Compute 6D restoring vector g(eta) from kinematic state
    [[nodiscard]] Vector6d compute_g_state(const KinematicState& state) const;

    /// Compute reduced n x 1 restoring vector for a given DOF configuration
    [[nodiscard]] VectorNd compute_reduced(const DofConfig& config, const KinematicState& state) const;

    /// Compute net vertical force (W - B)
    [[nodiscard]] double net_submerged_weight() const;

    /// Static calculation helper using R_nb, mass, volume, rho, g, r_G, r_B
    static Vector6d calculate_g(double mass, double volume, double fluid_density, double g_acc,
                                const Vector3d& r_G, const Vector3d& r_B, const Matrix3d& R_nb);

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
