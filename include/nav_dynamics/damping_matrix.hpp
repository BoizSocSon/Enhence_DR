#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

/**
 * @brief Computes the hydrodynamic damping matrix:
 * D(nu_r) = D_l + D_q(nu_r)
 * where D_l is linear damping (skin friction) and D_q is nonlinear quadratic damping (form drag).
 */
class DampingMatrixEvaluator {
public:
    DampingMatrixEvaluator() = default;
    explicit DampingMatrixEvaluator(const VehicleParameters& params);

    /// Update vehicle parameters
    void set_parameters(const VehicleParameters& params);

    /// Get linear damping matrix D_l
    [[nodiscard]] const Matrix6d& D_linear() const { return params_.D_l; }

    /// Compute quadratic damping matrix D_q(nu_r)
    [[nodiscard]] Matrix6d compute_D_quadratic(const Vector6d& nu_r) const;

    /// Compute total damping matrix D(nu_r) = D_l + D_q(nu_r)
    [[nodiscard]] Matrix6d compute_D(const Vector6d& nu_r) const;

    /// Compute damping force/moment vector tau_D = D(nu_r) * nu_r
    [[nodiscard]] Vector6d compute_damping_force(const Vector6d& nu_r) const;

    /// Compute reduced n x n damping matrix for a given DOF configuration
    [[nodiscard]] MatrixNd compute_reduced(const DofConfig& config, const Vector6d& nu_r) const;

    /// Compute reduced n x 1 damping force vector for a given DOF configuration
    [[nodiscard]] VectorNd compute_damping_force_reduced(const DofConfig& config, const Vector6d& nu_r) const;

    /// Check if the damping matrix is strictly dissipative (nu_r^T * D * nu_r > 0)
    [[nodiscard]] bool is_dissipative(const Vector6d& nu_r) const;

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
