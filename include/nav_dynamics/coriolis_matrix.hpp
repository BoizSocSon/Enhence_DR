#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

/**
 * @brief Computes the Coriolis-Centripetal matrix C(nu) = C_RB(nu) + C_A(nu_r)
 * strictly formatted in skew-symmetric representation (C = -C^T, nu^T * C * nu = 0).
 */
class CoriolisMatrixEvaluator {
public:
    CoriolisMatrixEvaluator() = default;
    explicit CoriolisMatrixEvaluator(const VehicleParameters& params);

    /// Set vehicle parameters
    void set_parameters(const VehicleParameters& params);

    /// Compute 6x6 Rigid-Body Coriolis matrix C_RB(nu)
    [[nodiscard]] Matrix6d compute_C_RB(const Vector6d& nu) const;

    /// Compute 6x6 Added Mass Coriolis matrix C_A(nu_r)
    [[nodiscard]] Matrix6d compute_C_A(const Vector6d& nu_r) const;

    /// Compute 6x6 Total Coriolis matrix C(nu, nu_r) = C_RB(nu) + C_A(nu_r)
    [[nodiscard]] Matrix6d compute_C(const Vector6d& nu, const Vector6d& nu_r) const;

    /// Compute total Coriolis force vector tau_C = C_RB(nu)*nu + C_A(nu_r)*nu_r
    [[nodiscard]] Vector6d compute_coriolis_force(const Vector6d& nu, const Vector6d& nu_r) const;

    /// Compute reduced n x n Coriolis matrix for a given DOF configuration
    [[nodiscard]] MatrixNd compute_reduced(const DofConfig& config,
                                          const Vector6d& nu,
                                          const Vector6d& nu_r) const;

    /// Compute reduced n x 1 Coriolis force vector for a given DOF configuration
    [[nodiscard]] VectorNd compute_coriolis_force_reduced(const DofConfig& config,
                                                         const Vector6d& nu,
                                                         const Vector6d& nu_r) const;

    /// Static calculation of C_RB from mass, r_G, I_b, and velocity
    static Matrix6d calculate_C_RB(double mass, const Vector3d& r_G,
                                   const Matrix3d& I_b, const Vector6d& nu);

    /// Static calculation of C_A from M_A and relative velocity
    static Matrix6d calculate_C_A(const Matrix6d& M_A, const Vector6d& nu_r);

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
