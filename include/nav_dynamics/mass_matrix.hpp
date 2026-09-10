#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

/**
 * @brief Computes and manages the total system mass matrix:
 * M = M_RB + M_A
 * where M_RB is the rigid-body mass matrix and M_A is the hydrodynamic added mass matrix.
 */
class MassMatrixEvaluator {
public:
    MassMatrixEvaluator() = default;
    explicit MassMatrixEvaluator(const VehicleParameters& params);

    /// Update vehicle parameters and recompute cached matrices
    void set_parameters(const VehicleParameters& params);

    /// Get current vehicle parameters
    [[nodiscard]] const VehicleParameters& parameters() const { return params_; }

    /// Full 6x6 Rigid-Body mass matrix M_RB
    [[nodiscard]] const Matrix6d& M_RB() const { return M_RB_; }

    /// Full 6x6 Added mass matrix M_A
    [[nodiscard]] const Matrix6d& M_A() const { return M_A_; }

    /// Full 6x6 Total mass matrix M = M_RB + M_A
    [[nodiscard]] const Matrix6d& M_total() const { return M_total_; }

    /// Check if M_total is positive-definite
    [[nodiscard]] bool is_positive_definite() const;

    /// Solve M * x = b for 6-DOF
    [[nodiscard]] Vector6d solve(const Vector6d& b) const;

    /// Compute reduced n x n mass matrix for a given DOF configuration
    [[nodiscard]] MatrixNd compute_reduced(const DofConfig& config) const;

    /// Solve M_r * x_r = b_r for reduced n-DOF
    [[nodiscard]] VectorNd solve_reduced(const DofConfig& config, const VectorNd& b_r) const;

    /// Static computation helpers
    static Matrix6d compute_M_RB(double mass, const Vector3d& r_G, const Matrix3d& I_b);

private:
    void update_matrices();

    VehicleParameters params_;
    Matrix6d M_RB_ = Matrix6d::Zero();
    Matrix6d M_A_ = Matrix6d::Zero();
    Matrix6d M_total_ = Matrix6d::Zero();
};

} // namespace nav_dynamics
