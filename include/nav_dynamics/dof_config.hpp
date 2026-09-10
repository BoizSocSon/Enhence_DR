#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include <vector>
#include <string>

namespace nav_dynamics {

/**
 * @brief Manages Degrees of Freedom (DOF) configuration, active indices,
 * and linear projection operators between 6-DOF and reduced n-DOF spaces.
 */
class DofConfig {
public:
    DofConfig();
    explicit DofConfig(DofPreset preset);
    explicit DofConfig(const std::vector<DofIndex>& active_dofs);

    /// Get current active DOF indices
    [[nodiscard]] const std::vector<DofIndex>& active_dofs() const { return active_dofs_; }

    /// Dimension of the reduced state space (1 <= n <= 6)
    [[nodiscard]] size_t dim() const { return active_dofs_.size(); }

    /// Check if a particular DOF is active
    [[nodiscard]] bool is_active(DofIndex dof) const;

    /// Get the index (0..n-1) of an active DOF in the reduced space, returns -1 if not active
    [[nodiscard]] int reduced_index_of(DofIndex dof) const;

    /// Get projection matrix P in R^{n x 6} such that v_reduced = P * v_full
    [[nodiscard]] const MatrixNd& projection_matrix() const { return P_; }

    /// Dimensional reduction of a 6D vector: v_r = P * v
    [[nodiscard]] VectorNd reduce_vector(const Vector6d& v) const;

    /// Dimensional reduction of a 6x6 matrix: M_r = P * M * P^T
    [[nodiscard]] MatrixNd reduce_matrix(const Matrix6d& M) const;

    /// Reconstruct 6D vector: v_full = P^T * v_r, inactive DOFs set to default_val
    [[nodiscard]] Vector6d expand_vector(const VectorNd& v_r, double default_val = 0.0) const;

    /// Reconstruct 6x6 matrix: M_full = P^T * M_r * P
    [[nodiscard]] Matrix6d expand_matrix(const MatrixNd& M_r) const;

    /// Human readable names of active DOFs
    [[nodiscard]] std::vector<std::string> active_dof_names() const;

    /// Static factory helpers
    static DofConfig make_6dof();
    static DofConfig make_rov_3dof();
    static DofConfig make_planar_3dof();
    static DofConfig make_rov_4dof();

private:
    void rebuild_projection_matrix();

    std::vector<DofIndex> active_dofs_;
    MatrixNd P_; ///< Projection matrix (n x 6)
};

} // namespace nav_dynamics
