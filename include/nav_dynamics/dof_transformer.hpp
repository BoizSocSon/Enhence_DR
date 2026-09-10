#pragma once

#include "nav_dynamics/types.hpp"
#include <vector>
#include <string>

namespace nav_dynamics {

/**
 * @brief Common predefined DOF presets
 */
enum class DofPreset {
    FULL_6DOF,                     ///< 6-DOF (Surge, Sway, Heave, Roll, Pitch, Yaw)
    ROV_3DOF_SURGE_HEAVE_YAW,      ///< 3-DOF {u, w, r} for underwater ROV
    PLANAR_3DOF_SURGE_SWAY_YAW,    ///< 3-DOF {u, v, r} for surface vessel (USV)
    ROV_4DOF,                      ///< 4-DOF {u, v, w, r} (Surge, Sway, Heave, Yaw)
    CUSTOM                         ///< Custom user-defined DOF combination
};

/**
 * @brief Dedicated module that constructs the DOF transformation matrix T in R^{n x 6},
 * and performs the matrix multiplications with Fossen dynamic equations components:
 *   M_r   = T * M * T^T
 *   C_r   = T * C * T^T
 *   D_r   = T * D * T^T
 *   g_r   = T * g
 *   tau_r = T * tau
 *   B_r   = T * B
 * and expansion back to 6D:
 *   v_6d  = T^T * v_r + (I - T^T * T) * v_constrained
 */
class DofTransformer {
public:
    DofTransformer();
    explicit DofTransformer(DofPreset preset);
    explicit DofTransformer(const std::vector<DofIndex>& active_dofs);
    explicit DofTransformer(const MatrixNd& custom_T);

    /// Dimension of the reduced state space n (1 <= n <= 6)
    [[nodiscard]] size_t reduced_dim() const { return reduced_dim_; }

    /// Full 6-DOF dimension (always 6)
    [[nodiscard]] size_t full_dim() const { return 6; }

    /// The transformation matrix T in R^{n x 6}
    [[nodiscard]] const MatrixNd& T_matrix() const { return T_; }

    /// The transpose / pseudo-inverse expansion matrix T^T in R^{6 x n}
    [[nodiscard]] MatrixNd T_transpose() const { return T_.transpose(); }

    /// Active DOF indices (in order of reduced rows 0..n-1)
    [[nodiscard]] const std::vector<DofIndex>& active_dofs() const { return active_dofs_; }

    /// Check if a particular DOF is active
    [[nodiscard]] bool is_dof_active(DofIndex dof) const;

    /// Get the row index (0..n-1) of an active DOF in the reduced space (-1 if inactive)
    [[nodiscard]] int reduced_index_of(DofIndex dof) const;

    /// Human-readable names of active DOFs
    [[nodiscard]] std::vector<std::string> active_dof_names() const;

    /// Verify right-orthogonality property: T * T^T = I_n
    [[nodiscard]] bool is_orthogonal(double tol = 1e-9) const;

    // =========================================================================
    // Fossen Equation Matrix Reduction (T * M * T^T, etc.)
    // =========================================================================

    /// Reduce 6x6 Mass Matrix: M_r = T * M * T^T in R^{n x n}
    [[nodiscard]] MatrixNd transform_mass(const Matrix6d& M) const;

    /// Reduce 6x6 Coriolis Matrix: C_r = T * C * T^T in R^{n x n}
    [[nodiscard]] MatrixNd transform_coriolis(const Matrix6d& C) const;

    /// Reduce 6x6 Damping Matrix: D_r = T * D * T^T in R^{n x n}
    [[nodiscard]] MatrixNd transform_damping(const Matrix6d& D) const;

    /// Reduce 6x1 Force/Moment/Velocity Vector: v_r = T * v in R^{n x 1}
    [[nodiscard]] VectorNd transform_vector(const Vector6d& v) const;

    /// Reduce 6x1 Restoring Force Vector: g_r = T * g in R^{n x 1}
    [[nodiscard]] VectorNd transform_restoring(const Vector6d& g) const;

    /// Reduce 6x1 Control Wrench Vector: tau_r = T * tau in R^{n x 1}
    [[nodiscard]] VectorNd transform_wrench(const Vector6d& tau) const;

    /// Reduce 6 x k Thruster Allocation Matrix: B_r = T * B in R^{n x k}
    [[nodiscard]] MatrixNd transform_thruster_allocation(const MatrixNd& B) const;

    // =========================================================================
    // Expansion Operators back to 6D Space
    // =========================================================================

    /// Expand reduced n x 1 vector back to 6D vector: v_6d = T^T * v_r + (I - T^T*T) * v_constrained
    [[nodiscard]] Vector6d expand_vector(const VectorNd& v_r,
                                         const Vector6d& constrained_vals = Vector6d::Zero()) const;

    /// Expand reduced n x n matrix back to 6x6 matrix: M_6d = T^T * M_r * T
    [[nodiscard]] Matrix6d expand_matrix(const MatrixNd& M_r) const;

    // =========================================================================
    // Static Factory Helpers
    // =========================================================================
    static DofTransformer make_6dof();
    static DofTransformer make_rov_3dof();
    static DofTransformer make_planar_3dof();
    static DofTransformer make_rov_4dof();
    static DofTransformer from_preset(DofPreset preset);
    static DofTransformer from_dof_names(const std::vector<std::string>& dof_names);

private:
    void build_T_from_active_dofs();

    std::vector<DofIndex> active_dofs_;
    size_t reduced_dim_ = 6;
    MatrixNd T_; ///< Transformation matrix (n x 6)
};

} // namespace nav_dynamics
