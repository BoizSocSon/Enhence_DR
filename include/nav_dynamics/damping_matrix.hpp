#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"

namespace nav_dynamics {

/**
 * @brief Tính toán ma trận cản thủy động học:
 * D(nu_r) = D_l + D_q(nu_r)
 * trong đó D_l là cản tuyến tính và D_q là cản bậc hai phi tuyến.
 */
class DampingMatrixEvaluator {
public:
    DampingMatrixEvaluator() = default;
    explicit DampingMatrixEvaluator(const VehicleParameters& params);

    /// Cập nhật thông số phương tiện
    void set_parameters(const VehicleParameters& params);

    /// Lấy ma trận cản tuyến tính D_l
    [[nodiscard]] const Matrix6d& D_linear() const { return params_.D_l; }

    /// Tính ma trận cản bậc hai D_q(nu_r) — vectorized bằng Eigen
    [[nodiscard]] Matrix6d compute_D_quadratic(const Vector6d& nu_r) const;

    /// Tính ma trận cản tổng D(nu_r) = D_l + D_q(nu_r)
    [[nodiscard]] Matrix6d compute_D(const Vector6d& nu_r) const;

    /// Tính véc-tơ lực cản tau_D = D(nu_r) * nu_r
    [[nodiscard]] Vector6d compute_damping_force(const Vector6d& nu_r) const;

    /// Tính ma trận cản thu giảm n×n: D_r = T * D * T^T
    [[nodiscard]] MatrixNd compute_reduced(const DofTransformer& transformer, const Vector6d& nu_r) const;

    /// Tính véc-tơ lực cản thu giảm n×1: tau_D_r = T * tau_D
    [[nodiscard]] VectorNd compute_damping_force_reduced(const DofTransformer& transformer, const Vector6d& nu_r) const;

    /// Kiểm tra tính tiêu tán năng lượng (nu_r^T * D * nu_r > 0)
    [[nodiscard]] bool is_dissipative(const Vector6d& nu_r) const;

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
