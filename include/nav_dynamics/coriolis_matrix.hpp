#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"

namespace nav_dynamics {

/**
 * @brief Tính toán ma trận Coriolis - Hướng tâm C(nu) = C_RB(nu) + C_A(nu_r)
 * theo biểu diễn phản đối xứng (C = -C^T, nu^T * C * nu = 0).
 */
class CoriolisMatrixEvaluator {
public:
    CoriolisMatrixEvaluator() = default;
    explicit CoriolisMatrixEvaluator(const VehicleParameters& params);

    /// Cài đặt thông số phương tiện
    void set_parameters(const VehicleParameters& params);

    /// Tính ma trận Coriolis vật rắn 6x6 C_RB(nu)
    [[nodiscard]] Matrix6d compute_C_RB(const Vector6d& nu) const;

    /// Tính ma trận Coriolis khối lượng gia tăng 6x6 C_A(nu_r)
    [[nodiscard]] Matrix6d compute_C_A(const Vector6d& nu_r) const;

    /// Tính ma trận Coriolis tổng 6x6 C(nu, nu_r)
    [[nodiscard]] Matrix6d compute_C(const Vector6d& nu, const Vector6d& nu_r) const;

    /// Tính véc-tơ lực Coriolis tổng tau_C = C_RB(nu)*nu + C_A(nu_r)*nu_r
    [[nodiscard]] Vector6d compute_coriolis_force(const Vector6d& nu, const Vector6d& nu_r) const;

    /// Tính ma trận Coriolis thu giảm n×n: C_r = T * C * T^T
    [[nodiscard]] MatrixNd compute_reduced(const DofTransformer& transformer,
                                          const Vector6d& nu,
                                          const Vector6d& nu_r) const;

    /// Tính véc-tơ lực Coriolis thu giảm n×1: tau_C_r = T * tau_C
    [[nodiscard]] VectorNd compute_coriolis_force_reduced(const DofTransformer& transformer,
                                                         const Vector6d& nu,
                                                         const Vector6d& nu_r) const;

    /// Hàm tĩnh tính C_RB
    static Matrix6d calculate_C_RB(double mass, const Vector3d& r_G,
                                   const Matrix3d& I_b, const Vector6d& nu);

    /// Hàm tĩnh tính C_A
    static Matrix6d calculate_C_A(const Matrix6d& M_A, const Vector6d& nu_r);

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
