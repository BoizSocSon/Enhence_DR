#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"

namespace nav_dynamics {

/**
 * @brief Tính toán véc-tơ lực và mô-men hồi phục g(eta) do
 * trọng lực (W = mg) và lực nổi (B = rho * g * nabla).
 */
class RestoringForceEvaluator {
public:
    RestoringForceEvaluator() = default;
    explicit RestoringForceEvaluator(const VehicleParameters& params);

    /// Cập nhật thông số phương tiện
    void set_parameters(const VehicleParameters& params);

    /// Tính g(eta) từ ma trận quay R_nb
    [[nodiscard]] Vector6d compute_g(const Matrix3d& R_nb) const;

    /// Tính g(eta) từ các góc Euler
    [[nodiscard]] Vector6d compute_g_euler(double phi, double theta, double psi) const;

    /// Tính g(eta) từ trạng thái động học
    [[nodiscard]] Vector6d compute_g_state(const KinematicState& state) const;

    /// Tính g_r = T * g từ trạng thái
    [[nodiscard]] VectorNd compute_reduced(const DofTransformer& transformer, const KinematicState& state) const;

    /// Tính g_r = T * g từ ma trận quay
    [[nodiscard]] VectorNd compute_reduced(const DofTransformer& transformer, const Matrix3d& R_nb) const;

    /// Lực thẳng đứng thuần (W - B)
    [[nodiscard]] double net_submerged_weight() const;

    /// Hàm tĩnh tính g(eta)
    static Vector6d calculate_g(double mass, double volume, double fluid_density, double g_acc,
                                const Vector3d& r_G, const Vector3d& r_B, const Matrix3d& R_nb);

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
