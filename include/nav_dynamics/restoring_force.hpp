#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

/**
 * @brief Tính toán véc-tơ lực và mô-men hồi phục g(eta) do
 * trọng lực (W = mg) và lực nổi (B = rho * g * nabla).
 */
class RestoringForceEvaluator {
public:
    RestoringForceEvaluator() = default;
    explicit RestoringForceEvaluator(const VehicleParameters& params);

    /// Cập nhật các thông số phương tiện
    void set_parameters(const VehicleParameters& params);

    /// Tính véc-tơ hồi phục 6D g(eta) từ ma trận quay R_nb
    [[nodiscard]] Vector6d compute_g(const Matrix3d& R_nb) const;

    /// Tính véc-tơ hồi phục 6D g(eta) từ các góc Euler (roll phi, pitch theta, yaw psi)
    [[nodiscard]] Vector6d compute_g_euler(double phi, double theta, double psi) const;

    /// Tính véc-tơ hồi phục 6D g(eta) từ trạng thái động học
    [[nodiscard]] Vector6d compute_g_state(const KinematicState& state) const;

    /// Tính véc-tơ hồi phục thu giảm n x 1 cho cấu hình DOF đã cho
    [[nodiscard]] VectorNd compute_reduced(const DofConfig& config, const KinematicState& state) const;

    /// Tính lực thẳng đứng thuần (W - B) khi chìm dưới nước
    [[nodiscard]] double net_submerged_weight() const;

    /// Hàm tĩnh hỗ trợ tính toán sử dụng R_nb, mass, volume, rho, g, r_G, r_B
    static Vector6d calculate_g(double mass, double volume, double fluid_density, double g_acc,
                                const Vector3d& r_G, const Vector3d& r_B, const Matrix3d& R_nb);

private:
    VehicleParameters params_;
};

} // namespace nav_dynamics
