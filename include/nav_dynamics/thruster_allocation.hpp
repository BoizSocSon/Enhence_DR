#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include <vector>

namespace nav_dynamics {

/**
 * @brief Định nghĩa vật lý của một cơ cấu chấp hành động cơ đẩy (thruster)
 */
struct ThrusterUnit {
    std::string name = "thruster";
    Vector3d position_body = Vector3d::Zero();  ///< Vị trí so với CoM trong hệ thân tàu [m]
    Vector3d direction_body = Vector3d::UnitX();///< Véc-tơ đơn vị hướng lực đẩy

    // Các hệ số lực đẩy
    double K_t_fwd = 0.035;       ///< Hệ số lực đẩy tiến
    double K_t_rev = 0.025;       ///< Hệ số lực đẩy lùi
    double prop_diameter = 0.076; ///< Đường kính chân vịt [m]
    double max_thrust_fwd = 50.0; ///< Lực đẩy tiến tối đa [N]
    double max_thrust_rev = 40.0; ///< Lực đẩy lùi tối đa [N]
    double deadband_pwm = 25.0;   ///< Dải chết PWM [us]
    double neutral_pwm = 1500.0;  ///< Xung PWM trung hòa [us]
    double time_constant = 0.0;   ///< Hằng số thời gian trễ bậc 1 [s]

    /// Tính cột wrench 6D: b_j = [e_j; r_j × e_j]
    [[nodiscard]] Vector6d wrench_column() const {
        Vector6d col;
        Vector3d e_unit = direction_body.normalized();
        col.head<3>() = e_unit;
        col.tail<3>() = position_body.cross(e_unit);
        return col;
    }

    /// PWM → Lực đẩy [N]
    [[nodiscard]] double pwm_to_thrust(double pwm) const;

    /// Lực đẩy [N] → PWM
    [[nodiscard]] double thrust_to_pwm(double thrust) const;
};

/**
 * @brief Quản lý bố trí động cơ, ma trận phân bổ lực đẩy B (tau = B * u),
 * và bài toán phân bổ nghịch (u = B_pinv * tau).
 */
class ThrusterAllocation {
public:
    ThrusterAllocation() = default;
    explicit ThrusterAllocation(const std::vector<ThrusterUnit>& thrusters);

    /// Thiết lập cấu hình động cơ
    void set_thrusters(const std::vector<ThrusterUnit>& thrusters);

    /// Số lượng động cơ k
    [[nodiscard]] size_t num_thrusters() const { return thrusters_.size(); }

    /// Danh sách động cơ
    [[nodiscard]] const std::vector<ThrusterUnit>& thrusters() const { return thrusters_; }

    /// Ma trận phân bổ B (6 × k)
    [[nodiscard]] const MatrixNd& B_matrix() const { return B_; }

    /// Phân bổ thuận: tau_6d = B * u
    [[nodiscard]] Vector6d forward_allocation(const VectorNd& thrusts) const;

    /// Phân bổ thuận thu giảm: tau_r = T * B * u
    [[nodiscard]] VectorNd forward_allocation_reduced(const DofTransformer& transformer, const VectorNd& thrusts) const;

    /// Ma trận phân bổ thu giảm n×k: B_r = T * B
    [[nodiscard]] MatrixNd compute_reduced_B(const DofTransformer& transformer) const;

    /// Phân bổ nghịch: u = B_pinv * tau
    [[nodiscard]] VectorNd inverse_allocation(const Vector6d& desired_tau) const;

    /// Phân bổ nghịch thu giảm: u = B_r_pinv * tau_r
    [[nodiscard]] VectorNd inverse_allocation_reduced(const DofTransformer& transformer, const VectorNd& desired_tau_r) const;

    /// PWM → Lực đẩy (véc-tơ)
    [[nodiscard]] VectorNd pwm_to_thrusts(const VectorNd& pwms) const;

    /// Lực đẩy → PWM (véc-tơ)
    [[nodiscard]] VectorNd thrusts_to_pwm(const VectorNd& thrusts) const;

    /// Cập nhật động học trễ bậc 1
    VectorNd step_thruster_dynamics(const VectorNd& target_thrusts, double dt);

    /// Lực đẩy tức thời hiện tại
    [[nodiscard]] const VectorNd& current_thrusts() const { return current_thrusts_; }

    /// Gán lực đẩy tức thời
    void set_current_thrusts(const VectorNd& thrusts) { current_thrusts_ = thrusts; }

    /// Factory: ROV chuẩn 3 động cơ (TL, TR, TV)
    static ThrusterAllocation make_project_rov_3thruster(double l_x = 0.15, double d_y = 0.12, double z_t = 0.0);

private:
    void rebuild_allocation_matrix();

    std::vector<ThrusterUnit> thrusters_;
    MatrixNd B_;          ///< Ma trận phân bổ (6 × k)
    MatrixNd B_pinv_;     ///< Giả nghịch đảo (k × 6)
    VectorNd current_thrusts_;
};

} // namespace nav_dynamics
