#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"
#include <vector>

namespace nav_dynamics {

/**
 * @brief Định nghĩa vật lý của một cơ cấu chấp hành động cơ đẩy (thruster)
 */
struct ThrusterUnit {
    std::string name = "thruster";
    Vector3d position_body = Vector3d::Zero();  ///< Vị trí tương đối so với trọng tâm CoM trong hệ thân tàu [m]
    Vector3d direction_body = Vector3d::UnitX();///< Véc-tơ đơn vị hướng lực đẩy trong hệ thân tàu

    // Các hệ số lực đẩy
    double K_t_fwd = 0.035;    ///< Hệ số lực đẩy tiến
    double K_t_rev = 0.025;    ///< Hệ số lực đẩy lùi
    double prop_diameter = 0.076; ///< Đường kính chân vịt/cánh quạt [m] (ví dụ: BlueRobotics T200 ~76mm)
    double max_thrust_fwd = 50.0; ///< Lực đẩy tiến tối đa [N]
    double max_thrust_rev = 40.0; ///< Lực đẩy lùi tối đa [N]
    double deadband_pwm = 25.0;   ///< Dải chết PWM quanh vị trí trung hòa [us] (ví dụ: 1500 +/- 25)
    double neutral_pwm = 1500.0;  ///< Giá trị xung PWM trung hòa [us]

    /// Tính cột véc-tơ lực và mô-men tổng quát 6D: b_j = [e_j; r_j x e_j]
    [[nodiscard]] Vector6d wrench_column() const {
        Vector6d col;
        Vector3d e_unit = direction_body.normalized();
        col.head<3>() = e_unit;
        col.tail<3>() = position_body.cross(e_unit);
        return col;
    }

    /// Chuyển đổi lệnh xung PWM (1100..1900 us) sang lực đẩy [N]
    [[nodiscard]] double pwm_to_thrust(double pwm) const;

    /// Chuyển đổi lực đẩy mong muốn [N] sang xung điều khiển PWM (1100..1900 us)
    [[nodiscard]] double thrust_to_pwm(double thrust) const;
};

/**
 * @brief Quản lý bố trí động cơ, ma trận phân bổ lực đẩy B (tau = B * T),
 * và bài toán phân bổ nghịch đảo (T = B_pinv * tau).
 */
class ThrusterAllocation {
public:
    ThrusterAllocation() = default;
    explicit ThrusterAllocation(const std::vector<ThrusterUnit>& thrusters);

    /// Thiết lập cấu hình các động cơ đẩy
    void set_thrusters(const std::vector<ThrusterUnit>& thrusters);

    /// Số lượng động cơ đẩy k
    [[nodiscard]] size_t num_thrusters() const { return thrusters_.size(); }

    /// Danh sách các động cơ đẩy
    [[nodiscard]] const std::vector<ThrusterUnit>& thrusters() const { return thrusters_; }

    /// Ma trận phân bổ lực đẩy B kích thước 6 x k
    [[nodiscard]] const MatrixNd& B_matrix() const { return B_; }

    /// Phân bổ thuận: tau_6d = B * T
    [[nodiscard]] Vector6d forward_allocation(const VectorNd& thrusts) const;

    /// Phân bổ thuận cho không gian thu giảm: tau_r = P * B * T
    [[nodiscard]] VectorNd forward_allocation_reduced(const DofConfig& config, const VectorNd& thrusts) const;

    /// Phân bổ nghịch: tính lực đẩy các động cơ T từ wrench 6D mong muốn tau
    [[nodiscard]] VectorNd inverse_allocation(const Vector6d& desired_tau) const;

    /// Phân bổ nghịch cho không gian thu giảm: tính lực đẩy các động cơ T từ tau_r mong muốn
    [[nodiscard]] VectorNd inverse_allocation_reduced(const DofConfig& config, const VectorNd& desired_tau_r) const;

    /// Chuyển đổi véc-tơ lệnh xung PWM sang lực đẩy các động cơ
    [[nodiscard]] VectorNd pwm_to_thrusts(const VectorNd& pwms) const;

    /// Chuyển đổi véc-tơ lực đẩy sang lệnh xung PWM tương ứng
    [[nodiscard]] VectorNd thrusts_to_pwm(const VectorNd& thrusts) const;

    /// Hàm tĩnh hỗ trợ tạo cấu hình ROV chuẩn 3 động cơ của dự án (TL, TR, TV)
    static ThrusterAllocation make_project_rov_3thruster(double l_x = 0.15, double d_y = 0.12, double z_t = 0.0);

private:
    void rebuild_allocation_matrix();

    std::vector<ThrusterUnit> thrusters_;
    MatrixNd B_;      ///< Ma trận phân bổ kích thước 6 x k
    MatrixNd B_pinv_; ///< Ma trận giả nghịch đảo kích thước k x 6
};

} // namespace nav_dynamics
