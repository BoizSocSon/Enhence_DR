#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

/**
 * @brief Tính toán và quản lý ma trận khối lượng tổng của hệ thống:
 * M = M_RB + M_A
 * trong đó M_RB là ma trận khối lượng vật rắn và M_A là ma trận khối lượng gia tăng thủy động học.
 */
class MassMatrixEvaluator {
public:
    MassMatrixEvaluator() = default;
    explicit MassMatrixEvaluator(const VehicleParameters& params);

    /// Cập nhật các thông số phương tiện và tính toán lại các ma trận được lưu đệm
    void set_parameters(const VehicleParameters& params);

    /// Lấy các thông số phương tiện hiện tại
    [[nodiscard]] const VehicleParameters& parameters() const { return params_; }

    /// Ma trận khối lượng vật rắn 6x6 đầy đủ M_RB
    [[nodiscard]] const Matrix6d& M_RB() const { return M_RB_; }

    /// Ma trận khối lượng gia tăng 6x6 đầy đủ M_A
    [[nodiscard]] const Matrix6d& M_A() const { return M_A_; }

    /// Ma trận khối lượng tổng 6x6 đầy đủ M = M_RB + M_A
    [[nodiscard]] const Matrix6d& M_total() const { return M_total_; }

    /// Kiểm tra ma trận M_total có xác định dương hay không
    [[nodiscard]] bool is_positive_definite() const;

    /// Giải phương trình tuyến tính M * x = b cho 6-DOF
    [[nodiscard]] Vector6d solve(const Vector6d& b) const;

    /// Tính ma trận khối lượng thu giảm n x n cho cấu hình DOF đã cho
    [[nodiscard]] MatrixNd compute_reduced(const DofConfig& config) const;

    /// Giải hệ phương trình M_r * x_r = b_r cho không gian thu giảm n-DOF
    [[nodiscard]] VectorNd solve_reduced(const DofConfig& config, const VectorNd& b_r) const;

    /// Các hàm tĩnh hỗ trợ tính toán
    static Matrix6d compute_M_RB(double mass, const Vector3d& r_G, const Matrix3d& I_b);

private:
    void update_matrices();

    VehicleParameters params_;
    Matrix6d M_RB_ = Matrix6d::Zero();
    Matrix6d M_A_ = Matrix6d::Zero();
    Matrix6d M_total_ = Matrix6d::Zero();
};

} // namespace nav_dynamics
