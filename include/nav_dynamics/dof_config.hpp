#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include <vector>
#include <string>

namespace nav_dynamics {

/**
 * @brief Quản lý cấu hình bậc tự do (DOF), các chỉ số hoạt động,
 * và toán tử chiếu tuyến tính giữa không gian 6 bậc tự do và không gian thu giảm n bậc tự do.
 */
class DofConfig {
public:
    DofConfig();
    explicit DofConfig(DofPreset preset);
    explicit DofConfig(const std::vector<DofIndex>& active_dofs);
    explicit DofConfig(const MatrixNd& custom_P, const std::vector<DofIndex>& active_dofs = {});
    explicit DofConfig(const DofTransformer& transformer);

    /// Chuyển đổi sang DofTransformer
    [[nodiscard]] DofTransformer to_transformer() const;

    /// Lấy danh sách các chỉ số DOF đang hoạt động hiện tại
    [[nodiscard]] const std::vector<DofIndex>& active_dofs() const { return active_dofs_; }

    /// Số chiều của không gian trạng thái thu giảm (1 <= n <= 6)
    [[nodiscard]] size_t dim() const { return active_dofs_.size(); }

    /// Kiểm tra xem một bậc tự do cụ thể có đang hoạt động hay không
    [[nodiscard]] bool is_active(DofIndex dof) const;

    /// Lấy chỉ số (0..n-1) của một DOF đang hoạt động trong không gian thu giảm, trả về -1 nếu không hoạt động
    [[nodiscard]] int reduced_index_of(DofIndex dof) const;

    /// Lấy ma trận chiếu P thuộc R^{n x 6} sao cho v_reduced = P * v_full
    [[nodiscard]] const MatrixNd& projection_matrix() const { return P_; }

    /// Thu giảm số chiều của véc-tơ 6D: v_r = P * v
    [[nodiscard]] VectorNd reduce_vector(const Vector6d& v) const;

    /// Thu giảm số chiều của ma trận 6x6: M_r = P * M * P^T
    [[nodiscard]] MatrixNd reduce_matrix(const Matrix6d& M) const;

    /// Tái tạo véc-tơ 6D: v_full = P^T * v_r, các DOF không hoạt động gán bằng default_val
    [[nodiscard]] Vector6d expand_vector(const VectorNd& v_r, double default_val = 0.0) const;

    /// Tái tạo ma trận 6x6: M_full = P^T * M_r * P
    [[nodiscard]] Matrix6d expand_matrix(const MatrixNd& M_r) const;

    /// Tên gọi dễ đọc của các DOF đang hoạt động
    [[nodiscard]] std::vector<std::string> active_dof_names() const;

    /// Các hàm khởi tạo tĩnh phụ trợ (factory helpers)
    static DofConfig make_6dof();
    static DofConfig make_rov_4dof();
    static DofConfig make_rov_3dof();
    static DofConfig make_planar_3dof();
    static DofConfig make_rov_6dof_full();
    static DofConfig make_rov_4dof_config_1();
    static DofConfig make_rov_3dof_config_1();

private:
    void rebuild_projection_matrix();

    std::vector<DofIndex> active_dofs_;
    MatrixNd P_; ///< Ma trận chiếu (n x 6)
};

} // namespace nav_dynamics
