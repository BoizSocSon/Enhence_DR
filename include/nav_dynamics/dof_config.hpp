#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include <vector>
#include <string>

namespace nav_dynamics {

/**
 * @brief Thin wrapper quanh DofTransformer, cung cấp API tương thích ngược
 * sử dụng thuật ngữ "projection matrix" (P) thay vì "transformation matrix" (T).
 * 
 * Nội bộ delegate toàn bộ sang DofTransformer. Không lưu trữ dữ liệu riêng.
 */
class DofConfig {
public:
    DofConfig();
    explicit DofConfig(DofPreset preset);
    explicit DofConfig(const std::vector<DofIndex>& active_dofs);
    explicit DofConfig(const MatrixNd& custom_P, const std::vector<DofIndex>& active_dofs = {});
    explicit DofConfig(const DofTransformer& transformer);

    /// Chuyển đổi sang DofTransformer
    [[nodiscard]] const DofTransformer& transformer() const { return transformer_; }
    [[nodiscard]] DofTransformer to_transformer() const { return transformer_; }

    /// Lấy danh sách các chỉ số DOF đang hoạt động hiện tại
    [[nodiscard]] const std::vector<DofIndex>& active_dofs() const { return transformer_.active_dofs(); }

    /// Số chiều của không gian trạng thái thu giảm (1 <= n <= 6)
    [[nodiscard]] size_t dim() const { return transformer_.reduced_dim(); }

    /// Kiểm tra xem một bậc tự do cụ thể có đang hoạt động hay không
    [[nodiscard]] bool is_active(DofIndex dof) const { return transformer_.is_dof_active(dof); }

    /// Lấy chỉ số (0..n-1) của một DOF đang hoạt động, trả về -1 nếu không hoạt động
    [[nodiscard]] int reduced_index_of(DofIndex dof) const { return transformer_.reduced_index_of(dof); }

    /// Lấy ma trận chiếu P ∈ R^{n×6} (≡ T_matrix() của DofTransformer)
    [[nodiscard]] const MatrixNd& projection_matrix() const { return transformer_.T_matrix(); }

    /// Thu giảm véc-tơ 6D: v_r = P * v
    [[nodiscard]] VectorNd reduce_vector(const Vector6d& v) const { return transformer_.reduce_vector(v); }

    /// Thu giảm ma trận 6x6: M_r = P * M * P^T
    [[nodiscard]] MatrixNd reduce_matrix(const Matrix6d& M) const { return transformer_.reduce_matrix(M); }

    /// Tái tạo véc-tơ 6D: các DOF không hoạt động gán bằng default_val
    [[nodiscard]] Vector6d expand_vector(const VectorNd& v_r, double default_val = 0.0) const;

    /// Tái tạo ma trận 6x6: M_full = P^T * M_r * P
    [[nodiscard]] Matrix6d expand_matrix(const MatrixNd& M_r) const { return transformer_.expand_matrix(M_r); }

    /// Tên gọi dễ đọc của các DOF đang hoạt động
    [[nodiscard]] std::vector<std::string> active_dof_names() const { return transformer_.active_dof_names(); }

    /// Các hàm khởi tạo tĩnh phụ trợ (factory helpers)
    static DofConfig make_6dof();
    static DofConfig make_rov_4dof();
    static DofConfig make_rov_3dof();
    static DofConfig make_planar_3dof();

private:
    DofTransformer transformer_;
};

} // namespace nav_dynamics
