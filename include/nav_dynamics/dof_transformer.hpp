#pragma once

#include "nav_dynamics/types.hpp"
#include <string>
#include <vector>

namespace nav_dynamics {

/**
 * @brief Các cấu hình bậc tự do (DOF) định sẵn phổ biến
 */
enum class DofPreset {
  ROV_6DOF_FULL,        ///< 6-DOF {u, v, w, p, q, r} (Surge, Sway, Heave, Roll, Pitch, Yaw)
  ROV_4DOF_CONFIG_1,    // 4-DOF {u, w, q, r} (Surge, Heave, Pitch, Yaw)
  ROV_3DOF_CONFIG_1,    // 3-DOF {u, w, r} (Surge, Heave, Yaw)
};

class DofConfig;

/**
 * @brief Mô-đun chuyên trách xây dựng ma trận biến đổi bậc tự do T thuộc R^{n x
 * 6}, và thực hiện các phép nhân ma trận với các thành phần trong phương trình
 * động lực Fossen: M_r   = T * M * T^T C_r   = T * C * T^T D_r   = T * D * T^T
 *   g_r   = T * g
 *   tau_r = T * tau
 *   B_r   = T * B
 * và phép mở rộng ngược lại không gian 6D:
 *   v_6d  = T^T * v_r + (I - T^T * T) * v_constrained
 */
class DofTransformer {
public:
  DofTransformer();
  explicit DofTransformer(DofPreset preset);
  explicit DofTransformer(const std::vector<DofIndex> &active_dofs);
  explicit DofTransformer(const MatrixNd &custom_T,
                          const std::vector<DofIndex> &active_dofs = {});

  /// Chuyển đổi sang cấu hình DofConfig tương ứng
  [[nodiscard]] DofConfig to_dof_config() const;

  /// Số chiều của không gian trạng thái thu giảm n (1 <= n <= 6)
  [[nodiscard]] size_t reduced_dim() const { return reduced_dim_; }

  /// Số chiều đầy đủ 6-DOF (luôn bằng 6)
  [[nodiscard]] size_t full_dim() const { return 6; }

  /// Ma trận biến đổi T thuộc R^{n x 6}
  [[nodiscard]] const MatrixNd &T_matrix() const { return T_; }

  /// Ma trận chuyển vị / giả nghịch đảo mở rộng T^T thuộc R^{6 x n}
  [[nodiscard]] MatrixNd T_transpose() const { return T_.transpose(); }

  /// Các chỉ số DOF đang hoạt động (theo thứ tự hàng thu giảm 0..n-1)
  [[nodiscard]] const std::vector<DofIndex> &active_dofs() const {
    return active_dofs_;
  }

  /// Kiểm tra xem một bậc tự do cụ thể có đang hoạt động hay không
  [[nodiscard]] bool is_dof_active(DofIndex dof) const;

  /// Lấy chỉ số hàng (0..n-1) của một DOF hoạt động trong không gian thu giảm
  /// (-1 nếu không hoạt động)
  [[nodiscard]] int reduced_index_of(DofIndex dof) const;

  /// Tên gọi dễ đọc của các DOF đang hoạt động
  [[nodiscard]] std::vector<std::string> active_dof_names() const;

  /// Kiểm tra tính trực giao phải: T * T^T = I_n
  [[nodiscard]] bool is_orthogonal(double tol = 1e-9) const;

  // =========================================================================
  // Phép thu giảm ma trận phương trình Fossen (T * M * T^T, v.v.)
  // =========================================================================

  /// Thu giảm ma trận khối lượng 6x6: M_r = T * M * T^T thuộc R^{n x n}
  [[nodiscard]] MatrixNd transform_mass(const Matrix6d &M) const;

  /// Thu giảm ma trận Coriolis 6x6: C_r = T * C * T^T thuộc R^{n x n}
  [[nodiscard]] MatrixNd transform_coriolis(const Matrix6d &C) const;

  /// Thu giảm ma trận cản 6x6: D_r = T * D * T^T thuộc R^{n x n}
  [[nodiscard]] MatrixNd transform_damping(const Matrix6d &D) const;

  /// Thu giảm véc-tơ Lực/Mô-men/Vận tốc 6x1: v_r = T * v thuộc R^{n x 1}
  [[nodiscard]] VectorNd transform_vector(const Vector6d &v) const;

  /// Thu giảm véc-tơ lực hồi phục 6x1: g_r = T * g thuộc R^{n x 1}
  [[nodiscard]] VectorNd transform_restoring(const Vector6d &g) const;

  /// Thu giảm véc-tơ lực điều khiển tổng quát 6x1: tau_r = T * tau thuộc R^{n x
  /// 1}
  [[nodiscard]] VectorNd transform_wrench(const Vector6d &tau) const;

  /// Thu giảm ma trận phân bổ lực đẩy 6 x k: B_r = T * B thuộc R^{n x k}
  [[nodiscard]] MatrixNd transform_thruster_allocation(const MatrixNd &B) const;

  // =========================================================================
  // Các toán tử mở rộng ngược về không gian 6D
  // =========================================================================

  /// Mở rộng véc-tơ thu giảm n x 1 về véc-tơ 6D: v_6d = T^T * v_r + (I - T^T*T)
  /// * v_constrained
  [[nodiscard]] Vector6d
  expand_vector(const VectorNd &v_r,
                const Vector6d &constrained_vals = Vector6d::Zero()) const;

  /// Mở rộng ma trận thu giảm n x n về ma trận 6x6: M_6d = T^T * M_r * T
  [[nodiscard]] Matrix6d expand_matrix(const MatrixNd &M_r) const;

  // =========================================================================
  // Các hàm khởi tạo tĩnh phụ trợ (Static Factory Helpers)
  // =========================================================================
  static DofTransformer make_6dof();
  static DofTransformer make_rov_4dof();
  static DofTransformer make_rov_3dof();
  static DofTransformer make_planar_3dof();
  static DofTransformer make_rov_6dof_full();
  static DofTransformer make_rov_4dof_config_1();
  static DofTransformer make_rov_3dof_config_1();
  static DofTransformer from_preset(DofPreset preset);
  static DofTransformer
  from_dof_names(const std::vector<std::string> &dof_names);
  static DofTransformer
  from_matrix(const MatrixNd &custom_T,
              const std::vector<DofIndex> &active_dofs = {});

private:
  void build_T_from_active_dofs();

  std::vector<DofIndex> active_dofs_;
  size_t reduced_dim_ = 6;
  MatrixNd T_; ///< Ma trận biến đổi (n x 6)
};

} // namespace nav_dynamics
