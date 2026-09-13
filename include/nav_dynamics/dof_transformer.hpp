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
  ROV_4DOF_CONFIG_1,    ///< 4-DOF {u, w, q, r} (Surge, Heave, Pitch, Yaw)
  ROV_3DOF_CONFIG_1,    ///< 3-DOF {u, w, r} (Surge, Heave, Yaw)
};

class DofConfig;

/**
 * @brief Mô-đun chuyên trách xây dựng ma trận biến đổi bậc tự do T ∈ R^{n×6},
 * và thực hiện các phép thu giảm/mở rộng cho phương trình động lực Fossen:
 *   M_r = T * M * T^T,  C_r = T * C * T^T,  D_r = T * D * T^T
 *   g_r = T * g,  tau_r = T * tau,  B_r = T * B
 * và phép mở rộng ngược lại không gian 6D:
 *   v_6d = T^T * v_r + (I - T^T * T) * v_constrained
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

  /// Ma trận biến đổi T ∈ R^{n×6}
  [[nodiscard]] const MatrixNd &T_matrix() const { return T_; }

  /// Ma trận chuyển vị T^T ∈ R^{6×n}
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
  // Phép thu giảm/mở rộng cốt lõi (Core reduction/expansion operations)
  // =========================================================================

  /// Thu giảm ma trận 6x6: M_r = T * M * T^T ∈ R^{n×n}
  [[nodiscard]] MatrixNd reduce_matrix(const Matrix6d &M) const;

  /// Thu giảm véc-tơ 6×1: v_r = T * v ∈ R^{n×1}
  [[nodiscard]] VectorNd reduce_vector(const Vector6d &v) const;

  /// Mở rộng véc-tơ thu giảm n×1 về véc-tơ 6D:
  /// v_6d = T^T * v_r + (I_6 - T^T*T) * constrained_vals
  [[nodiscard]] Vector6d
  expand_vector(const VectorNd &v_r,
                const Vector6d &constrained_vals = Vector6d::Zero()) const;

  /// Mở rộng ma trận thu giảm n×n về ma trận 6x6: M_6d = T^T * M_r * T
  [[nodiscard]] Matrix6d expand_matrix(const MatrixNd &M_r) const;

  /// Ma trận chiếu lên không gian con hoạt động: Projector = T^T * T ∈ R^{6×6}
  [[nodiscard]] Matrix6d active_subspace_projector() const;

  /// Chiếu véc-tơ 6D về không gian con hoạt động
  [[nodiscard]] Vector6d project_to_active(const Vector6d &v) const;

  // =========================================================================
  // Các bí danh ngữ nghĩa Fossen (Semantic aliases — inline delegates)
  // Dùng để tăng tính đọc hiểu khi làm việc với các thành phần cụ thể.
  // =========================================================================

  /// Thu giảm ma trận khối lượng: M_r = T * M * T^T
  [[nodiscard]] MatrixNd transform_mass(const Matrix6d &M) const { return reduce_matrix(M); }
  /// Thu giảm ma trận Coriolis: C_r = T * C * T^T
  [[nodiscard]] MatrixNd transform_coriolis(const Matrix6d &C) const { return reduce_matrix(C); }
  /// Thu giảm ma trận cản: D_r = T * D * T^T
  [[nodiscard]] MatrixNd transform_damping(const Matrix6d &D) const { return reduce_matrix(D); }
  /// Thu giảm ma trận Jacobian: J_r = T * J * T^T
  [[nodiscard]] MatrixNd transform_jacobian(const Matrix6d &J) const { return reduce_matrix(J); }

  /// Thu giảm véc-tơ tổng quát: v_r = T * v
  [[nodiscard]] VectorNd transform_vector(const Vector6d &v) const { return reduce_vector(v); }
  /// Thu giảm véc-tơ lực hồi phục: g_r = T * g
  [[nodiscard]] VectorNd transform_restoring(const Vector6d &g) const { return reduce_vector(g); }
  /// Thu giảm véc-tơ lực điều khiển: tau_r = T * tau
  [[nodiscard]] VectorNd transform_wrench(const Vector6d &tau) const { return reduce_vector(tau); }

  /// Thu giảm ma trận phân bổ lực đẩy 6×k: B_r = T * B ∈ R^{n×k}
  [[nodiscard]] MatrixNd transform_thruster_allocation(const MatrixNd &B) const;

  /// Tính ma trận Jacobian động học thu giảm từ trạng thái
  [[nodiscard]] MatrixNd compute_reduced_jacobian(const KinematicState &state) const;

  // =========================================================================
  // Các hàm khởi tạo tĩnh (Static Factory Helpers)
  // =========================================================================
  static DofTransformer make_6dof();
  static DofTransformer make_rov_3dof();
  static DofTransformer make_rov_4dof();
  static DofTransformer make_planar_3dof();
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
  MatrixNd T_; ///< Ma trận biến đổi (n × 6)
};

} // namespace nav_dynamics
