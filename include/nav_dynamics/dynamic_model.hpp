#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/mass_matrix.hpp"
#include "nav_dynamics/coriolis_matrix.hpp"
#include "nav_dynamics/damping_matrix.hpp"
#include "nav_dynamics/restoring_force.hpp"
#include "nav_dynamics/thruster_allocation.hpp"

namespace nav_dynamics {

struct RovConfig;

/**
 * @brief Chi tiết phân tách toàn bộ các thành phần lực và mô-men trong phương trình chuyển động
 */
struct DynamicBreakdown {
    Vector6d control_wrench = Vector6d::Zero();   ///< Lực và mô-men điều khiển tau
    Vector6d coriolis_force = Vector6d::Zero();    ///< Lực Coriolis - hướng tâm C(nu)*nu
    Vector6d damping_force = Vector6d::Zero();     ///< Lực cản thủy động học D(nu_r)*nu_r
    Vector6d restoring_force = Vector6d::Zero();   ///< Lực và mô-men hồi phục thủy tĩnh g(eta)
    Vector6d net_wrench = Vector6d::Zero();        ///< Lực tổng hợp tác dụng = tau - C - D - g
    Vector6d acceleration_6d = Vector6d::Zero();   ///< Gia tốc nu_dot trong hệ thân tàu
};

/**
 * @brief Động cơ tính toán Mô hình Động lực học chính tích hợp toàn bộ các thành phần ma trận
 * (M, C, D, g, B) với hỗ trợ đầy đủ 6-DOF và khả năng cấu hình thu giảm n-DOF qua DofTransformer.
 */
class DynamicModel {
public:
    DynamicModel();
    explicit DynamicModel(const VehicleParameters& params,
                          const DofTransformer& transformer = DofTransformer::make_6dof(),
                          const ThrusterAllocation& thrusters = ThrusterAllocation::make_project_rov_3thruster());

    // Hàm khởi tạo tương thích ngược với DofConfig
    explicit DynamicModel(const VehicleParameters& params,
                          const DofConfig& dof_config,
                          const ThrusterAllocation& thrusters = ThrusterAllocation::make_project_rov_3thruster());

    /// Tái cấu hình các bậc tự do hoạt động
    void set_dof_transformer(const DofTransformer& transformer);

    /// Lấy bộ biến đổi DOF hiện tại
    [[nodiscard]] const DofTransformer& dof_transformer() const { return transformer_; }

    // Tương thích ngược: set/get DofConfig (delegate sang DofTransformer)
    void set_dof_config(const DofConfig& dof_config);
    [[nodiscard]] DofConfig dof_config() const { return DofConfig(transformer_); }

    /// Cập nhật các thông số vật lý của phương tiện
    void set_parameters(const VehicleParameters& params);

    /// Lấy các thông số của phương tiện
    [[nodiscard]] const VehicleParameters& parameters() const { return params_; }

    /// Truy cập các bộ tính toán ma trận thành phần
    [[nodiscard]] const MassMatrixEvaluator& mass_evaluator() const { return mass_evaluator_; }
    [[nodiscard]] const CoriolisMatrixEvaluator& coriolis_evaluator() const { return coriolis_evaluator_; }
    [[nodiscard]] const DampingMatrixEvaluator& damping_evaluator() const { return damping_evaluator_; }
    [[nodiscard]] const RestoringForceEvaluator& restoring_evaluator() const { return restoring_evaluator_; }
    [[nodiscard]] const ThrusterAllocation& thruster_allocation() const { return thruster_allocation_; }

    /// Động lực học thuận (6D đầy đủ):
    /// nu_dot = M^{-1} * (tau - C(nu)*nu - D(nu_r)*nu_r - g(eta))
    [[nodiscard]] Vector6d compute_forward_dynamics_6d(const KinematicState& state,
                                                       const Vector6d& tau,
                                                       const FluidCurrent& current = FluidCurrent()) const;

    /// Động lực học thuận theo cấu hình DOF hiện tại:
    /// Giải trên không gian n-DOF nếu cấu hình < 6, rồi mở rộng về 6D
    [[nodiscard]] Vector6d compute_forward_dynamics(const KinematicState& state,
                                                    const Vector6d& tau,
                                                    const FluidCurrent& current = FluidCurrent()) const;

    /// Tính động lực học thuận trực tiếp trong không gian thu giảm n-DOF
    [[nodiscard]] VectorNd compute_forward_dynamics_reduced(const KinematicState& state,
                                                            const VectorNd& tau_r,
                                                            const FluidCurrent& current = FluidCurrent()) const;

    /// Phân tích chi tiết tất cả các thành phần lực
    [[nodiscard]] DynamicBreakdown evaluate_breakdown(const KinematicState& state,
                                                     const Vector6d& tau,
                                                     const FluidCurrent& current = FluidCurrent()) const;

    /// Lấy số chiều không gian thu giảm
    [[nodiscard]] size_t reduced_dim() const { return transformer_.reduced_dim(); }

    /// Động lực học nghịch (6D đầy đủ):
    /// tau = M*nu_dot + C(nu)*nu + D(nu_r)*nu_r + g(eta)
    [[nodiscard]] Vector6d compute_inverse_dynamics(const KinematicState& state,
                                                    const Vector6d& nu_dot,
                                                    const FluidCurrent& current = FluidCurrent()) const;

    /// Động lực học nghịch trong không gian thu giảm n-DOF
    [[nodiscard]] VectorNd compute_inverse_dynamics_reduced(const KinematicState& state,
                                                           const VectorNd& nu_dot_r,
                                                           const FluidCurrent& current = FluidCurrent()) const;

    /// Tích phân số: 1 bước Euler
    void step_euler(KinematicState& state,
                    const Vector6d& tau,
                    double dt,
                    const FluidCurrent& current = FluidCurrent()) const;

    /// Tích phân số: 1 bước Runge-Kutta bậc 4 (RK4)
    void step_rk4(KinematicState& state,
                  const Vector6d& tau,
                  double dt,
                  const FluidCurrent& current = FluidCurrent()) const;

    /// Áp đặt các ràng buộc hình học theo cấu hình DOF
    void apply_kinematic_constraints(KinematicState& state) const;

private:
    VehicleParameters params_;
    DofTransformer transformer_;
    MassMatrixEvaluator mass_evaluator_;
    CoriolisMatrixEvaluator coriolis_evaluator_;
    DampingMatrixEvaluator damping_evaluator_;
    RestoringForceEvaluator restoring_evaluator_;
    ThrusterAllocation thruster_allocation_;
};

} // namespace nav_dynamics
