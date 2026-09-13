#include "nav_dynamics/dynamic_model.hpp"
#include <Eigen/Cholesky>
#include <cassert>
#include <cmath>

namespace nav_dynamics {

// =============================================================================
// Constructors
// =============================================================================

DynamicModel::DynamicModel()
    : DynamicModel(VehicleParameters(), DofTransformer::make_6dof(),
                   ThrusterAllocation::make_project_rov_3thruster()) {}

DynamicModel::DynamicModel(const VehicleParameters& params,
                           const DofTransformer& transformer,
                           const ThrusterAllocation& thrusters)
    : params_(params),
      transformer_(transformer),
      mass_evaluator_(params),
      coriolis_evaluator_(params),
      damping_evaluator_(params),
      restoring_evaluator_(params),
      thruster_allocation_(thrusters) {}

DynamicModel::DynamicModel(const VehicleParameters& params,
                           const DofConfig& dof_config,
                           const ThrusterAllocation& thrusters)
    : DynamicModel(params, dof_config.transformer(), thrusters) {}

// =============================================================================
// Configuration setters
// =============================================================================

void DynamicModel::set_dof_transformer(const DofTransformer& transformer) {
    transformer_ = transformer;
}

void DynamicModel::set_dof_config(const DofConfig& dof_config) {
    transformer_ = dof_config.transformer();
}

void DynamicModel::set_parameters(const VehicleParameters& params) {
    params_ = params;
    mass_evaluator_.set_parameters(params);
    coriolis_evaluator_.set_parameters(params);
    damping_evaluator_.set_parameters(params);
    restoring_evaluator_.set_parameters(params);
}

// =============================================================================
// Động lực học thuận — 6D đầy đủ
// =============================================================================

Vector6d DynamicModel::compute_forward_dynamics_6d(const KinematicState& state,
                                                   const Vector6d& tau,
                                                   const FluidCurrent& current) const {
    const auto& nu = state.nu;
    const Vector6d nu_r = current.compute_relative_velocity(state);

    // Các lực trong hệ quy chiếu thân tàu
    const Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    const Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    const Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    const Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    // Lực tổng hợp = tau - C*nu - D*nu_r - g + M_A*nu_c_dot
    const Vector6d net_wrench = tau - f_coriolis - f_damping - f_restoring + f_current_acc;

    // Gia tốc nu_dot = M^{-1} * net_wrench
    return mass_evaluator_.solve(net_wrench);
}

// =============================================================================
// Động lực học thuận — không gian thu giảm n-DOF
// =============================================================================

VectorNd DynamicModel::compute_forward_dynamics_reduced(const KinematicState& state,
                                                       const VectorNd& tau_r,
                                                       const FluidCurrent& current) const {
    const auto& nu = state.nu;
    const Vector6d nu_r = current.compute_relative_velocity(state);

    // Tính toán đầy đủ các lực rồi thu giảm bằng ma trận T
    const Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    const Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    const Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    const Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    const VectorNd f_coriolis_r = transformer_.reduce_vector(f_coriolis);
    const VectorNd f_damping_r = transformer_.reduce_vector(f_damping);
    const VectorNd f_restoring_r = transformer_.reduce_vector(f_restoring);
    const VectorNd f_current_acc_r = transformer_.reduce_vector(f_current_acc);

    const VectorNd net_wrench_r = tau_r - f_coriolis_r - f_damping_r - f_restoring_r + f_current_acc_r;

    // Ma trận khối lượng thu giảm: M_r = T * M * T^T
    const MatrixNd M_r = transformer_.reduce_matrix(mass_evaluator_.M_total());

    // Giải hệ M_r * nu_dot_r = net_wrench_r (ưu tiên Cholesky, dự phòng LDLT)
    Eigen::LLT<MatrixNd> llt(M_r);
    if (llt.info() == Eigen::Success) {
        return llt.solve(net_wrench_r);
    }
    return M_r.ldlt().solve(net_wrench_r);
}

// =============================================================================
// Động lực học thuận — router theo cấu hình DOF
// =============================================================================

Vector6d DynamicModel::compute_forward_dynamics(const KinematicState& state,
                                                const Vector6d& tau,
                                                const FluidCurrent& current) const {
    if (transformer_.reduced_dim() == 6) {
        return compute_forward_dynamics_6d(state, tau, current);
    }
    // Thu giảm → giải → mở rộng
    const VectorNd tau_r = transformer_.reduce_vector(tau);
    const VectorNd nu_dot_r = compute_forward_dynamics_reduced(state, tau_r, current);
    return transformer_.expand_vector(nu_dot_r, Vector6d::Zero());
}

// =============================================================================
// Phân tích chi tiết lực (Dynamic Breakdown)
// =============================================================================

DynamicBreakdown DynamicModel::evaluate_breakdown(const KinematicState& state,
                                                  const Vector6d& tau,
                                                  const FluidCurrent& current) const {
    DynamicBreakdown bd;
    bd.control_wrench = tau;
    const auto& nu = state.nu;
    const Vector6d nu_r = current.compute_relative_velocity(state);

    bd.coriolis_force = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    bd.damping_force = damping_evaluator_.compute_damping_force(nu_r);
    bd.restoring_force = restoring_evaluator_.compute_g_state(state);
    const Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);
    bd.net_wrench = bd.control_wrench - bd.coriolis_force - bd.damping_force - bd.restoring_force + f_current_acc;

    if (transformer_.reduced_dim() == 6) {
        bd.acceleration_6d = mass_evaluator_.solve(bd.net_wrench);
    } else {
        const VectorNd net_r = transformer_.reduce_vector(bd.net_wrench);
        const MatrixNd M_r = transformer_.reduce_matrix(mass_evaluator_.M_total());
        const VectorNd acc_r = M_r.ldlt().solve(net_r);
        bd.acceleration_6d = transformer_.expand_vector(acc_r, Vector6d::Zero());
    }
    return bd;
}

// =============================================================================
// Động lực học nghịch
// =============================================================================

Vector6d DynamicModel::compute_inverse_dynamics(const KinematicState& state,
                                                const Vector6d& nu_dot,
                                                const FluidCurrent& current) const {
    const auto& nu = state.nu;
    const Vector6d nu_r = current.compute_relative_velocity(state);

    const Vector6d f_inertial = mass_evaluator_.M_total() * nu_dot;
    const Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    const Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    const Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    const Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    return f_inertial + f_coriolis + f_damping + f_restoring - f_current_acc;
}

VectorNd DynamicModel::compute_inverse_dynamics_reduced(const KinematicState& state,
                                                       const VectorNd& nu_dot_r,
                                                       const FluidCurrent& current) const {
    const auto& nu = state.nu;
    const Vector6d nu_r = current.compute_relative_velocity(state);

    const MatrixNd M_r = transformer_.reduce_matrix(mass_evaluator_.M_total());
    const VectorNd f_inertial_r = M_r * nu_dot_r;

    const Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    const Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    const Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    const Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    const VectorNd f_coriolis_r = transformer_.reduce_vector(f_coriolis);
    const VectorNd f_damping_r = transformer_.reduce_vector(f_damping);
    const VectorNd f_restoring_r = transformer_.reduce_vector(f_restoring);
    const VectorNd f_current_acc_r = transformer_.reduce_vector(f_current_acc);

    return f_inertial_r + f_coriolis_r + f_damping_r + f_restoring_r - f_current_acc_r;
}

// =============================================================================
// Kinematic constraints
// =============================================================================

void DynamicModel::apply_kinematic_constraints(KinematicState& state) const {
    if (transformer_.reduced_dim() >= 6) {
        return;
    }

    // 1. Triệt tiêu vận tốc ở các trục không hoạt động
    state.nu = transformer_.project_to_active(state.nu);
    state.nu_dot = transformer_.project_to_active(state.nu_dot);

    // 2. Ép ràng buộc góc tư thế theo DOF không hoạt động
    const bool roll_active = transformer_.is_dof_active(DofIndex::ROLL);
    const bool pitch_active = transformer_.is_dof_active(DofIndex::PITCH);
    const bool yaw_active = transformer_.is_dof_active(DofIndex::YAW);

    if (!roll_active) {
        state.euler_rpy.x() = 0.0;
    }
    if (!pitch_active) {
        state.euler_rpy.y() = 0.0;
    }
    if (!yaw_active) {
        state.euler_rpy.z() = 0.0;
    }

    if (!roll_active || !pitch_active || !yaw_active) {
        state.update_quaternion_from_euler();
    }
}

// =============================================================================
// Propagation helper — cập nhật động học (vị trí + quaternion)
// =============================================================================

static void propagate_kinematics(KinematicState& state, double dt) {
    // 1. Cập nhật vị trí: dot_p_ned = R_nb * v_b
    const Vector3d v_body = state.nu.head<3>();
    state.pos_ned += state.R_nb() * v_body * dt;

    // 2. Cập nhật quaternion: dq = q * exp(0.5 * omega * dt)
    const Vector3d omega_body = state.nu.tail<3>();
    const double angle = omega_body.norm() * dt;
    if (angle > 1e-12) {
        const Vector3d axis = omega_body.normalized();
        Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
        state.orientation = state.orientation * delta_q;
        state.orientation.normalize();
    }
    state.update_euler_from_quaternion();
}

// =============================================================================
// Tích phân Euler
// =============================================================================

void DynamicModel::step_euler(KinematicState& state,
                              const Vector6d& tau,
                              double dt,
                              const FluidCurrent& current) const {
    apply_kinematic_constraints(state);
    const Vector6d nu_dot = compute_forward_dynamics(state, tau, current);
    state.nu_dot = nu_dot;
    state.nu += nu_dot * dt;
    propagate_kinematics(state, dt);
    apply_kinematic_constraints(state);
}

// =============================================================================
// Tích phân Runge-Kutta bậc 4 (RK4) — đã sửa quaternion propagation
// =============================================================================

/// Helper: tạo trạng thái trung gian RK4 với quaternion tính từ base_state
static KinematicState make_rk4_intermediate(const KinematicState& base_state,
                                            const Vector6d& k_nu,
                                            const Vector3d& k_pos,
                                            const Vector3d& k_omega,
                                            double h) {
    KinematicState s = base_state;
    s.nu += h * k_nu;
    s.pos_ned += h * k_pos;

    // Quaternion tính từ base_state (đúng — RK4 "frozen base" approach)
    const double angle = k_omega.norm() * h;
    if (angle > 1e-12) {
        s.orientation = base_state.orientation *
                        Eigen::Quaterniond(Eigen::AngleAxisd(angle, k_omega.normalized()));
        s.orientation.normalize();
    }
    s.update_euler_from_quaternion();
    return s;
}

void DynamicModel::step_rk4(KinematicState& state,
                             const Vector6d& tau,
                             double dt,
                             const FluidCurrent& current) const {
    apply_kinematic_constraints(state);

    // Stage 1 (k1)
    const Vector6d k1_nu = compute_forward_dynamics(state, tau, current);
    const Vector3d k1_pos = state.R_nb() * state.nu.head<3>();
    const Vector3d k1_omega = state.nu.tail<3>();

    // Stage 2 (k2) — trạng thái trung gian tại t + dt/2 theo k1
    KinematicState s2 = make_rk4_intermediate(state, k1_nu, k1_pos, k1_omega, 0.5 * dt);
    apply_kinematic_constraints(s2);

    const Vector6d k2_nu = compute_forward_dynamics(s2, tau, current);
    const Vector3d k2_pos = s2.R_nb() * s2.nu.head<3>();
    const Vector3d k2_omega = s2.nu.tail<3>();

    // Stage 3 (k3) — trạng thái trung gian tại t + dt/2 theo k2
    KinematicState s3 = make_rk4_intermediate(state, k2_nu, k2_pos, k2_omega, 0.5 * dt);
    apply_kinematic_constraints(s3);

    const Vector6d k3_nu = compute_forward_dynamics(s3, tau, current);
    const Vector3d k3_pos = s3.R_nb() * s3.nu.head<3>();
    const Vector3d k3_omega = s3.nu.tail<3>();

    // Stage 4 (k4) — trạng thái trung gian tại t + dt theo k3
    KinematicState s4 = make_rk4_intermediate(state, k3_nu, k3_pos, k3_omega, dt);
    apply_kinematic_constraints(s4);

    const Vector6d k4_nu = compute_forward_dynamics(s4, tau, current);
    const Vector3d k4_pos = s4.R_nb() * s4.nu.head<3>();
    const Vector3d k4_omega = s4.nu.tail<3>();

    // Tổ hợp trọng số RK4: (1/6)*(k1 + 2k2 + 2k3 + k4)
    const Vector6d nu_dot_avg = (k1_nu + 2.0 * k2_nu + 2.0 * k3_nu + k4_nu) / 6.0;
    const Vector3d pos_dot_avg = (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos) / 6.0;
    const Vector3d omega_avg = (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega) / 6.0;

    state.nu_dot = nu_dot_avg;
    state.nu += dt * nu_dot_avg;
    state.pos_ned += dt * pos_dot_avg;

    const double a_final = omega_avg.norm() * dt;
    if (a_final > 1e-12) {
        state.orientation = state.orientation *
                            Eigen::Quaterniond(Eigen::AngleAxisd(a_final, omega_avg.normalized()));
        state.orientation.normalize();
    }
    state.update_euler_from_quaternion();
    apply_kinematic_constraints(state);
}

} // namespace nav_dynamics
