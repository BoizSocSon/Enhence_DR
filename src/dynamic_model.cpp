#include "nav_dynamics/dynamic_model.hpp"
#include <Eigen/Cholesky>
#include <cmath>

namespace nav_dynamics {

DynamicModel::DynamicModel()
    : DynamicModel(VehicleParameters(), DofTransformer::make_6dof(), ThrusterAllocation::make_project_rov_3thruster()) {}

DynamicModel::DynamicModel(const VehicleParameters& params,
                           const DofTransformer& transformer,
                           const ThrusterAllocation& thrusters)
    : params_(params),
      transformer_(transformer),
      dof_config_(transformer.active_dofs()),
      mass_evaluator_(params),
      coriolis_evaluator_(params),
      damping_evaluator_(params),
      restoring_evaluator_(params),
      thruster_allocation_(thrusters) {}

DynamicModel::DynamicModel(const VehicleParameters& params,
                           const DofConfig& dof_config,
                           const ThrusterAllocation& thrusters)
    : params_(params),
      transformer_(dof_config.active_dofs()),
      dof_config_(dof_config),
      mass_evaluator_(params),
      coriolis_evaluator_(params),
      damping_evaluator_(params),
      restoring_evaluator_(params),
      thruster_allocation_(thrusters) {}

void DynamicModel::set_dof_transformer(const DofTransformer& transformer) {
    transformer_ = transformer;
    dof_config_ = DofConfig(transformer.active_dofs());
}

void DynamicModel::set_dof_config(const DofConfig& dof_config) {
    dof_config_ = dof_config;
    transformer_ = DofTransformer(dof_config.active_dofs());
}

void DynamicModel::set_parameters(const VehicleParameters& params) {
    params_ = params;
    mass_evaluator_.set_parameters(params);
    coriolis_evaluator_.set_parameters(params);
    damping_evaluator_.set_parameters(params);
    restoring_evaluator_.set_parameters(params);
}

Vector6d DynamicModel::compute_forward_dynamics_6d(const KinematicState& state,
                                                  const Vector6d& tau,
                                                  const FluidCurrent& current) const {
    Vector6d nu = state.nu;
    Vector6d nu_r = current.compute_relative_velocity(state);

    // Các lực trong hệ quy chiếu thân tàu
    Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    // Lực tổng hợp tác dụng = tau - C*nu - D*nu_r - g + M_A*nu_c_dot
    Vector6d net_wrench = tau - f_coriolis - f_damping - f_restoring + f_current_acc;

    // Gia tốc nu_dot = M^{-1} * net_wrench
    return mass_evaluator_.solve(net_wrench);
}

VectorNd DynamicModel::compute_forward_dynamics_reduced(const KinematicState& state,
                                                       const VectorNd& tau_r,
                                                       const FluidCurrent& current) const {
    Vector6d nu = state.nu;
    Vector6d nu_r = current.compute_relative_velocity(state);

    // Tính toán đầy đủ các lực rồi biến đổi bằng ma trận T
    Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    VectorNd f_coriolis_r = transformer_.transform_vector(f_coriolis);
    VectorNd f_damping_r = transformer_.transform_vector(f_damping);
    VectorNd f_restoring_r = transformer_.transform_vector(f_restoring);
    VectorNd f_current_acc_r = transformer_.transform_vector(f_current_acc);

    VectorNd net_wrench_r = tau_r - f_coriolis_r - f_damping_r - f_restoring_r + f_current_acc_r;

    // Ma trận khối lượng thu giảm: M_r = T * M * T^T
    MatrixNd M_r = transformer_.transform_mass(mass_evaluator_.M_total());

    // Giải hệ phương trình M_r * nu_dot_r = net_wrench_r bằng phân rã Cholesky
    Eigen::LLT<MatrixNd> llt(M_r);
    if (llt.info() == Eigen::Success) {
        return llt.solve(net_wrench_r);
    }
    return M_r.ldlt().solve(net_wrench_r);
}

Vector6d DynamicModel::compute_forward_dynamics(const KinematicState& state,
                                               const Vector6d& tau,
                                               const FluidCurrent& current) const {
    if (transformer_.reduced_dim() == 6) {
        return compute_forward_dynamics_6d(state, tau, current);
    }
    // Tính toán trong không gian thu giảm thông qua DofTransformer
    VectorNd tau_r = transformer_.transform_wrench(tau);
    VectorNd nu_dot_r = compute_forward_dynamics_reduced(state, tau_r, current);
    return transformer_.expand_vector(nu_dot_r, Vector6d::Zero());
}

DynamicBreakdown DynamicModel::evaluate_breakdown(const KinematicState& state,
                                                 const Vector6d& tau,
                                                 const FluidCurrent& current) const {
    DynamicBreakdown bd;
    bd.control_wrench = tau;
    Vector6d nu = state.nu;
    Vector6d nu_r = current.compute_relative_velocity(state);

    bd.coriolis_force = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    bd.damping_force = damping_evaluator_.compute_damping_force(nu_r);
    bd.restoring_force = restoring_evaluator_.compute_g_state(state);
    Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);
    bd.net_wrench = bd.control_wrench - bd.coriolis_force - bd.damping_force - bd.restoring_force + f_current_acc;

    if (transformer_.reduced_dim() == 6) {
        bd.acceleration_6d = mass_evaluator_.solve(bd.net_wrench);
    } else {
        VectorNd net_r = transformer_.transform_wrench(bd.net_wrench);
        MatrixNd M_r = transformer_.transform_mass(mass_evaluator_.M_total());
        VectorNd acc_r = M_r.ldlt().solve(net_r);
        bd.acceleration_6d = transformer_.expand_vector(acc_r, Vector6d::Zero());
    }
    return bd;
}

Vector6d DynamicModel::compute_inverse_dynamics(const KinematicState& state,
                                               const Vector6d& nu_dot,
                                               const FluidCurrent& current) const {
    Vector6d nu = state.nu;
    Vector6d nu_r = current.compute_relative_velocity(state);

    Vector6d f_inertial = mass_evaluator_.M_total() * nu_dot;
    Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    return f_inertial + f_coriolis + f_damping + f_restoring - f_current_acc;
}

VectorNd DynamicModel::compute_inverse_dynamics_reduced(const KinematicState& state,
                                                       const VectorNd& nu_dot_r,
                                                       const FluidCurrent& current) const {
    Vector6d nu = state.nu;
    Vector6d nu_r = current.compute_relative_velocity(state);

    MatrixNd M_r = transformer_.transform_mass(mass_evaluator_.M_total());
    VectorNd f_inertial_r = M_r * nu_dot_r;

    Vector6d f_coriolis = coriolis_evaluator_.compute_coriolis_force(nu, nu_r);
    Vector6d f_damping = damping_evaluator_.compute_damping_force(nu_r);
    Vector6d f_restoring = restoring_evaluator_.compute_g_state(state);
    Vector6d f_current_acc = params_.M_A * current.compute_current_acceleration(state);

    VectorNd f_coriolis_r = transformer_.transform_vector(f_coriolis);
    VectorNd f_damping_r = transformer_.transform_vector(f_damping);
    VectorNd f_restoring_r = transformer_.transform_vector(f_restoring);
    VectorNd f_current_acc_r = transformer_.transform_vector(f_current_acc);

    return f_inertial_r + f_coriolis_r + f_damping_r + f_restoring_r - f_current_acc_r;
}

static void propagate_kinematics(KinematicState& state, double dt) {
    // 1. Cập nhật vị trí: dot_p_ned = R_nb * v_b
    Vector3d v_body = state.nu.head<3>();
    state.pos_ned += state.R_nb() * v_body * dt;

    // 2. Cập nhật quaternion định hướng: dq = q * exp(0.5 * omega * dt)
    Vector3d omega_body = state.nu.tail<3>();
    double angle = omega_body.norm() * dt;
    if (angle > 1e-12) {
        Vector3d axis = omega_body.normalized();
        Eigen::Quaterniond delta_q(Eigen::AngleAxisd(angle, axis));
        state.orientation = state.orientation * delta_q;
        state.orientation.normalize();
    }
    state.update_euler_from_quaternion();
}

void DynamicModel::apply_kinematic_constraints(KinematicState& state) const {
    if (transformer_.reduced_dim() >= 6) {
        return;
    }

    // 1. Chiếu triệt tiêu vận tốc ở các trục không hoạt động
    state.nu = transformer_.project_to_active(state.nu);
    state.nu_dot = transformer_.project_to_active(state.nu_dot);

    // 2. Ép ràng buộc góc tư thế theo các DOF không hoạt động
    bool roll_active = transformer_.is_dof_active(DofIndex::ROLL);
    bool pitch_active = transformer_.is_dof_active(DofIndex::PITCH);
    bool yaw_active = transformer_.is_dof_active(DofIndex::YAW);

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

void DynamicModel::step_euler(KinematicState& state,
                             const Vector6d& tau,
                             double dt,
                             const FluidCurrent& current) const {
    apply_kinematic_constraints(state);
    Vector6d nu_dot = compute_forward_dynamics(state, tau, current);
    state.nu_dot = nu_dot;
    state.nu += nu_dot * dt;
    propagate_kinematics(state, dt);
    apply_kinematic_constraints(state);
}

void DynamicModel::step_rk4(KinematicState& state,
                           const Vector6d& tau,
                           double dt,
                           const FluidCurrent& current) const {
    apply_kinematic_constraints(state);

    // Stage 1 (k1)
    Vector6d k1_nu = compute_forward_dynamics(state, tau, current);
    Vector3d k1_pos = state.R_nb() * state.nu.head<3>();
    Vector3d k1_omega = state.nu.tail<3>();

    // Stage 2 (k2)
    KinematicState s2 = state;
    s2.nu += (0.5 * dt) * k1_nu;
    s2.pos_ned += (0.5 * dt) * k1_pos;
    double a1 = k1_omega.norm() * (0.5 * dt);
    if (a1 > 1e-12) {
        s2.orientation = state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(a1, k1_omega.normalized()));
        s2.orientation.normalize();
    }
    s2.update_euler_from_quaternion();
    apply_kinematic_constraints(s2);

    Vector6d k2_nu = compute_forward_dynamics(s2, tau, current);
    Vector3d k2_pos = s2.R_nb() * s2.nu.head<3>();
    Vector3d k2_omega = s2.nu.tail<3>();

    // Stage 3 (k3)
    KinematicState s3 = state;
    s3.nu += (0.5 * dt) * k2_nu;
    s3.pos_ned += (0.5 * dt) * k2_pos;
    double a2 = k2_omega.norm() * (0.5 * dt);
    if (a2 > 1e-12) {
        s3.orientation = state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(a2, k2_omega.normalized()));
        s3.orientation.normalize();
    }
    s3.update_euler_from_quaternion();
    apply_kinematic_constraints(s3);

    Vector6d k3_nu = compute_forward_dynamics(s3, tau, current);
    Vector3d k3_pos = s3.R_nb() * s3.nu.head<3>();
    Vector3d k3_omega = s3.nu.tail<3>();

    // Stage 4 (k4)
    KinematicState s4 = state;
    s4.nu += dt * k3_nu;
    s4.pos_ned += dt * k3_pos;
    double a3 = k3_omega.norm() * dt;
    if (a3 > 1e-12) {
        s4.orientation = state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(a3, k3_omega.normalized()));
        s4.orientation.normalize();
    }
    s4.update_euler_from_quaternion();
    apply_kinematic_constraints(s4);

    Vector6d k4_nu = compute_forward_dynamics(s4, tau, current);
    Vector3d k4_pos = s4.R_nb() * s4.nu.head<3>();
    Vector3d k4_omega = s4.nu.tail<3>();

    // Tổ hợp trọng số trung bình bậc 4 (Simpson RK4: 1/6*(k1 + 2k2 + 2k3 + k4))
    Vector6d nu_dot_avg = (k1_nu + 2.0 * k2_nu + 2.0 * k3_nu + k4_nu) / 6.0;
    Vector3d pos_dot_avg = (k1_pos + 2.0 * k2_pos + 2.0 * k3_pos + k4_pos) / 6.0;
    Vector3d omega_avg = (k1_omega + 2.0 * k2_omega + 2.0 * k3_omega + k4_omega) / 6.0;

    state.nu_dot = nu_dot_avg;
    state.nu += dt * nu_dot_avg;
    state.pos_ned += dt * pos_dot_avg;

    double a_final = omega_avg.norm() * dt;
    if (a_final > 1e-12) {
        state.orientation = state.orientation * Eigen::Quaterniond(Eigen::AngleAxisd(a_final, omega_avg.normalized()));
        state.orientation.normalize();
    }
    state.update_euler_from_quaternion();
    apply_kinematic_constraints(state);
}

} // namespace nav_dynamics
