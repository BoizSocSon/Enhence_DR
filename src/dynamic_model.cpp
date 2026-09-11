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

    // Lực tổng hợp tác dụng = tau - C*nu - D*nu_r - g
    Vector6d net_wrench = tau - f_coriolis - f_damping - f_restoring;

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

    VectorNd f_coriolis_r = transformer_.transform_vector(f_coriolis);
    VectorNd f_damping_r = transformer_.transform_vector(f_damping);
    VectorNd f_restoring_r = transformer_.transform_vector(f_restoring);

    VectorNd net_wrench_r = tau_r - f_coriolis_r - f_damping_r - f_restoring_r;

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
    bd.net_wrench = bd.control_wrench - bd.coriolis_force - bd.damping_force - bd.restoring_force;

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

    return f_inertial + f_coriolis + f_damping + f_restoring;
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

void DynamicModel::step_euler(KinematicState& state,
                             const Vector6d& tau,
                             double dt,
                             const FluidCurrent& current) const {
    Vector6d nu_dot = compute_forward_dynamics(state, tau, current);
    state.nu_dot = nu_dot;
    state.nu += nu_dot * dt;
    propagate_kinematics(state, dt);
}

void DynamicModel::step_rk4(KinematicState& state,
                           const Vector6d& tau,
                           double dt,
                           const FluidCurrent& current) const {
    // Tích phân RK4 để truyền lan vận tốc
    KinematicState s = state;

    // k1
    Vector6d k1_nu = compute_forward_dynamics(s, tau, current);

    // k2
    KinematicState s_half1 = state;
    s_half1.nu += 0.5 * dt * k1_nu;
    propagate_kinematics(s_half1, 0.5 * dt);
    Vector6d k2_nu = compute_forward_dynamics(s_half1, tau, current);

    // k3
    KinematicState s_half2 = state;
    s_half2.nu += 0.5 * dt * k2_nu;
    propagate_kinematics(s_half2, 0.5 * dt);
    Vector6d k3_nu = compute_forward_dynamics(s_half2, tau, current);

    // k4
    KinematicState s_full = state;
    s_full.nu += dt * k3_nu;
    propagate_kinematics(s_full, dt);
    Vector6d k4_nu = compute_forward_dynamics(s_full, tau, current);

    // Tổ hợp trọng số gia tốc cho vận tốc
    Vector6d nu_dot_avg = (k1_nu + 2.0 * k2_nu + 2.0 * k3_nu + k4_nu) / 6.0;
    state.nu_dot = nu_dot_avg;
    state.nu += nu_dot_avg * dt;

    // Truyền lan động học vị trí và tư thế
    propagate_kinematics(state, dt);
}

} // namespace nav_dynamics
