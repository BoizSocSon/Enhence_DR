#include "nav_dynamics/restoring_force.hpp"
#include <cmath>

namespace nav_dynamics {

RestoringForceEvaluator::RestoringForceEvaluator(const VehicleParameters& params)
    : params_(params) {}

void RestoringForceEvaluator::set_parameters(const VehicleParameters& params) {
    params_ = params;
}

Vector6d RestoringForceEvaluator::calculate_g(double mass, double volume, double fluid_density, double g_acc,
                                             const Vector3d& r_G, const Vector3d& r_B, const Matrix3d& R_nb) {
    double W = mass * g_acc;
    double B = fluid_density * g_acc * volume;

    // Gravity vector in NED: [0, 0, W]^T
    Vector3d f_g_ned(0.0, 0.0, W);
    // Buoyancy vector in NED: [0, 0, -B]^T
    Vector3d f_b_ned(0.0, 0.0, -B);

    // Transform to body frame: f_b = R_nb^T * f_n
    Matrix3d R_bn = R_nb.transpose();
    Vector3d f_g_body = R_bn * f_g_ned;
    Vector3d f_b_body = R_bn * f_b_ned;

    // Total hydrostatic force in body frame
    Vector3d f_hydro_body = f_g_body + f_b_body;

    // Total hydrostatic moment in body frame
    Vector3d m_hydro_body = r_G.cross(f_g_body) + r_B.cross(f_b_body);

    // In equations of motion: M*nu_dot + ... + g(eta) = tau
    // Therefore g(eta) = - [f_hydro_body; m_hydro_body]
    Vector6d g;
    g.head<3>() = -f_hydro_body;
    g.tail<3>() = -m_hydro_body;

    return g;
}

Vector6d RestoringForceEvaluator::compute_g(const Matrix3d& R_nb) const {
    return calculate_g(params_.mass, params_.volume, params_.fluid_density, params_.gravity,
                       params_.r_G, params_.r_B, R_nb);
}

Vector6d RestoringForceEvaluator::compute_g_euler(double phi, double theta, double psi) const {
    Eigen::Matrix3d R = (Eigen::AngleAxisd(psi, Vector3d::UnitZ())
                       * Eigen::AngleAxisd(theta, Vector3d::UnitY())
                       * Eigen::AngleAxisd(phi, Vector3d::UnitX())).toRotationMatrix();
    return compute_g(R);
}

Vector6d RestoringForceEvaluator::compute_g_state(const KinematicState& state) const {
    return compute_g(state.R_nb());
}

VectorNd RestoringForceEvaluator::compute_reduced(const DofConfig& config, const KinematicState& state) const {
    Vector6d g_full = compute_g_state(state);
    return config.reduce_vector(g_full);
}

double RestoringForceEvaluator::net_submerged_weight() const {
    return params_.weight() - params_.buoyancy();
}

} // namespace nav_dynamics
