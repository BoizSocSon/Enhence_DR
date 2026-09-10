#include "nav_dynamics/damping_matrix.hpp"
#include <cmath>

namespace nav_dynamics {

DampingMatrixEvaluator::DampingMatrixEvaluator(const VehicleParameters& params)
    : params_(params) {}

void DampingMatrixEvaluator::set_parameters(const VehicleParameters& params) {
    params_ = params;
}

Matrix6d DampingMatrixEvaluator::compute_D_quadratic(const Vector6d& nu_r) const {
    Matrix6d D_q = Matrix6d::Zero();
    // Diagonals: D_q(i, i) = params_.D_q(i, i) * |nu_r(i)|
    for (int i = 0; i < 6; ++i) {
        D_q(i, i) = params_.D_q(i, i) * std::abs(nu_r(i));
    }
    return D_q;
}

Matrix6d DampingMatrixEvaluator::compute_D(const Vector6d& nu_r) const {
    return params_.D_l + compute_D_quadratic(nu_r);
}

Vector6d DampingMatrixEvaluator::compute_damping_force(const Vector6d& nu_r) const {
    Matrix6d D = compute_D(nu_r);
    return D * nu_r;
}

MatrixNd DampingMatrixEvaluator::compute_reduced(const DofConfig& config, const Vector6d& nu_r) const {
    Matrix6d D = compute_D(nu_r);
    return config.reduce_matrix(D);
}

VectorNd DampingMatrixEvaluator::compute_damping_force_reduced(const DofConfig& config, const Vector6d& nu_r) const {
    Vector6d f = compute_damping_force(nu_r);
    return config.reduce_vector(f);
}

bool DampingMatrixEvaluator::is_dissipative(const Vector6d& nu_r) const {
    if (nu_r.isZero(1e-9)) {
        return true;
    }
    double dissipation = nu_r.dot(compute_damping_force(nu_r));
    return dissipation > 0.0;
}

} // namespace nav_dynamics
