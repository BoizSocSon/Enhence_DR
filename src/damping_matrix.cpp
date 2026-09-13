#include "nav_dynamics/damping_matrix.hpp"
#include <cmath>

namespace nav_dynamics {

DampingMatrixEvaluator::DampingMatrixEvaluator(const VehicleParameters& params)
    : params_(params) {}

void DampingMatrixEvaluator::set_parameters(const VehicleParameters& params) {
    params_ = params;
}

Matrix6d DampingMatrixEvaluator::compute_D_quadratic(const Vector6d& nu_r) const {
    // Vectorized Eigen: D_q(i,j) = params_.D_q(i,j) * |nu_r(j)|
    // Tương đương nhân mỗi cột j của D_q với |nu_r(j)|
    const Eigen::Array<double, 1, 6> abs_nu_r = nu_r.array().abs().transpose();
    return (params_.D_q.array().rowwise() * abs_nu_r).matrix();
}

Matrix6d DampingMatrixEvaluator::compute_D(const Vector6d& nu_r) const {
    return params_.D_l + compute_D_quadratic(nu_r);
}

Vector6d DampingMatrixEvaluator::compute_damping_force(const Vector6d& nu_r) const {
    return compute_D(nu_r) * nu_r;
}

MatrixNd DampingMatrixEvaluator::compute_reduced(const DofTransformer& transformer, const Vector6d& nu_r) const {
    return transformer.reduce_matrix(compute_D(nu_r));
}

VectorNd DampingMatrixEvaluator::compute_damping_force_reduced(const DofTransformer& transformer, const Vector6d& nu_r) const {
    return transformer.reduce_vector(compute_damping_force(nu_r));
}

bool DampingMatrixEvaluator::is_dissipative(const Vector6d& nu_r) const {
    if (nu_r.isZero(1e-9)) {
        return true;
    }
    return nu_r.dot(compute_damping_force(nu_r)) > 0.0;
}

} // namespace nav_dynamics
