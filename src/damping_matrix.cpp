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
    // Hỗ trợ đầy đủ cả ma trận đường chéo và các hệ số cản ghép chéo (cross-coupling):
    // D_q(i, j) = params_.D_q(i, j) * |nu_r(j)| sao cho tau_D_q = D_q(nu_r) * nu_r
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            D_q(i, j) = params_.D_q(i, j) * std::abs(nu_r(j));
        }
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

MatrixNd DampingMatrixEvaluator::compute_reduced(const DofTransformer& transformer, const Vector6d& nu_r) const {
    Matrix6d D = compute_D(nu_r);
    return transformer.transform_damping(D);
}

VectorNd DampingMatrixEvaluator::compute_damping_force_reduced(const DofConfig& config, const Vector6d& nu_r) const {
    Vector6d f = compute_damping_force(nu_r);
    return config.reduce_vector(f);
}

VectorNd DampingMatrixEvaluator::compute_damping_force_reduced(const DofTransformer& transformer, const Vector6d& nu_r) const {
    Vector6d f = compute_damping_force(nu_r);
    return transformer.transform_vector(f);
}

bool DampingMatrixEvaluator::is_dissipative(const Vector6d& nu_r) const {
    if (nu_r.isZero(1e-9)) {
        return true;
    }
    double dissipation = nu_r.dot(compute_damping_force(nu_r));
    return dissipation > 0.0;
}

} // namespace nav_dynamics
