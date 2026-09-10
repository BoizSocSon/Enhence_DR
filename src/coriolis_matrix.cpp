#include "nav_dynamics/coriolis_matrix.hpp"

namespace nav_dynamics {

CoriolisMatrixEvaluator::CoriolisMatrixEvaluator(const VehicleParameters& params)
    : params_(params) {}

void CoriolisMatrixEvaluator::set_parameters(const VehicleParameters& params) {
    params_ = params;
}

Matrix6d CoriolisMatrixEvaluator::calculate_C_RB(double mass, const Vector3d& r_G,
                                                 const Matrix3d& I_b, const Vector6d& nu) {
    Vector3d nu1 = nu.head<3>(); // [u, v, w]^T
    Vector3d nu2 = nu.tail<3>(); // [p, q, r]^T

    // a = nu1 + nu2 x r_G
    Vector3d a = nu1 + nu2.cross(r_G);

    // Skew matrices
    Matrix3d s_a = skew(a);
    Matrix3d s_Ib_nu2 = skew(I_b * nu2);

    Matrix6d C_rb = Matrix6d::Zero();
    // Top-left: 0_3x3
    // Top-right: -m * [a]_\times
    C_rb.block<3, 3>(0, 3) = -mass * s_a;
    // Bottom-left: -m * [a]_\times
    C_rb.block<3, 3>(3, 0) = -mass * s_a;
    // Bottom-right: -[I_b * nu2]_\times
    C_rb.block<3, 3>(3, 3) = -s_Ib_nu2;

    return C_rb;
}

Matrix6d CoriolisMatrixEvaluator::calculate_C_A(const Matrix6d& M_A, const Vector6d& nu_r) {
    Vector6d a_full = M_A * nu_r;
    Vector3d a1 = a_full.head<3>(); // a1 = M_A11 * nu_r1 + M_A12 * nu_r2
    Vector3d a2 = a_full.tail<3>(); // a2 = M_A21 * nu_r1 + M_A22 * nu_r2

    Matrix3d s_a1 = skew(a1);
    Matrix3d s_a2 = skew(a2);

    Matrix6d C_a = Matrix6d::Zero();
    // Top-left: 0_3x3
    // Top-right: -[a1]_\times
    C_a.block<3, 3>(0, 3) = -s_a1;
    // Bottom-left: -[a1]_\times
    C_a.block<3, 3>(3, 0) = -s_a1;
    // Bottom-right: -[a2]_\times
    C_a.block<3, 3>(3, 3) = -s_a2;

    return C_a;
}

Matrix6d CoriolisMatrixEvaluator::compute_C_RB(const Vector6d& nu) const {
    return calculate_C_RB(params_.mass, params_.r_G, params_.I_b, nu);
}

Matrix6d CoriolisMatrixEvaluator::compute_C_A(const Vector6d& nu_r) const {
    return calculate_C_A(params_.M_A, nu_r);
}

Matrix6d CoriolisMatrixEvaluator::compute_C(const Vector6d& nu, const Vector6d& nu_r) const {
    return compute_C_RB(nu) + compute_C_A(nu_r);
}

Vector6d CoriolisMatrixEvaluator::compute_coriolis_force(const Vector6d& nu, const Vector6d& nu_r) const {
    // tau_C = C_RB(nu)*nu + C_A(nu_r)*nu_r
    return compute_C_RB(nu) * nu + compute_C_A(nu_r) * nu_r;
}

MatrixNd CoriolisMatrixEvaluator::compute_reduced(const DofConfig& config,
                                                 const Vector6d& nu,
                                                 const Vector6d& nu_r) const {
    Matrix6d C = compute_C(nu, nu_r);
    return config.reduce_matrix(C);
}

VectorNd CoriolisMatrixEvaluator::compute_coriolis_force_reduced(const DofConfig& config,
                                                                const Vector6d& nu,
                                                                const Vector6d& nu_r) const {
    Vector6d f = compute_coriolis_force(nu, nu_r);
    return config.reduce_vector(f);
}

} // namespace nav_dynamics
