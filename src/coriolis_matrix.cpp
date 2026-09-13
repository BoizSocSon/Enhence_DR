#include "nav_dynamics/coriolis_matrix.hpp"

namespace nav_dynamics {

CoriolisMatrixEvaluator::CoriolisMatrixEvaluator(const VehicleParameters& params)
    : params_(params) {}

void CoriolisMatrixEvaluator::set_parameters(const VehicleParameters& params) {
    params_ = params;
}

Matrix6d CoriolisMatrixEvaluator::calculate_C_RB(double mass, const Vector3d& r_G,
                                                 const Matrix3d& I_b, const Vector6d& nu) {
    const Vector3d nu1 = nu.head<3>(); // Vận tốc tịnh tiến [u, v, w]^T
    const Vector3d nu2 = nu.tail<3>(); // Vận tốc góc [p, q, r]^T

    // a = nu1 + nu2 × r_G
    const Vector3d a = nu1 + nu2.cross(r_G);

    const Matrix3d s_a = skew(a);
    const Matrix3d s_Ib_nu2 = skew(I_b * nu2);

    Matrix6d C_rb = Matrix6d::Zero();
    C_rb.block<3, 3>(0, 3) = -mass * s_a;
    C_rb.block<3, 3>(3, 0) = -mass * s_a;
    C_rb.block<3, 3>(3, 3) = -s_Ib_nu2;

    return C_rb;
}

Matrix6d CoriolisMatrixEvaluator::calculate_C_A(const Matrix6d& M_A, const Vector6d& nu_r) {
    const Vector6d a_full = M_A * nu_r;
    const Vector3d a1 = a_full.head<3>();
    const Vector3d a2 = a_full.tail<3>();

    const Matrix3d s_a1 = skew(a1);
    const Matrix3d s_a2 = skew(a2);

    Matrix6d C_a = Matrix6d::Zero();
    C_a.block<3, 3>(0, 3) = -s_a1;
    C_a.block<3, 3>(3, 0) = -s_a1;
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
    return compute_C_RB(nu) * nu + compute_C_A(nu_r) * nu_r;
}

MatrixNd CoriolisMatrixEvaluator::compute_reduced(const DofTransformer& transformer,
                                                 const Vector6d& nu,
                                                 const Vector6d& nu_r) const {
    return transformer.reduce_matrix(compute_C(nu, nu_r));
}

VectorNd CoriolisMatrixEvaluator::compute_coriolis_force_reduced(const DofTransformer& transformer,
                                                                const Vector6d& nu,
                                                                const Vector6d& nu_r) const {
    return transformer.reduce_vector(compute_coriolis_force(nu, nu_r));
}

} // namespace nav_dynamics
