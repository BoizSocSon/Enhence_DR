#include "nav_dynamics/mass_matrix.hpp"
#include <Eigen/Cholesky>
#include <cassert>

namespace nav_dynamics {

MassMatrixEvaluator::MassMatrixEvaluator(const VehicleParameters& params)
    : params_(params) {
    update_matrices();
}

void MassMatrixEvaluator::set_parameters(const VehicleParameters& params) {
    params_ = params;
    update_matrices();
}

Matrix6d MassMatrixEvaluator::compute_M_RB(double mass, const Vector3d& r_G, const Matrix3d& I_b) {
    Matrix6d M_rb = Matrix6d::Zero();
    // Khối trên-trái: m * I_3
    M_rb.block<3, 3>(0, 0) = mass * Matrix3d::Identity();

    // Khối trên-phải: -m * [r_G]_×
    Matrix3d s_rg = skew(r_G);
    M_rb.block<3, 3>(0, 3) = -mass * s_rg;

    // Khối dưới-trái: m * [r_G]_× (= chuyển vị của khối trên-phải vì S(v)^T = -S(v))
    M_rb.block<3, 3>(3, 0) = mass * s_rg;

    // Khối dưới-phải: I_b
    M_rb.block<3, 3>(3, 3) = I_b;

    // Xác minh tính đối xứng (tiên đề Fossen: M_RB = M_RB^T)
    assert(M_rb.isApprox(M_rb.transpose(), 1e-9) && "M_RB must be symmetric!");

    return M_rb;
}

void MassMatrixEvaluator::update_matrices() {
    M_RB_ = compute_M_RB(params_.mass, params_.r_G, params_.I_b);
    M_A_ = params_.M_A;
    M_total_ = M_RB_ + M_A_;

    // Xác minh tính đối xứng của ma trận tổng (cần cho Cholesky)
    assert(M_total_.isApprox(M_total_.transpose(), 1e-9) && "M_total must be symmetric!");
}

bool MassMatrixEvaluator::is_positive_definite() const {
    Eigen::LLT<Matrix6d> llt(M_total_);
    return llt.info() == Eigen::Success;
}

Vector6d MassMatrixEvaluator::solve(const Vector6d& b) const {
    // Giải M * x = b qua Cholesky (LLT), dự phòng bằng LDLT
    Eigen::LLT<Matrix6d> llt(M_total_);
    if (llt.info() == Eigen::Success) {
        return llt.solve(b);
    }
    return M_total_.ldlt().solve(b);
}

MatrixNd MassMatrixEvaluator::compute_reduced(const DofTransformer& transformer) const {
    return transformer.reduce_matrix(M_total_);
}

VectorNd MassMatrixEvaluator::solve_reduced(const DofTransformer& transformer, const VectorNd& b_r) const {
    MatrixNd M_r = compute_reduced(transformer);
    Eigen::LLT<MatrixNd> llt(M_r);
    if (llt.info() == Eigen::Success) {
        return llt.solve(b_r);
    }
    return M_r.ldlt().solve(b_r);
}

} // namespace nav_dynamics
