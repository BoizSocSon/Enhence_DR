#include "nav_dynamics/thruster_allocation.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nav_dynamics {

// =============================================================================
// ThrusterUnit — chuyển đổi PWM ↔ Lực đẩy
// =============================================================================

double ThrusterUnit::pwm_to_thrust(double pwm) const {
    const double diff = pwm - neutral_pwm;
    if (std::abs(diff) <= deadband_pwm) {
        return 0.0;
    }
    if (diff > 0.0) {
        double ratio = (diff - deadband_pwm) / (400.0 - deadband_pwm);
        ratio = std::clamp(ratio, 0.0, 1.0);
        return max_thrust_fwd * (ratio * ratio);
    } else {
        double ratio = (-diff - deadband_pwm) / (400.0 - deadband_pwm);
        ratio = std::clamp(ratio, 0.0, 1.0);
        return -max_thrust_rev * (ratio * ratio);
    }
}

double ThrusterUnit::thrust_to_pwm(double thrust) const {
    if (std::abs(thrust) < 1e-4) {
        return neutral_pwm;
    }
    if (thrust > 0.0) {
        const double clamped = std::clamp(thrust, 0.0, max_thrust_fwd);
        const double ratio = std::sqrt(clamped / max_thrust_fwd);
        return neutral_pwm + deadband_pwm + ratio * (400.0 - deadband_pwm);
    } else {
        const double clamped = std::clamp(-thrust, 0.0, max_thrust_rev);
        const double ratio = std::sqrt(clamped / max_thrust_rev);
        return neutral_pwm - deadband_pwm - ratio * (400.0 - deadband_pwm);
    }
}

// =============================================================================
// ThrusterAllocation
// =============================================================================

ThrusterAllocation::ThrusterAllocation(const std::vector<ThrusterUnit>& thrusters) {
    set_thrusters(thrusters);
}

void ThrusterAllocation::set_thrusters(const std::vector<ThrusterUnit>& thrusters) {
    thrusters_ = thrusters;
    rebuild_allocation_matrix();
}

void ThrusterAllocation::rebuild_allocation_matrix() {
    const size_t k = thrusters_.size();
    if (k == 0) {
        B_ = MatrixNd::Zero(6, 0);
        B_pinv_ = MatrixNd::Zero(0, 6);
        return;
    }

    B_ = MatrixNd::Zero(6, k);
    for (size_t j = 0; j < k; ++j) {
        B_.col(j) = thrusters_[j].wrench_column();
    }

    B_pinv_ = B_.completeOrthogonalDecomposition().pseudoInverse();

    if (current_thrusts_.size() != static_cast<Eigen::Index>(k)) {
        current_thrusts_ = VectorNd::Zero(k);
    }
}

Vector6d ThrusterAllocation::forward_allocation(const VectorNd& thrusts) const {
    if (static_cast<size_t>(thrusts.size()) != thrusters_.size()) {
        throw std::invalid_argument("ThrusterAllocation::forward_allocation dimension mismatch!");
    }
    return B_ * thrusts;
}

VectorNd ThrusterAllocation::forward_allocation_reduced(const DofTransformer& transformer, const VectorNd& thrusts) const {
    return transformer.reduce_vector(forward_allocation(thrusts));
}

MatrixNd ThrusterAllocation::compute_reduced_B(const DofTransformer& transformer) const {
    return transformer.transform_thruster_allocation(B_);
}

VectorNd ThrusterAllocation::inverse_allocation(const Vector6d& desired_tau) const {
    return B_pinv_ * desired_tau;
}

VectorNd ThrusterAllocation::inverse_allocation_reduced(const DofTransformer& transformer, const VectorNd& desired_tau_r) const {
    MatrixNd B_r = transformer.transform_thruster_allocation(B_);
    MatrixNd B_r_pinv = B_r.completeOrthogonalDecomposition().pseudoInverse();
    return B_r_pinv * desired_tau_r;
}

VectorNd ThrusterAllocation::pwm_to_thrusts(const VectorNd& pwms) const {
    const size_t k = thrusters_.size();
    if (static_cast<size_t>(pwms.size()) != k) {
        throw std::invalid_argument("pwm_to_thrusts: dimension mismatch!");
    }
    VectorNd thrusts(k);
    for (size_t j = 0; j < k; ++j) {
        thrusts[j] = thrusters_[j].pwm_to_thrust(pwms[j]);
    }
    return thrusts;
}

VectorNd ThrusterAllocation::thrusts_to_pwm(const VectorNd& thrusts) const {
    const size_t k = thrusters_.size();
    if (static_cast<size_t>(thrusts.size()) != k) {
        throw std::invalid_argument("thrusts_to_pwm: dimension mismatch!");
    }
    VectorNd pwms(k);
    for (size_t j = 0; j < k; ++j) {
        pwms[j] = thrusters_[j].thrust_to_pwm(thrusts[j]);
    }
    return pwms;
}

VectorNd ThrusterAllocation::step_thruster_dynamics(const VectorNd& target_thrusts, double dt) {
    const size_t k = thrusters_.size();
    if (static_cast<size_t>(target_thrusts.size()) != k) {
        throw std::invalid_argument("step_thruster_dynamics: target_thrusts size mismatch!");
    }
    if (current_thrusts_.size() != static_cast<Eigen::Index>(k)) {
        current_thrusts_ = VectorNd::Zero(k);
    }

    for (size_t j = 0; j < k; ++j) {
        const double tau_m = thrusters_[j].time_constant;
        double target = std::clamp(target_thrusts[j], -thrusters_[j].max_thrust_rev, thrusters_[j].max_thrust_fwd);

        if (tau_m <= 1e-6 || dt <= 0.0) {
            current_thrusts_[j] = target;
        } else {
            const double alpha = dt / (tau_m + dt);
            current_thrusts_[j] += alpha * (target - current_thrusts_[j]);
            current_thrusts_[j] = std::clamp(current_thrusts_[j], -thrusters_[j].max_thrust_rev, thrusters_[j].max_thrust_fwd);
        }
    }
    return current_thrusts_;
}

ThrusterAllocation ThrusterAllocation::make_project_rov_3thruster(double l_x, double d_y, double z_t) {
    std::vector<ThrusterUnit> thrusters(3);

    // TL — Ngang bên trái
    thrusters[0].name = "thruster_left";
    thrusters[0].position_body = Vector3d(-l_x, -d_y, 0.0);
    thrusters[0].direction_body = Vector3d(1.0, 0.0, 0.0);

    // TR — Ngang bên phải
    thrusters[1].name = "thruster_right";
    thrusters[1].position_body = Vector3d(-l_x, d_y, 0.0);
    thrusters[1].direction_body = Vector3d(1.0, 0.0, 0.0);

    // TV — Thẳng đứng
    thrusters[2].name = "thruster_vertical";
    thrusters[2].position_body = Vector3d(0.0, 0.0, z_t);
    thrusters[2].direction_body = Vector3d(0.0, 0.0, -1.0);

    return ThrusterAllocation(thrusters);
}

} // namespace nav_dynamics
