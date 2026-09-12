#include "nav_dynamics/ros_adapter.hpp"

namespace nav_dynamics {

TwistPOD RosAdapter::to_twist_pod(const Vector6d& nu) {
    TwistPOD pod;
    pod.linear.x  = nu(0);
    pod.linear.y  = nu(1);
    pod.linear.z  = nu(2);
    pod.angular.x = nu(3);
    pod.angular.y = nu(4);
    pod.angular.z = nu(5);
    return pod;
}

Vector6d RosAdapter::from_twist_pod(const TwistPOD& pod) {
    Vector6d nu;
    nu << pod.linear.x, pod.linear.y, pod.linear.z,
          pod.angular.x, pod.angular.y, pod.angular.z;
    return nu;
}

WrenchPOD RosAdapter::to_wrench_pod(const Vector6d& tau) {
    WrenchPOD pod;
    pod.force.x  = tau(0);
    pod.force.y  = tau(1);
    pod.force.z  = tau(2);
    pod.torque.x = tau(3);
    pod.torque.y = tau(4);
    pod.torque.z = tau(5);
    return pod;
}

Vector6d RosAdapter::from_wrench_pod(const WrenchPOD& pod) {
    Vector6d tau;
    tau << pod.force.x, pod.force.y, pod.force.z,
           pod.torque.x, pod.torque.y, pod.torque.z;
    return tau;
}

PosePOD RosAdapter::to_pose_pod(const KinematicState& state) {
    PosePOD pod;
    pod.position.x = state.pos_ned.x();
    pod.position.y = state.pos_ned.y();
    pod.position.z = state.pos_ned.z();
    pod.orientation.x = state.orientation.x();
    pod.orientation.y = state.orientation.y();
    pod.orientation.z = state.orientation.z();
    pod.orientation.w = state.orientation.w();
    return pod;
}

void RosAdapter::from_pose_pod(const PosePOD& pod, KinematicState& state) {
    state.pos_ned.x() = pod.position.x;
    state.pos_ned.y() = pod.position.y;
    state.pos_ned.z() = pod.position.z;
    state.orientation.x() = pod.orientation.x;
    state.orientation.y() = pod.orientation.y;
    state.orientation.z() = pod.orientation.z;
    state.orientation.w() = pod.orientation.w;
    state.update_euler_from_quaternion();
}

AccelPOD RosAdapter::to_accel_pod(const Vector6d& nu_dot) {
    AccelPOD pod;
    pod.linear.x  = nu_dot(0);
    pod.linear.y  = nu_dot(1);
    pod.linear.z  = nu_dot(2);
    pod.angular.x = nu_dot(3);
    pod.angular.y = nu_dot(4);
    pod.angular.z = nu_dot(5);
    return pod;
}

Vector6d RosAdapter::from_accel_pod(const AccelPOD& pod) {
    Vector6d nu_dot;
    nu_dot << pod.linear.x, pod.linear.y, pod.linear.z,
              pod.angular.x, pod.angular.y, pod.angular.z;
    return nu_dot;
}

OdometryPOD RosAdapter::to_odometry_pod(const KinematicState& state) {
    OdometryPOD pod;
    pod.pose = to_pose_pod(state);
    pod.twist = to_twist_pod(state.nu);
    return pod;
}

void RosAdapter::from_odometry_pod(const OdometryPOD& pod, KinematicState& state) {
    from_pose_pod(pod.pose, state);
    state.nu = from_twist_pod(pod.twist);
}

void RosAdapter::to_flat_6d(const Vector6d& in, double* out_6d) {
    std::memcpy(out_6d, in.data(), 6 * sizeof(double));
}

Vector6d RosAdapter::from_flat_6d(const double* in_6d) {
    Vector6d out;
    std::memcpy(out.data(), in_6d, 6 * sizeof(double));
    return out;
}

VectorNd RosAdapter::reduce_vector(const Vector6d& v_6d, const DofTransformer& transformer) {
    return transformer.transform_vector(v_6d);
}

Vector6d RosAdapter::expand_vector(const VectorNd& v_r, const DofTransformer& transformer,
                                   const Vector6d& default_constrained) {
    return transformer.expand_vector(v_r, default_constrained);
}

VectorNd RosAdapter::to_reduced_twist(const Vector6d& nu, const DofTransformer& transformer) {
    return transformer.transform_vector(nu);
}

Vector6d RosAdapter::from_reduced_twist(const VectorNd& nu_r, const DofTransformer& transformer) {
    return transformer.expand_vector(nu_r);
}

VectorNd RosAdapter::to_reduced_wrench(const Vector6d& tau, const DofTransformer& transformer) {
    return transformer.transform_wrench(tau);
}

Vector6d RosAdapter::from_reduced_wrench(const VectorNd& tau_r, const DofTransformer& transformer) {
    return transformer.expand_vector(tau_r);
}

VectorNd RosAdapter::to_reduced_accel(const Vector6d& nu_dot, const DofTransformer& transformer) {
    return transformer.transform_vector(nu_dot);
}

Vector6d RosAdapter::from_reduced_accel(const VectorNd& nu_dot_r, const DofTransformer& transformer) {
    return transformer.expand_vector(nu_dot_r);
}

TwistPOD RosAdapter::to_rov_3dof_twist_pod(double u, double w, double r) {
    TwistPOD pod;
    pod.linear.x = u;
    pod.linear.y = 0.0;
    pod.linear.z = w;
    pod.angular.x = 0.0;
    pod.angular.y = 0.0;
    pod.angular.z = r;
    return pod;
}

void RosAdapter::from_rov_3dof_twist_pod(const TwistPOD& pod, double& u, double& w, double& r) {
    u = pod.linear.x;
    w = pod.linear.z;
    r = pod.angular.z;
}

} // namespace nav_dynamics
