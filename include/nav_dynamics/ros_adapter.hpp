#pragma once

#include "nav_dynamics/types.hpp"
#include <array>
#include <cstring>

namespace nav_dynamics {

#pragma pack(push, 1)

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn véc-tơ 3D
 */
struct Vector3POD {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn Quaternion (chuẩn ROS: x, y, z, w)
 */
struct QuaternionPOD {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn Twist tương thích geometry_msgs::Twist
 */
struct TwistPOD {
    Vector3POD linear;
    Vector3POD angular;
};

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn Wrench tương thích geometry_msgs::Wrench
 */
struct WrenchPOD {
    Vector3POD force;
    Vector3POD torque;
};

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn Tư thế Pose tương thích geometry_msgs::Pose
 */
struct PosePOD {
    Vector3POD position;
    QuaternionPOD orientation;
};

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn Gia tốc Accel tương thích geometry_msgs::Accel
 */
struct AccelPOD {
    Vector3POD linear;
    Vector3POD angular;
};

/**
 * @brief Cấu trúc dữ liệu thuần (POD) biểu diễn Odometry tương thích nav_msgs::Odometry
 */
struct OdometryPOD {
    PosePOD pose;
    TwistPOD twist;
};

#pragma pack(pop)

/**
 * @brief Mô-đun ROS Adapter cung cấp khả năng chuyển đổi liền mạch giữa các kiểu dữ liệu
 * Eigen nội bộ của nav_dynamics và các cấu trúc POD tương thích ROS mà không cần phụ thuộc
 * vào bất kỳ gói cài đặt hay bản dựng ROS nào.
 */
class RosAdapter {
public:
    // --- Các hàm chuyển đổi Twist (nu) ---
    static TwistPOD to_twist_pod(const Vector6d& nu);
    static Vector6d from_twist_pod(const TwistPOD& pod);

    // --- Các hàm chuyển đổi Wrench (tau) ---
    static WrenchPOD to_wrench_pod(const Vector6d& tau);
    static Vector6d from_wrench_pod(const WrenchPOD& pod);

    // --- Các hàm chuyển đổi Tư thế Pose ---
    static PosePOD to_pose_pod(const KinematicState& state);
    static void from_pose_pod(const PosePOD& pod, KinematicState& state);

    // --- Các hàm chuyển đổi Gia tốc Accel (nu_dot) ---
    static AccelPOD to_accel_pod(const Vector6d& nu_dot);
    static Vector6d from_accel_pod(const AccelPOD& pod);

    // --- Các hàm chuyển đổi Odometry ---
    static OdometryPOD to_odometry_pod(const KinematicState& state);
    static void from_odometry_pod(const OdometryPOD& pod, KinematicState& state);

    // --- Các thao tác mảng phẳng nhanh / Zero-Copy ---
    static void to_flat_6d(const Vector6d& in, double* out_6d);
    static Vector6d from_flat_6d(const double* in_6d);

    // --- Các template Duck-Typed ánh xạ trực tiếp sang thông điệp ROS 1 / ROS 2 ---
    // Cách sử dụng trong nút ROS:
    //   nav_dynamics::RosAdapter::to_ros_twist(state.nu, ros_twist_msg);
    template <typename RosTwistMsg>
    static void to_ros_twist(const Vector6d& nu, RosTwistMsg& msg) {
        msg.linear.x  = nu(0);
        msg.linear.y  = nu(1);
        msg.linear.z  = nu(2);
        msg.angular.x = nu(3);
        msg.angular.y = nu(4);
        msg.angular.z = nu(5);
    }

    template <typename RosTwistMsg>
    static Vector6d from_ros_twist(const RosTwistMsg& msg) {
        Vector6d nu;
        nu << msg.linear.x, msg.linear.y, msg.linear.z,
              msg.angular.x, msg.angular.y, msg.angular.z;
        return nu;
    }

    template <typename RosWrenchMsg>
    static void to_ros_wrench(const Vector6d& tau, RosWrenchMsg& msg) {
        msg.force.x  = tau(0);
        msg.force.y  = tau(1);
        msg.force.z  = tau(2);
        msg.torque.x = tau(3);
        msg.torque.y = tau(4);
        msg.torque.z = tau(5);
    }

    template <typename RosWrenchMsg>
    static Vector6d from_ros_wrench(const RosWrenchMsg& msg) {
        Vector6d tau;
        tau << msg.force.x, msg.force.y, msg.force.z,
               msg.torque.x, msg.torque.y, msg.torque.z;
        return tau;
    }

    template <typename RosPoseMsg>
    static void to_ros_pose(const KinematicState& state, RosPoseMsg& msg) {
        msg.position.x = state.pos_ned.x();
        msg.position.y = state.pos_ned.y();
        msg.position.z = state.pos_ned.z();
        msg.orientation.x = state.orientation.x();
        msg.orientation.y = state.orientation.y();
        msg.orientation.z = state.orientation.z();
        msg.orientation.w = state.orientation.w();
    }

    template <typename RosPoseMsg>
    static void from_ros_pose(const RosPoseMsg& msg, KinematicState& state) {
        state.pos_ned.x() = msg.position.x;
        state.pos_ned.y() = msg.position.y;
        state.pos_ned.z() = msg.position.z;
        state.orientation.x() = msg.orientation.x;
        state.orientation.y() = msg.orientation.y;
        state.orientation.z() = msg.orientation.z;
        state.orientation.w() = msg.orientation.w;
        state.update_euler_from_quaternion();
    }
};

} // namespace nav_dynamics
