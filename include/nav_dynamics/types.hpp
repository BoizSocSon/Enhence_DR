#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <string>
#include <vector>

namespace nav_dynamics {

// Các bí danh (aliases) chuẩn của Eigen
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using VectorNd = Eigen::VectorXd;
using MatrixNd = Eigen::MatrixXd;

/**
 * @brief Bảng liệt kê chỉ số cho 6 bậc tự do (theo quy chuẩn SNAME)
 */
enum class DofIndex : uint8_t {
    SURGE = 0, ///< Chuyển động tịnh tiến dọc trục x (tiến/lùi)
    SWAY  = 1, ///< Chuyển động tịnh tiến dọc trục y (dạt ngang mạn phải/trái)
    HEAVE = 2, ///< Chuyển động tịnh tiến dọc trục z (hạ xuống/chìm/nổi)
    ROLL  = 3, ///< Chuyển động quay quanh trục x (lắc ngang trái/phải)
    PITCH = 4, ///< Chuyển động quay quanh trục y (chúi/ngóc mũi)
    YAW   = 5  ///< Chuyển động quay quanh trục z (quay trở/đổi hướng trái/phải)
};

/**
 * @brief Hàm phụ trợ tính ma trận phản đối xứng 3x3 của một véc-tơ 3D.
 * [v]_\times * a = v \times a
 */
inline Matrix3d skew(const Vector3d& v) {
    Matrix3d s;
    s <<      0.0, -v.z(),  v.y(),
          v.z(),     0.0, -v.x(),
         -v.y(),  v.x(),     0.0;
    return s;
}

/**
 * @brief Các thông số vật lý và thủy động học của phương tiện (theo mô hình Fossen)
 */
struct VehicleParameters {
    // --- Các thông số vật rắn (Rigid-Body) ---
    double mass = 11.5;                         ///< Khối lượng phương tiện [kg]
    Vector3d r_G = Vector3d(0.0, 0.0, 0.02);    ///< Trọng tâm (CoG) trong hệ quy chiếu thân tàu [m]
    Matrix3d I_b = Matrix3d::Identity();        ///< Tensor quán tính trong hệ quy chiếu thân tàu [kg*m^2]

    // --- Các thông số thủy tĩnh (Trọng lực & Lực nổi) ---
    double volume = 0.0115;                     ///< Thể tích chiếm nước [m^3]
    double fluid_density = 1025.0;              ///< Khối lượng riêng của chất lỏng/nước [kg/m^3] (nước biển ~1025, nước ngọt ~1000)
    double gravity = 9.80665;                   ///< Gia tốc trọng trường [m/s^2]
    Vector3d r_B = Vector3d(0.0, 0.0, -0.02);   ///< Tâm nổi (CoB) trong hệ quy chiếu thân tàu [m]

    // --- Ma trận khối lượng gia tăng (Added Mass 6x6) ---
    // M_A = -diag(X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot)
    Matrix6d M_A = Matrix6d::Zero();

    // --- Ma trận cản tuyến tính (Linear Damping 6x6) ---
    // D_l = -diag(X_u, Y_v, Z_w, K_p, M_q, N_r)
    Matrix6d D_l = Matrix6d::Zero();

    // --- Ma trận / Các hệ số cản bậc hai (phi tuyến) (Quadratic Damping 6x6) ---
    // D_q = -diag(X_uu * |u|, Y_vv * |v|, Z_ww * |w|, K_pp * |p|, M_qq * |q|, N_rr * |r|)
    Matrix6d D_q = Matrix6d::Zero();

    VehicleParameters() {
        // Tensor quán tính mặc định (kg * m^2)
        I_b << 0.16,  0.0,   0.0,
               0.0,   0.35,  0.0,
               0.0,   0.0,   0.35;

        // Khối lượng gia tăng đường chéo mặc định (các đại lượng cộng thêm dương, M_A = -diag(hydro_coeffs))
        // ví dụ: X_udot = -5.5 kg -> M_A(0,0) = +5.5 kg
        set_added_mass_diagonal(5.5, 8.0, 14.6, 0.05, 0.12, 0.12);

        // Cản tuyến tính mặc định (các hệ số cản dương D_l)
        // ví dụ: X_u = -4.03 -> D_l(0,0) = +4.03 Ns/m
        set_linear_damping_diagonal(4.03, 6.22, 11.17, 0.07, 0.07, 0.07);

        // Cản bậc hai mặc định (các hệ số cản dương D_q)
        // ví dụ: X_uu = -18.18 -> D_q(0,0) = +18.18 Ns^2/m^2
        set_quadratic_damping_diagonal(18.18, 21.66, 36.99, 1.55, 1.55, 1.55);
    }

    void set_added_mass_diagonal(double X_udot, double Y_vdot, double Z_wdot,
                                 double K_pdot, double M_qdot, double N_rdot) {
        M_A.setZero();
        M_A(0, 0) = std::abs(X_udot);
        M_A(1, 1) = std::abs(Y_vdot);
        M_A(2, 2) = std::abs(Z_wdot);
        M_A(3, 3) = std::abs(K_pdot);
        M_A(4, 4) = std::abs(M_qdot);
        M_A(5, 5) = std::abs(N_rdot);
    }

    void set_linear_damping_diagonal(double Xu, double Yv, double Zw,
                                     double Kp, double Mq, double Nr) {
        D_l.setZero();
        D_l(0, 0) = std::abs(Xu);
        D_l(1, 1) = std::abs(Yv);
        D_l(2, 2) = std::abs(Zw);
        D_l(3, 3) = std::abs(Kp);
        D_l(4, 4) = std::abs(Mq);
        D_l(5, 5) = std::abs(Nr);
    }

    void set_quadratic_damping_diagonal(double Xuu, double Yvv, double Zww,
                                        double Kpp, double Mqq, double Nrr) {
        D_q.setZero();
        D_q(0, 0) = std::abs(Xuu);
        D_q(1, 1) = std::abs(Yvv);
        D_q(2, 2) = std::abs(Zww);
        D_q(3, 3) = std::abs(Kpp);
        D_q(4, 4) = std::abs(Mqq);
        D_q(5, 5) = std::abs(Nrr);
    }

    [[nodiscard]] double weight() const {
        return mass * gravity;
    }

    [[nodiscard]] double buoyancy() const {
        return fluid_density * gravity * volume;
    }
};

/**
 * @brief Trạng thái động học biểu diễn tư thế 6D, vận tốc và gia tốc trong hệ thân tàu
 */
struct KinematicState {
    Vector3d pos_ned = Vector3d::Zero();                       ///< Vị trí trong hệ quy chiếu NED [m] (Bắc, Đông, Xuống)
    Vector3d euler_rpy = Vector3d::Zero();                     ///< Các góc Euler [rad] (lắc ngang roll phi, chúi pitch theta, quay yaw psi)
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); ///< Quaternion định hướng (w, x, y, z)

    Vector6d nu = Vector6d::Zero();                            ///< Vận tốc trong hệ thân tàu [m/s, rad/s] (u, v, w, p, q, r)
    Vector6d nu_dot = Vector6d::Zero();                        ///< Gia tốc trong hệ thân tàu [m/s^2, rad/s^2]

    KinematicState() = default;

    /// Cập nhật quaternion định hướng từ các góc Euler hiện tại (theo chuỗi ZYX)
    void update_quaternion_from_euler() {
        orientation = Eigen::AngleAxisd(euler_rpy.z(), Vector3d::UnitZ())
                    * Eigen::AngleAxisd(euler_rpy.y(), Vector3d::UnitY())
                    * Eigen::AngleAxisd(euler_rpy.x(), Vector3d::UnitX());
        orientation.normalize();
    }

    /// Cập nhật các góc Euler từ quaternion định hướng hiện tại
    void update_euler_from_quaternion() {
        // Roll (x), Pitch (y), Yaw (z) theo quy ước ZYX
        const auto& q = orientation;
        // Roll (phi)
        double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        euler_rpy.x() = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (theta)
        double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1.0) {
            euler_rpy.y() = std::copysign(M_PI / 2.0, sinp); // giới hạn góc ở 90 độ
        } else {
            euler_rpy.y() = std::asin(sinp);
        }

        // Yaw (psi)
        double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        euler_rpy.z() = std::atan2(siny_cosp, cosy_cosp);
    }

    /// Ma trận quay từ hệ quy chiếu Thân tàu sang hệ NED: R_nb = R_b^n
    [[nodiscard]] Matrix3d R_nb() const {
        return orientation.toRotationMatrix();
    }

    /// Ma trận quay từ hệ quy chiếu NED sang hệ Thân tàu: R_bn = (R_nb)^T
    [[nodiscard]] Matrix3d R_bn() const {
        return R_nb().transpose();
    }

    /// Ma trận biến đổi động học 6x6 J(eta) ánh xạ nu -> eta_dot
    [[nodiscard]] Matrix6d J_full() const {
        Matrix6d J = Matrix6d::Zero();
        Matrix3d R = R_nb();
        J.block<3, 3>(0, 0) = R;

        double phi = euler_rpy.x();
        double theta = euler_rpy.y();
        double cos_theta = std::cos(theta);
        if (std::abs(cos_theta) < 1e-6) {
            cos_theta = 1e-6; // tránh chia cho 0 do khóa trục (gimbal lock)
        }
        Matrix3d T;
        T << 1.0, std::sin(phi) * std::tan(theta),  std::cos(phi) * std::tan(theta),
             0.0, std::cos(phi),                   -std::sin(phi),
             0.0, std::sin(phi) / cos_theta,        std::cos(phi) / cos_theta;

        J.block<3, 3>(3, 3) = T;
        return J;
    }
};

/**
 * @brief Lực và mô-men tổng quát (wrench) trong hệ quy chiếu thân tàu
 */
struct ControlWrench {
    Vector6d tau = Vector6d::Zero(); ///< [X, Y, Z, K, M, N]^T theo đơn vị [N, Nm]

    ControlWrench() = default;
    explicit ControlWrench(const Vector6d& t) : tau(t) {}
    ControlWrench(double X, double Y, double Z, double K, double M, double N) {
        tau << X, Y, Z, K, M, N;
    }

    [[nodiscard]] Vector3d force() const { return tau.head<3>(); }
    [[nodiscard]] Vector3d torque() const { return tau.tail<3>(); }

    void set_force(const Vector3d& f) { tau.head<3>() = f; }
    void set_torque(const Vector3d& t) { tau.tail<3>() = t; }
};

/**
 * @brief Vận tốc dòng chảy đại dương / chất lỏng
 */
struct FluidCurrent {
    Vector3d v_c_ned = Vector3d::Zero(); ///< Vận tốc dòng chảy trong hệ quy chiếu NED [m/s]

    FluidCurrent() = default;
    explicit FluidCurrent(const Vector3d& vc) : v_c_ned(vc) {}

    /// Tính vận tốc tương đối nu_r = nu - nu_c_body
    [[nodiscard]] Vector6d compute_relative_velocity(const KinematicState& state) const {
        Vector6d nu_r = state.nu;
        // Biến đổi dòng chảy từ hệ quy chiếu NED sang hệ thân tàu
        Vector3d v_c_body = state.R_bn() * v_c_ned;
        // Dòng chảy chỉ ảnh hưởng trực tiếp lên vận tốc tịnh tiến (surge, sway, heave)
        nu_r.head<3>() -= v_c_body;
        return nu_r;
    }
};

} // namespace nav_dynamics
