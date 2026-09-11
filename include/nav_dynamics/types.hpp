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

constexpr size_t DOF = 6;

/**
 * @brief Chuyển đổi DofIndex sang kiểu chỉ số Index của Eigen
 */
constexpr auto to_idx(DofIndex dof) noexcept {
    return static_cast<Eigen::Index>(dof);
}

/**
 * @brief Hàm phụ trợ tính ma trận phản đối xứng 3x3 của một véc-tơ 3D.
 * [v]_\times * a = v \times a
 */
inline Matrix3d skew(const Vector3d& v) {
    Matrix3d s;
    s <<   0.0, -v.z(),  v.y(),
         v.z(),    0.0, -v.x(),
        -v.y(),  v.x(),    0.0;
    return s;
}

/**
 * @brief Các thông số vật lý và thủy động học của phương tiện (theo mô hình Fossen)
 */
struct VehicleParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // --- Các thông số vật rắn (Rigid-Body) ---
    double mass = 11.5;                         ///< Khối lượng phương tiện [kg]
    Vector3d r_G = Vector3d(0.0, 0.0, 0.02);    ///< Trọng tâm (CoG) trong hệ quy chiếu thân tàu [m]
    Matrix3d I_b = Matrix3d::Zero();            ///< Tensor quán tính trong hệ quy chiếu thân tàu [kg*m^2]

    // --- Các thông số thủy tĩnh (Trọng lực & Lực nổi) ---
    double volume = 0.0115;                     ///< Thể tích chiếm nước [m^3]
    double fluid_density = 1025.0;              ///< Khối lượng riêng của chất lỏng/nước [kg/m^3] (nước biển ~1025, nước ngọt ~1000)
    double gravity = 9.80665;                   ///< Gia tốc trọng trường [m/s^2]
    Vector3d r_B = Vector3d(0.0, 0.0, -0.02);   ///< Tâm nổi (CoB) trong hệ quy chiếu thân tàu [m]

    // --- Ma trận khối lượng gia tăng (Added Mass 6x6, bao gồm cả coupling) ---
    // M_A(6x6) = [M_A_diag + M_A_coupling]
    Matrix6d M_A = Matrix6d::Zero();

    // --- Ma trận cản tuyến tính (Linear Damping 6x6, bao gồm cả coupling) ---
    Matrix6d D_l = Matrix6d::Zero();

    // --- Ma trận cản bậc hai (Quadratic Damping 6x6, bao gồm cả coupling) ---
    Matrix6d D_q = Matrix6d::Zero();

    VehicleParameters() {
        // Tensor quán tính mặc định (kg * m^2)
        set_inertia_tensor(0.16, 0.0, 0.0,
                            0.0, 0.35, 0.0,
                            0.0, 0.0, 0.35);

        // Khối lượng gia tăng đường chéo mặc định (M_A dương)
        set_added_mass_diagonal(5.5, 8.0, 14.6, 0.05, 0.12, 0.12);

        // Cản tuyến tính mặc định (D_l dương)
        set_linear_damping_diagonal(4.03, 6.22, 11.17, 0.07, 0.07, 0.07);

        // Cản bậc hai mặc định (D_q dương)
        set_quadratic_damping_diagonal(18.18, 21.66, 36.99, 1.55, 1.55, 1.55);
    }

    // =========================================================================
    // 1. CÁC HÀM THIẾT LẬP MA TRẬN 6x6 ĐẦY ĐỦ (HỖ TRỢ COUPLING TERMS)
    // =========================================================================

    /// Gán trực tiếp ma trận Added Mass 6x6 đầy đủ
    void set_added_mass(const Matrix6d& M) {
        M_A = M;
    }

    /// Nạp Added Mass 6x6 từ mảng bộ nhớ liên tục (RowMajor theo mặc định) qua Eigen::Map
    void set_added_mass(const double* data, bool row_major = true) {
        if (row_major) {
            M_A = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(data);
        } else {
            M_A = Eigen::Map<const Matrix6d>(data);
        }
    }

    /// Gán trực tiếp ma trận Linear Damping 6x6 đầy đủ
    void set_linear_damping(const Matrix6d& D) {
        D_l = D;
    }

    /// Nạp Linear Damping 6x6 từ mảng bộ nhớ liên tục (RowMajor) qua Eigen::Map
    void set_linear_damping(const double* data, bool row_major = true) {
        if (row_major) {
            D_l = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(data);
        } else {
            D_l = Eigen::Map<const Matrix6d>(data);
        }
    }

    /// Gán trực tiếp ma trận Quadratic Damping 6x6 đầy đủ
    void set_quadratic_damping(const Matrix6d& D) {
        D_q = D;
    }

    /// Nạp Quadratic Damping 6x6 từ mảng bộ nhớ liên tục (RowMajor) qua Eigen::Map
    void set_quadratic_damping(const double* data, bool row_major = true) {
        if (row_major) {
            D_q = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(data);
        } else {
            D_q = Eigen::Map<const Matrix6d>(data);
        }
    }

    // =========================================================================
    // 2. CÁC HÀM THIẾT LẬP ĐƯỜNG CHÉO (DIAGONAL ONLY - 6 THAM SỐ)
    // =========================================================================

    void set_added_mass_diagonal(const Vector6d& diag) {
        M_A = diag.asDiagonal();
    }

    void set_added_mass_diagonal(double X_udot, double Y_vdot, double Z_wdot,
                                 double K_pdot, double M_qdot, double N_rdot) {
        M_A.setZero();
        M_A.diagonal() << X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot;
    }

    void set_linear_damping_diagonal(const Vector6d& diag) {
        D_l = diag.asDiagonal();
    }

    void set_linear_damping_diagonal(double Xu, double Yv, double Zw,
                                     double Kp, double Mq, double Nr) {
        D_l.setZero();
        D_l.diagonal() << Xu, Yv, Zw, Kp, Mq, Nr;
    }

    void set_quadratic_damping_diagonal(const Vector6d& diag) {
        D_q = diag.asDiagonal();
    }

    void set_quadratic_damping_diagonal(double X_uu, double Y_vv, double Z_ww,
                                        double K_pp, double M_qq, double N_rr) {
        D_q.setZero();
        D_q.diagonal() << X_uu, Y_vv, Z_ww, K_pp, M_qq, N_rr;
    }

    // =========================================================================
    // 3. THIẾT LẬP TENSOR QUÁN TÍNH
    // =========================================================================

    void set_inertia_tensor(const Matrix3d& I) {
        I_b = I;
    }

    void set_inertia_tensor(double I_xx, double I_xy, double I_xz,
                            double I_yx, double I_yy, double I_yz,
                            double I_zx, double I_zy, double I_zz) {
        I_b << I_xx, I_xy, I_xz,
               I_yx, I_yy, I_yz,
               I_zx, I_zy, I_zz;
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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector3d pos_ned = Vector3d::Zero();                             ///< Vị trí trong hệ quy chiếu NED [m] (Bắc, Đông, Xuống)
    Vector3d euler_rpy = Vector3d::Zero();                           ///< Các góc Euler [rad] (roll phi, pitch theta, yaw psi)
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity(); ///< Quaternion định hướng (w, x, y, z)

    Vector6d nu = Vector6d::Zero();                                  ///< Vận tốc trong hệ thân tàu [m/s, rad/s] (u, v, w, p, q, r)
    Vector6d nu_dot = Vector6d::Zero();                              ///< Gia tốc trong hệ thân tàu [m/s^2, rad/s^2]

    KinematicState() = default;

    /// Cập nhật quaternion định hướng từ các góc Euler hiện tại (theo chuỗi ZYX)
    void update_quaternion_from_euler() {
        orientation = Eigen::AngleAxisd(euler_rpy.z(), Vector3d::UnitZ())
                    * Eigen::AngleAxisd(euler_rpy.y(), Vector3d::UnitY())
                    * Eigen::AngleAxisd(euler_rpy.x(), Vector3d::UnitX());
        orientation.normalize();
    }

    /// Cập nhật các góc Euler từ quaternion định hướng hiện tại (có xử lý Gimbal Lock)
    void update_euler_from_quaternion() {
        const auto& q = orientation;
        double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());

        if (std::abs(sinp) >= 0.99999) {
            // Gimbal lock (pitch = +-90 deg): roll và yaw đồng trục
            euler_rpy.x() = 0.0;
            euler_rpy.y() = std::copysign(M_PI / 2.0, sinp);
            euler_rpy.z() = -2.0 * std::atan2(q.z(), q.w());
        } else {
            euler_rpy.y() = std::asin(sinp);

            double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
            double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
            euler_rpy.x() = std::atan2(sinr_cosp, cosr_cosp);

            double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
            double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
            euler_rpy.z() = std::atan2(siny_cosp, cosy_cosp);
        }
    }

    /// Ma trận quay từ hệ quy chiếu Thân tàu sang hệ NED: R_nb = R_b^n
    [[nodiscard]] Matrix3d R_nb() const {
        return orientation.toRotationMatrix();
    }

    /// Ma trận quay từ hệ quy chiếu NED sang hệ Thân tàu: R_bn = (R_nb)^T
    [[nodiscard]] Matrix3d R_bn() const {
        return orientation.conjugate().toRotationMatrix();
    }

    /// Ma trận biến đổi động học 6x6 J(eta) ánh xạ nu -> eta_dot (đã tối ưu hóa SIMD & lượng giác)
    [[nodiscard]] Matrix6d J_full() const {
        Matrix6d J = Matrix6d::Zero();
        J.block<3, 3>(0, 0) = R_nb();

        const double phi = euler_rpy.x();
        const double theta = euler_rpy.y();

        const double sin_phi = std::sin(phi);
        const double cos_phi = std::cos(phi);
        const double sin_theta = std::sin(theta);
        double cos_theta = std::cos(theta);

        if (std::abs(cos_theta) < 1e-6) {
            cos_theta = std::copysign(1e-6, cos_theta); // Bảo toàn dấu tránh đảo chiều góc 180 độ
        }
        const double inv_cos = 1.0 / cos_theta;
        const double tan_theta = sin_theta * inv_cos;

        Matrix3d T;
        T << 1.0,  sin_phi * tan_theta,   cos_phi * tan_theta,
             0.0,  cos_phi,              -sin_phi,
             0.0,  sin_phi * inv_cos,     cos_phi * inv_cos;

        J.block<3, 3>(3, 3) = T;
        return J;
    }
};

/**
 * @brief Lực và mô-men tổng quát (wrench) trong hệ quy chiếu thân tàu
 */
struct ControlWrench {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector6d tau = Vector6d::Zero(); ///< [X, Y, Z, K, M, N]^T theo đơn vị [N, Nm]

    ControlWrench() = default;
    explicit ControlWrench(const Vector6d& t) : tau(t) {}
    ControlWrench(double X, double Y, double Z, double K, double M, double N) {
        tau << X, Y, Z, K, M, N;
    }

    [[nodiscard]] auto force() const { return tau.head<3>(); }
    [[nodiscard]] auto torque() const { return tau.tail<3>(); }
    auto force() { return tau.head<3>(); }
    auto torque() { return tau.tail<3>(); }

    void set_force(const Vector3d& f) { tau.head<3>() = f; }
    void set_torque(const Vector3d& t) { tau.tail<3>() = t; }
};

/**
 * @brief Vận tốc dòng chảy đại dương / chất lỏng
 */
struct FluidCurrent {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vector3d v_c_ned = Vector3d::Zero(); ///< Vận tốc dòng chảy trong hệ quy chiếu NED [m/s]

    FluidCurrent() = default;
    explicit FluidCurrent(const Vector3d& vc) : v_c_ned(vc) {}

    /// Tính vận tốc tương đối nu_r = nu - nu_c_body
    [[nodiscard]] Vector6d compute_relative_velocity(const KinematicState& state) const {
        Vector6d nu_r = state.nu;
        // Quay trực tiếp qua Quaternion liên hợp: tránh tạo ma trận 3x3 và chuyển vị, nhanh gấp ~3 lần
        nu_r.head<3>() -= state.orientation.conjugate() * v_c_ned;
        return nu_r;
    }
};

} // namespace nav_dynamics
