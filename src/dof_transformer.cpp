#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/dof_config.hpp"
#include <algorithm>
#include <stdexcept>

namespace nav_dynamics {

DofTransformer::DofTransformer()
    : DofTransformer(DofPreset::FULL_6DOF) {}

DofTransformer::DofTransformer(DofPreset preset) {
    switch (preset) {
        case DofPreset::FULL_6DOF:
            active_dofs_ = {DofIndex::SURGE, DofIndex::SWAY,  DofIndex::HEAVE,
                            DofIndex::ROLL,  DofIndex::PITCH, DofIndex::YAW};
            break;
        case DofPreset::ROV_3DOF_SURGE_HEAVE_YAW:
            active_dofs_ = {DofIndex::SURGE, DofIndex::HEAVE, DofIndex::YAW};
            break;
        case DofPreset::PLANAR_3DOF_SURGE_SWAY_YAW:
            active_dofs_ = {DofIndex::SURGE, DofIndex::SWAY, DofIndex::YAW};
            break;
        case DofPreset::ROV_4DOF:
            active_dofs_ = {DofIndex::SURGE, DofIndex::SWAY, DofIndex::HEAVE, DofIndex::YAW};
            break;
        case DofPreset::CUSTOM:
            active_dofs_ = {DofIndex::SURGE, DofIndex::SWAY,  DofIndex::HEAVE,
                            DofIndex::ROLL,  DofIndex::PITCH, DofIndex::YAW};
            break;
    }
    reduced_dim_ = active_dofs_.size();
    build_T_from_active_dofs();
}

DofTransformer::DofTransformer(const std::vector<DofIndex>& active_dofs)
    : active_dofs_(active_dofs), reduced_dim_(active_dofs.size()) {
    if (active_dofs_.empty()) {
        throw std::invalid_argument("DofTransformer: active_dofs cannot be empty!");
    }
    if (active_dofs_.size() > 6) {
        throw std::invalid_argument("DofTransformer: active_dofs cannot exceed 6!");
    }
    build_T_from_active_dofs();
}

DofTransformer::DofTransformer(const MatrixNd& custom_T, const std::vector<DofIndex>& active_dofs)
    : reduced_dim_(custom_T.rows()), T_(custom_T) {
    if (custom_T.cols() != 6) {
        throw std::invalid_argument("DofTransformer: custom_T must have exactly 6 columns!");
    }
    if (custom_T.rows() < 1 || custom_T.rows() > 6) {
        throw std::invalid_argument("DofTransformer: custom_T rows must be between 1 and 6!");
    }

    if (!active_dofs.empty()) {
        if (active_dofs.size() != static_cast<size_t>(custom_T.rows())) {
            throw std::invalid_argument("DofTransformer: active_dofs size does not match custom_T rows!");
        }
        active_dofs_ = active_dofs;
    } else {
        active_dofs_.clear();
        for (Eigen::Index r = 0; r < custom_T.rows(); ++r) {
            Eigen::Index max_col = 0;
            double max_val = custom_T.row(r).cwiseAbs().maxCoeff(&max_col);
            if (max_val > 1e-6 && max_col >= 0 && max_col < 6) {
                active_dofs_.push_back(static_cast<DofIndex>(max_col));
            }
        }
    }
}

void DofTransformer::build_T_from_active_dofs() {
    T_ = MatrixNd::Zero(reduced_dim_, 6);
    for (size_t i = 0; i < reduced_dim_; ++i) {
        auto col = static_cast<size_t>(active_dofs_[i]);
        if (col < 6) {
            T_(i, col) = 1.0;
        }
    }
}

bool DofTransformer::is_dof_active(DofIndex dof) const {
    return std::find(active_dofs_.begin(), active_dofs_.end(), dof) != active_dofs_.end();
}

int DofTransformer::reduced_index_of(DofIndex dof) const {
    for (size_t i = 0; i < active_dofs_.size(); ++i) {
        if (active_dofs_[i] == dof) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::vector<std::string> DofTransformer::active_dof_names() const {
    std::vector<std::string> names;
    names.reserve(active_dofs_.size());
    for (auto dof : active_dofs_) {
        switch (dof) {
            case DofIndex::SURGE: names.emplace_back("Surge (u)"); break;
            case DofIndex::SWAY:  names.emplace_back("Sway (v)");  break;
            case DofIndex::HEAVE: names.emplace_back("Heave (w)"); break;
            case DofIndex::ROLL:  names.emplace_back("Roll (p)");  break;
            case DofIndex::PITCH: names.emplace_back("Pitch (q)"); break;
            case DofIndex::YAW:   names.emplace_back("Yaw (r)");   break;
        }
    }
    return names;
}

bool DofTransformer::is_orthogonal(double tol) const {
    MatrixNd I_r = MatrixNd::Identity(reduced_dim_, reduced_dim_);
    return (T_ * T_.transpose()).isApprox(I_r, tol);
}

MatrixNd DofTransformer::transform_mass(const Matrix6d& M) const {
    return T_ * M * T_.transpose();
}

MatrixNd DofTransformer::transform_coriolis(const Matrix6d& C) const {
    return T_ * C * T_.transpose();
}

MatrixNd DofTransformer::transform_damping(const Matrix6d& D) const {
    return T_ * D * T_.transpose();
}

VectorNd DofTransformer::transform_vector(const Vector6d& v) const {
    return T_ * v;
}

VectorNd DofTransformer::transform_restoring(const Vector6d& g) const {
    return T_ * g;
}

VectorNd DofTransformer::transform_wrench(const Vector6d& tau) const {
    return T_ * tau;
}

MatrixNd DofTransformer::transform_thruster_allocation(const MatrixNd& B) const {
    if (B.rows() != 6) {
        throw std::invalid_argument("DofTransformer::transform_thruster_allocation: B must have 6 rows!");
    }
    return T_ * B;
}

Vector6d DofTransformer::expand_vector(const VectorNd& v_r,
                                       const Vector6d& constrained_vals) const {
    if (static_cast<size_t>(v_r.size()) != reduced_dim_) {
        throw std::invalid_argument("DofTransformer::expand_vector: dimension mismatch!");
    }
    // Mở rộng về 6D và bù trừ các bậc tự do bị ràng buộc:
    // v_6d = T^T * v_r + (I_6 - T^T * T) * constrained_vals
    Matrix6d I_6 = Matrix6d::Identity();
    Matrix6d null_projector = I_6 - (T_.transpose() * T_);
    return (T_.transpose() * v_r) + (null_projector * constrained_vals);
}

Matrix6d DofTransformer::expand_matrix(const MatrixNd& M_r) const {
    if (static_cast<size_t>(M_r.rows()) != reduced_dim_ || static_cast<size_t>(M_r.cols()) != reduced_dim_) {
        throw std::invalid_argument("DofTransformer::expand_matrix: dimension mismatch!");
    }
    return T_.transpose() * M_r * T_;
}

DofTransformer DofTransformer::make_6dof() {
    return DofTransformer(DofPreset::FULL_6DOF);
}

DofTransformer DofTransformer::make_rov_3dof() {
    return DofTransformer(DofPreset::ROV_3DOF_SURGE_HEAVE_YAW);
}

DofTransformer DofTransformer::make_planar_3dof() {
    return DofTransformer(DofPreset::PLANAR_3DOF_SURGE_SWAY_YAW);
}

DofTransformer DofTransformer::make_rov_4dof() {
    return DofTransformer(DofPreset::ROV_4DOF);
}

DofTransformer DofTransformer::from_preset(DofPreset preset) {
    return DofTransformer(preset);
}

DofTransformer DofTransformer::from_dof_names(const std::vector<std::string>& dof_names) {
    std::vector<DofIndex> active;
    active.reserve(dof_names.size());
    for (const auto& name : dof_names) {
        std::string upper_name = name;
        std::transform(upper_name.begin(), upper_name.end(), upper_name.begin(), ::toupper);
        if (upper_name == "SURGE" || upper_name == "U") {
            active.push_back(DofIndex::SURGE);
        } else if (upper_name == "SWAY" || upper_name == "V") {
            active.push_back(DofIndex::SWAY);
        } else if (upper_name == "HEAVE" || upper_name == "W") {
            active.push_back(DofIndex::HEAVE);
        } else if (upper_name == "ROLL" || upper_name == "P") {
            active.push_back(DofIndex::ROLL);
        } else if (upper_name == "PITCH" || upper_name == "Q") {
            active.push_back(DofIndex::PITCH);
        } else if (upper_name == "YAW" || upper_name == "R") {
            active.push_back(DofIndex::YAW);
        } else {
            throw std::invalid_argument("Unknown DOF name: " + name);
        }
    }
    return DofTransformer(active);
}

DofConfig DofTransformer::to_dof_config() const {
    return DofConfig(T_, active_dofs_);
}

DofTransformer DofTransformer::from_matrix(const MatrixNd& custom_T, const std::vector<DofIndex>& active_dofs) {
    return DofTransformer(custom_T, active_dofs);
}

} // namespace nav_dynamics
