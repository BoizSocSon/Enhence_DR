#include "nav_dynamics/dof_config.hpp"
#include <algorithm>
#include <stdexcept>

namespace nav_dynamics {

DofConfig::DofConfig() : DofConfig(DofPreset::FULL_6DOF) {}

DofConfig::DofConfig(DofPreset preset) {
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
    rebuild_projection_matrix();
}

DofConfig::DofConfig(const std::vector<DofIndex>& active_dofs)
    : active_dofs_(active_dofs) {
    if (active_dofs_.empty()) {
        throw std::invalid_argument("DofConfig: active_dofs cannot be empty!");
    }
    if (active_dofs_.size() > 6) {
        throw std::invalid_argument("DofConfig: cannot exceed 6 DOFs!");
    }
    rebuild_projection_matrix();
}

DofConfig::DofConfig(const MatrixNd& custom_P, const std::vector<DofIndex>& active_dofs)
    : P_(custom_P) {
    if (custom_P.cols() != 6) {
        throw std::invalid_argument("DofConfig: custom_P must have 6 columns!");
    }
    if (custom_P.rows() < 1 || custom_P.rows() > 6) {
        throw std::invalid_argument("DofConfig: custom_P rows must be between 1 and 6!");
    }
    if (!active_dofs.empty()) {
        if (active_dofs.size() != static_cast<size_t>(custom_P.rows())) {
            throw std::invalid_argument("DofConfig: active_dofs size mismatch!");
        }
        active_dofs_ = active_dofs;
    } else {
        active_dofs_.clear();
        for (Eigen::Index r = 0; r < custom_P.rows(); ++r) {
            Eigen::Index max_col = 0;
            double max_val = custom_P.row(r).cwiseAbs().maxCoeff(&max_col);
            if (max_val > 1e-6 && max_col >= 0 && max_col < 6) {
                active_dofs_.push_back(static_cast<DofIndex>(max_col));
            }
        }
    }
}

DofConfig::DofConfig(const DofTransformer& transformer)
    : active_dofs_(transformer.active_dofs()), P_(transformer.T_matrix()) {}

DofTransformer DofConfig::to_transformer() const {
    return DofTransformer(P_, active_dofs_);
}

void DofConfig::rebuild_projection_matrix() {
    const size_t n = active_dofs_.size();
    P_ = MatrixNd::Zero(n, 6);
    for (size_t i = 0; i < n; ++i) {
        auto col = static_cast<size_t>(active_dofs_[i]);
        if (col < 6) {
            P_(i, col) = 1.0;
        }
    }
}

bool DofConfig::is_active(DofIndex dof) const {
    return std::find(active_dofs_.begin(), active_dofs_.end(), dof) != active_dofs_.end();
}

int DofConfig::reduced_index_of(DofIndex dof) const {
    for (size_t i = 0; i < active_dofs_.size(); ++i) {
        if (active_dofs_[i] == dof) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

VectorNd DofConfig::reduce_vector(const Vector6d& v) const {
    return P_ * v;
}

MatrixNd DofConfig::reduce_matrix(const Matrix6d& M) const {
    return P_ * M * P_.transpose();
}

Vector6d DofConfig::expand_vector(const VectorNd& v_r, double default_val) const {
    Vector6d v_full = Vector6d::Constant(default_val);
    const size_t n = active_dofs_.size();
    if (static_cast<size_t>(v_r.size()) != n) {
        throw std::invalid_argument("DofConfig::expand_vector dimension mismatch!");
    }
    for (size_t i = 0; i < n; ++i) {
        auto col = static_cast<size_t>(active_dofs_[i]);
        v_full[col] = v_r[i];
    }
    return v_full;
}

Matrix6d DofConfig::expand_matrix(const MatrixNd& M_r) const {
    const size_t n = active_dofs_.size();
    if (static_cast<size_t>(M_r.rows()) != n || static_cast<size_t>(M_r.cols()) != n) {
        throw std::invalid_argument("DofConfig::expand_matrix dimension mismatch!");
    }
    return P_.transpose() * M_r * P_;
}

std::vector<std::string> DofConfig::active_dof_names() const {
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

DofConfig DofConfig::make_6dof() {
    return DofConfig(DofPreset::FULL_6DOF);
}

DofConfig DofConfig::make_rov_3dof() {
    return DofConfig(DofPreset::ROV_3DOF_SURGE_HEAVE_YAW);
}

DofConfig DofConfig::make_planar_3dof() {
    return DofConfig(DofPreset::PLANAR_3DOF_SURGE_SWAY_YAW);
}

DofConfig DofConfig::make_rov_4dof() {
    return DofConfig(DofPreset::ROV_4DOF);
}

} // namespace nav_dynamics
