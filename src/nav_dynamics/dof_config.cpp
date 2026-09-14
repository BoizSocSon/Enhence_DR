#include "nav_dynamics/dof_config.hpp"

namespace nav_dynamics {

// =============================================================================
// Constructors — mỗi constructor delegate sang DofTransformer tương ứng
// =============================================================================

DofConfig::DofConfig()
    : transformer_(DofPreset::ROV_6DOF_FULL) {}

DofConfig::DofConfig(DofPreset preset)
    : transformer_(preset) {}

DofConfig::DofConfig(const std::vector<DofIndex>& active_dofs)
    : transformer_(active_dofs) {}

DofConfig::DofConfig(const MatrixNd& custom_P, const std::vector<DofIndex>& active_dofs)
    : transformer_(custom_P, active_dofs) {}

DofConfig::DofConfig(const DofTransformer& transformer)
    : transformer_(transformer) {}

// =============================================================================
// expand_vector — chuyển đổi scalar default_val sang Vector6d::Constant
// =============================================================================

Vector6d DofConfig::expand_vector(const VectorNd& v_r, double default_val) const {
    return transformer_.expand_vector(v_r, Vector6d::Constant(default_val));
}

// =============================================================================
// Static factory helpers
// =============================================================================

DofConfig DofConfig::make_6dof() {
    return DofConfig(DofPreset::ROV_6DOF_FULL);
}

DofConfig DofConfig::make_rov_4dof() {
    return DofConfig(DofPreset::ROV_4DOF_CONFIG_1);
}

DofConfig DofConfig::make_rov_3dof() {
    return DofConfig(DofPreset::ROV_3DOF_CONFIG_1);
}

DofConfig DofConfig::make_planar_3dof() {
    return DofConfig(std::vector<DofIndex>{DofIndex::SURGE, DofIndex::SWAY, DofIndex::YAW});
}

} // namespace nav_dynamics
