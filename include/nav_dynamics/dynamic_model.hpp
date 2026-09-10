#pragma once

#include "nav_dynamics/types.hpp"
#include "nav_dynamics/dof_config.hpp"
#include "nav_dynamics/dof_transformer.hpp"
#include "nav_dynamics/mass_matrix.hpp"
#include "nav_dynamics/coriolis_matrix.hpp"
#include "nav_dynamics/damping_matrix.hpp"
#include "nav_dynamics/restoring_force.hpp"
#include "nav_dynamics/thruster_allocation.hpp"

namespace nav_dynamics {

struct RovConfig;

/**
 * @brief Detailed breakdown of all forces and moments in the equation of motion
 */
struct DynamicBreakdown {
    Vector6d control_wrench = Vector6d::Zero();   ///< Control wrench tau
    Vector6d coriolis_force = Vector6d::Zero();    ///< Coriolis-centripetal force C(nu)*nu
    Vector6d damping_force = Vector6d::Zero();     ///< Hydrodynamic damping force D(nu_r)*nu_r
    Vector6d restoring_force = Vector6d::Zero();   ///< Hydrostatic restoring force g(eta)
    Vector6d net_wrench = Vector6d::Zero();        ///< tau - C - D - g
    Vector6d acceleration_6d = Vector6d::Zero();   ///< nu_dot in body frame
};

/**
 * @brief Main Dynamic Model computation engine integrating all matrix components
 * (M, C, D, g, B) with full 6-DOF support and configurable DOF reduction via DofTransformer.
 */
class DynamicModel {
public:
    DynamicModel();
    explicit DynamicModel(const VehicleParameters& params,
                          const DofTransformer& transformer = DofTransformer::make_6dof(),
                          const ThrusterAllocation& thrusters = ThrusterAllocation::make_project_rov_3thruster());
    
    // Backward compatibility constructor with DofConfig
    explicit DynamicModel(const VehicleParameters& params,
                          const DofConfig& dof_config,
                          const ThrusterAllocation& thrusters = ThrusterAllocation::make_project_rov_3thruster());

    /// Reconfigure active DOFs using dedicated DofTransformer
    void set_dof_transformer(const DofTransformer& transformer);

    /// Get current DOF transformer
    [[nodiscard]] const DofTransformer& dof_transformer() const { return transformer_; }

    // Backward compatibility methods for dof_config
    void set_dof_config(const DofConfig& dof_config);
    [[nodiscard]] const DofConfig& dof_config() const { return dof_config_; }

    /// Update vehicle physical parameters
    void set_parameters(const VehicleParameters& params);

    /// Get vehicle parameters
    [[nodiscard]] const VehicleParameters& parameters() const { return params_; }

    /// Access individual matrix evaluators
    [[nodiscard]] const MassMatrixEvaluator& mass_evaluator() const { return mass_evaluator_; }
    [[nodiscard]] const CoriolisMatrixEvaluator& coriolis_evaluator() const { return coriolis_evaluator_; }
    [[nodiscard]] const DampingMatrixEvaluator& damping_evaluator() const { return damping_evaluator_; }
    [[nodiscard]] const RestoringForceEvaluator& restoring_evaluator() const { return restoring_evaluator_; }
    [[nodiscard]] const ThrusterAllocation& thruster_allocation() const { return thruster_allocation_; }

    /// Forward Dynamics (Full 6D):
    /// Computes body acceleration nu_dot = M^{-1} * (tau - C(nu)*nu - D(nu_r)*nu_r - g(eta))
    [[nodiscard]] Vector6d compute_forward_dynamics_6d(const KinematicState& state,
                                                       const Vector6d& tau,
                                                       const FluidCurrent& current = FluidCurrent()) const;

    /// Forward Dynamics using current DOF configuration:
    /// Solves on reduced n-DOF space if configured (via DofTransformer), then expands to 6D
    [[nodiscard]] Vector6d compute_forward_dynamics(const KinematicState& state,
                                                    const Vector6d& tau,
                                                    const FluidCurrent& current = FluidCurrent()) const;

    /// Compute forward dynamics directly in reduced n-DOF space
    [[nodiscard]] VectorNd compute_forward_dynamics_reduced(const KinematicState& state,
                                                            const VectorNd& tau_r,
                                                            const FluidCurrent& current = FluidCurrent()) const;

    /// Detailed breakdown of all dynamic components
    [[nodiscard]] DynamicBreakdown evaluate_breakdown(const KinematicState& state,
                                                     const Vector6d& tau,
                                                     const FluidCurrent& current = FluidCurrent()) const;

    /// Inverse Dynamics:
    /// Computes required wrench tau = M*nu_dot + C(nu)*nu + D(nu_r)*nu_r + g(eta)
    [[nodiscard]] Vector6d compute_inverse_dynamics(const KinematicState& state,
                                                    const Vector6d& nu_dot,
                                                    const FluidCurrent& current = FluidCurrent()) const;

    /// Numerical integration: 1 step of Euler integration
    void step_euler(KinematicState& state,
                    const Vector6d& tau,
                    double dt,
                    const FluidCurrent& current = FluidCurrent()) const;

    /// Numerical integration: 1 step of 4th-order Runge-Kutta (RK4) integration
    void step_rk4(KinematicState& state,
                  const Vector6d& tau,
                  double dt,
                  const FluidCurrent& current = FluidCurrent()) const;

private:
    VehicleParameters params_;
    DofTransformer transformer_;
    DofConfig dof_config_;
    MassMatrixEvaluator mass_evaluator_;
    CoriolisMatrixEvaluator coriolis_evaluator_;
    DampingMatrixEvaluator damping_evaluator_;
    RestoringForceEvaluator restoring_evaluator_;
    ThrusterAllocation thruster_allocation_;
};

} // namespace nav_dynamics
