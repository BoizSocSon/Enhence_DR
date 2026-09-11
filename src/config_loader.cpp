#include "nav_dynamics/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace nav_dynamics {

RovConfig ConfigLoader::load_from_yaml(const std::string& filepath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("ConfigLoader::load_from_yaml failed to open file '" + filepath + "': " + e.what());
    }

    RovConfig config;

    // --- 1. Các thông số vật rắn của phương tiện ---
    if (root["vehicle"]) {
        auto v_node = root["vehicle"];
        if (v_node["name"]) config.vehicle_name = v_node["name"].as<std::string>();
        if (v_node["mass"]) config.vehicle_params.mass = v_node["mass"].as<double>();
        if (v_node["volume"]) config.vehicle_params.volume = v_node["volume"].as<double>();

        if (v_node["center_of_gravity"] && v_node["center_of_gravity"].IsSequence()) {
            auto cg = v_node["center_of_gravity"];
            config.vehicle_params.r_G = Vector3d(cg[0].as<double>(), cg[1].as<double>(), cg[2].as<double>());
        }

        if (v_node["center_of_buoyancy"] && v_node["center_of_buoyancy"].IsSequence()) {
            auto cb = v_node["center_of_buoyancy"];
            config.vehicle_params.r_B = Vector3d(cb[0].as<double>(), cb[1].as<double>(), cb[2].as<double>());
        }

        if (v_node["inertia_tensor"]) {
            auto I = v_node["inertia_tensor"];
            double Ixx = I["Ixx"] ? I["Ixx"].as<double>() : 0.16;
            double Iyy = I["Iyy"] ? I["Iyy"].as<double>() : 0.35;
            double Izz = I["Izz"] ? I["Izz"].as<double>() : 0.35;
            double Ixy = I["Ixy"] ? I["Ixy"].as<double>() : 0.0;
            double Ixz = I["Ixz"] ? I["Ixz"].as<double>() : 0.0;
            double Iyz = I["Iyz"] ? I["Iyz"].as<double>() : 0.0;

            config.vehicle_params.I_b <<  Ixx, -Ixy, -Ixz,
                                         -Ixy,  Iyy, -Iyz,
                                         -Ixz, -Iyz,  Izz;
        }
    }

    // --- 2. Các thông số thủy động học ---
    if (root["hydrodynamics"]) {
        auto h_node = root["hydrodynamics"];

        if (h_node["added_mass_diagonal"]) {
            auto am = h_node["added_mass_diagonal"];
            double X_udot = am["X_udot"] ? am["X_udot"].as<double>() : 5.5;
            double Y_vdot = am["Y_vdot"] ? am["Y_vdot"].as<double>() : 8.0;
            double Z_wdot = am["Z_wdot"] ? am["Z_wdot"].as<double>() : 14.6;
            double K_pdot = am["K_pdot"] ? am["K_pdot"].as<double>() : 0.05;
            double M_qdot = am["M_qdot"] ? am["M_qdot"].as<double>() : 0.12;
            double N_rdot = am["N_rdot"] ? am["N_rdot"].as<double>() : 0.12;
            config.vehicle_params.set_added_mass_diagonal(X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot);
        }

        if (h_node["linear_damping"]) {
            auto ld = h_node["linear_damping"];
            double Xu = ld["Xu"] ? ld["Xu"].as<double>() : 4.03;
            double Yv = ld["Yv"] ? ld["Yv"].as<double>() : 6.22;
            double Zw = ld["Zw"] ? ld["Zw"].as<double>() : 11.17;
            double Kp = ld["Kp"] ? ld["Kp"].as<double>() : 0.07;
            double Mq = ld["Mq"] ? ld["Mq"].as<double>() : 0.07;
            double Nr = ld["Nr"] ? ld["Nr"].as<double>() : 0.07;
            config.vehicle_params.set_linear_damping_diagonal(Xu, Yv, Zw, Kp, Mq, Nr);
        }

        if (h_node["quadratic_damping"]) {
            auto qd = h_node["quadratic_damping"];
            double Xuu = qd["Xuu"] ? qd["Xuu"].as<double>() : 18.18;
            double Yvv = qd["Yvv"] ? qd["Yvv"].as<double>() : 21.66;
            double Zww = qd["Zww"] ? qd["Zww"].as<double>() : 36.99;
            double Kpp = qd["Kpp"] ? qd["Kpp"].as<double>() : 1.55;
            double Mqq = qd["Mqq"] ? qd["Mqq"].as<double>() : 1.55;
            double Nrr = qd["Nrr"] ? qd["Nrr"].as<double>() : 1.55;
            config.vehicle_params.set_quadratic_damping_diagonal(Xuu, Yvv, Zww, Kpp, Mqq, Nrr);
        }
    }

    // --- 3. Các thông số ArduSub & Động cơ đẩy ---
    std::vector<ThrusterUnit> thrusters;
    if (root["ardusub"]) {
        auto a_node = root["ardusub"];
        if (a_node["frame_type"]) config.ardusub_cfg.frame_type = a_node["frame_type"].as<std::string>();

        if (a_node["pwm_limits"]) {
            auto pwm = a_node["pwm_limits"];
            if (pwm["min"]) config.ardusub_cfg.pwm_min = pwm["min"].as<double>();
            if (pwm["max"]) config.ardusub_cfg.pwm_max = pwm["max"].as<double>();
            if (pwm["trim"]) config.ardusub_cfg.pwm_trim = pwm["trim"].as<double>();
            if (pwm["deadband"]) config.ardusub_cfg.pwm_deadband = pwm["deadband"].as<double>();
        }

        if (a_node["imu_lever_arms"]) {
            for (const auto& kv : a_node["imu_lever_arms"]) {
                std::string imu_name = kv.first.as<std::string>();
                auto v = kv.second;
                if (v.IsSequence() && v.size() == 3) {
                    config.ardusub_cfg.imu_lever_arms[imu_name] = Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
                }
            }
        }

        if (a_node["thrusters"] && a_node["thrusters"].IsSequence()) {
            for (const auto& t_item : a_node["thrusters"]) {
                ThrusterUnit unit;
                if (t_item["name"]) unit.name = t_item["name"].as<std::string>();

                if (t_item["position"] && t_item["position"].IsSequence()) {
                    auto pos = t_item["position"];
                    unit.position_body = Vector3d(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>());
                }

                if (t_item["direction"] && t_item["direction"].IsSequence()) {
                    auto dir = t_item["direction"];
                    unit.direction_body = Vector3d(dir[0].as<double>(), dir[1].as<double>(), dir[2].as<double>());
                }

                unit.neutral_pwm = config.ardusub_cfg.pwm_trim;
                unit.deadband_pwm = config.ardusub_cfg.pwm_deadband;

                thrusters.push_back(unit);
            }
        }
    }

    if (!thrusters.empty()) {
        config.thruster_allocation.set_thrusters(thrusters);
    } else {
        config.thruster_allocation = ThrusterAllocation::make_project_rov_3thruster();
    }

    // --- 4. Cấu hình bậc tự do (DOF) ---
    if (root["dof_config"]) {
        auto dof_node = root["dof_config"];
        if (dof_node["active_dofs"] && dof_node["active_dofs"].IsSequence()) {
            std::vector<std::string> names;
            for (const auto& item : dof_node["active_dofs"]) {
                names.push_back(item.as<std::string>());
            }
            config.dof_transformer = DofTransformer::from_dof_names(names);
        } else if (dof_node["preset"]) {
            std::string preset = dof_node["preset"].as<std::string>();
            if (preset == "FULL_6DOF") {
                config.dof_transformer = DofTransformer::make_6dof();
            } else if (preset == "ROV_3DOF_SURGE_HEAVE_YAW") {
                config.dof_transformer = DofTransformer::make_rov_3dof();
            } else if (preset == "PLANAR_3DOF") {
                config.dof_transformer = DofTransformer::make_planar_3dof();
            } else if (preset == "ROV_4DOF") {
                config.dof_transformer = DofTransformer::make_rov_4dof();
            } else {
                config.dof_transformer = DofTransformer::make_rov_3dof();
            }
        }
    } else {
        config.dof_transformer = DofTransformer::make_rov_3dof();
    }

    // --- 5. Các thông số môi trường NED ---
    if (root["ned_environment"]) {
        auto n_node = root["ned_environment"];
        if (n_node["gravity"]) {
            config.ned_env.gravity = n_node["gravity"].as<double>();
            config.vehicle_params.gravity = config.ned_env.gravity;
        }
        if (n_node["fluid_density"]) {
            config.ned_env.fluid_density = n_node["fluid_density"].as<double>();
            config.vehicle_params.fluid_density = config.ned_env.fluid_density;
        }
        if (n_node["surface_atmospheric_pressure"]) {
            config.ned_env.surface_atmospheric_pressure = n_node["surface_atmospheric_pressure"].as<double>();
        }
        if (n_node["nominal_ocean_current"] && n_node["nominal_ocean_current"].IsSequence()) {
            auto oc = n_node["nominal_ocean_current"];
            config.ned_env.nominal_ocean_current = Vector3d(oc[0].as<double>(), oc[1].as<double>(), oc[2].as<double>());
        }
    }

    return config;
}

void ConfigLoader::save_to_yaml(const RovConfig& config, const std::string& filepath) {
    YAML::Emitter out;
    out << YAML::BeginMap;

    // Thông tin phương tiện
    out << YAML::Key << "vehicle" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << config.vehicle_name;
    out << YAML::Key << "mass" << YAML::Value << config.vehicle_params.mass;
    out << YAML::Key << "volume" << YAML::Value << config.vehicle_params.volume;
    out << YAML::Key << "center_of_gravity" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << config.vehicle_params.r_G.x() << config.vehicle_params.r_G.y() << config.vehicle_params.r_G.z() << YAML::EndSeq;
    out << YAML::Key << "center_of_buoyancy" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << config.vehicle_params.r_B.x() << config.vehicle_params.r_B.y() << config.vehicle_params.r_B.z() << YAML::EndSeq;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        throw std::runtime_error("ConfigLoader::save_to_yaml failed to open '" + filepath + "' for writing.");
    }
    fout << out.c_str();
}

} // namespace nav_dynamics
