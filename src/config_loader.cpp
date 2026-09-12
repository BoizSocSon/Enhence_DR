#include "nav_dynamics/config_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace nav_dynamics {

namespace {

bool parse_matrix_nx6(const YAML::Node& node, MatrixNd& out_mat) {
    if (!node) return false;
    if (node.IsSequence()) {
        // Dạng bảng n dòng x 6 cột: [[...], [...], ...]
        if (node.size() >= 1 && node.size() <= 6 && node[0].IsSequence() && node[0].size() == 6) {
            size_t rows = node.size();
            out_mat.resize(rows, 6);
            for (size_t r = 0; r < rows; ++r) {
                if (!node[r].IsSequence() || node[r].size() != 6) {
                    return false;
                }
                for (int c = 0; c < 6; ++c) {
                    out_mat(r, c) = node[r][c].as<double>();
                }
            }
            return true;
        }
        // Dạng mảng phẳng (bội số của 6, ví dụ 18 phần tử cho 3x6)
        if (node.size() >= 6 && node.size() <= 36 && node.size() % 6 == 0) {
            size_t rows = node.size() / 6;
            std::vector<double> vals = node.as<std::vector<double>>();
            out_mat.resize(rows, 6);
            for (size_t r = 0; r < rows; ++r) {
                for (size_t c = 0; c < 6; ++c) {
                    out_mat(r, c) = vals[r * 6 + c];
                }
            }
            return true;
        }
    }
    return false;
}

bool parse_matrix6d(const YAML::Node& node, Matrix6d& out_mat) {
    if (!node) return false;
    if (node.IsSequence()) {
        // Dạng bảng 6 dòng x 6 cột: [[...], [...], ...]
        if (node.size() == 6 && node[0].IsSequence() && node[0].size() == 6) {
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    out_mat(r, c) = node[r][c].as<double>();
                }
            }
            return true;
        }
        // Dạng mảng phẳng 36 phần tử liên tiếp
        if (node.size() == 36) {
            std::vector<double> vals = node.as<std::vector<double>>();
            out_mat = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(vals.data());
            return true;
        }
    }
    return false;
}

void parse_vehicle_parameters_from_node(const YAML::Node& root, VehicleParameters& params) {
    // 1. Các thông số vật rắn
    if (root["vehicle"]) {
        auto v_node = root["vehicle"];
        if (v_node["mass"]) params.mass = v_node["mass"].as<double>();
        if (v_node["volume"]) params.volume = v_node["volume"].as<double>();
        if (v_node["fluid_density"]) params.fluid_density = v_node["fluid_density"].as<double>();
        if (v_node["gravity"]) params.gravity = v_node["gravity"].as<double>();

        if (v_node["center_of_gravity"]) {
            auto cg = v_node["center_of_gravity"];
            if (cg.IsSequence() && cg.size() >= 3) {
                params.r_G = Vector3d(cg[0].as<double>(), cg[1].as<double>(), cg[2].as<double>());
            } else if (cg.IsMap()) {
                double x = cg["x_G"] ? cg["x_G"].as<double>() : (cg["x"] ? cg["x"].as<double>() : 0.0);
                double y = cg["y_G"] ? cg["y_G"].as<double>() : (cg["y"] ? cg["y"].as<double>() : 0.0);
                double z = cg["z_G"] ? cg["z_G"].as<double>() : (cg["z"] ? cg["z"].as<double>() : 0.0);
                params.r_G = Vector3d(x, y, z);
            }
        }

        if (v_node["center_of_buoyancy"]) {
            auto cb = v_node["center_of_buoyancy"];
            if (cb.IsSequence() && cb.size() >= 3) {
                params.r_B = Vector3d(cb[0].as<double>(), cb[1].as<double>(), cb[2].as<double>());
            } else if (cb.IsMap()) {
                double x = cb["x_B"] ? cb["x_B"].as<double>() : (cb["x"] ? cb["x"].as<double>() : 0.0);
                double y = cb["y_B"] ? cb["y_B"].as<double>() : (cb["y"] ? cb["y"].as<double>() : 0.0);
                double z = cb["z_B"] ? cb["z_B"].as<double>() : (cb["z"] ? cb["z"].as<double>() : 0.0);
                params.r_B = Vector3d(x, y, z);
            }
        }

        if (v_node["inertia_tensor"]) {
            auto I = v_node["inertia_tensor"];
            if (I.IsSequence() && I.size() == 3 && I[0].IsSequence() && I[0].size() == 3) {
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        params.I_b(r, c) = I[r][c].as<double>();
                    }
                }
            } else if (I.IsMap()) {
                double Ixx = I["Ixx"] ? I["Ixx"].as<double>() : 0.16;
                double Iyy = I["Iyy"] ? I["Iyy"].as<double>() : 0.35;
                double Izz = I["Izz"] ? I["Izz"].as<double>() : 0.35;
                double Ixy = I["Ixy"] ? I["Ixy"].as<double>() : 0.0;
                double Ixz = I["Ixz"] ? I["Ixz"].as<double>() : 0.0;
                double Iyz = I["Iyz"] ? I["Iyz"].as<double>() : 0.0;

                params.I_b <<  Ixx, -Ixy, -Ixz,
                              -Ixy,  Iyy, -Iyz,
                              -Ixz, -Iyz,  Izz;
            }
        }
    }

    // 2. Các thông số thủy động học 6x6 (hỗ trợ cả trục chính và coupling)
    if (root["hydrodynamics"]) {
        auto h_node = root["hydrodynamics"];

        // Added Mass (M_A)
        if (h_node["added_mass_matrix"] && parse_matrix6d(h_node["added_mass_matrix"], params.M_A)) {
            // Đã nạp thành công ma trận 6x6 đầy đủ
        } else if (h_node["added_mass_diagonal"]) {
            auto am = h_node["added_mass_diagonal"];
            if (am.IsSequence() && am.size() == 6) {
                params.set_added_mass_diagonal(
                    am[0].as<double>(), am[1].as<double>(), am[2].as<double>(),
                    am[3].as<double>(), am[4].as<double>(), am[5].as<double>()
                );
            } else if (am.IsMap()) {
                double X_udot = am["X_udot"] ? am["X_udot"].as<double>() : 5.5;
                double Y_vdot = am["Y_vdot"] ? am["Y_vdot"].as<double>() : 8.0;
                double Z_wdot = am["Z_wdot"] ? am["Z_wdot"].as<double>() : 14.6;
                double K_pdot = am["K_pdot"] ? am["K_pdot"].as<double>() : 0.05;
                double M_qdot = am["M_qdot"] ? am["M_qdot"].as<double>() : 0.12;
                double N_rdot = am["N_rdot"] ? am["N_rdot"].as<double>() : 0.12;
                params.set_added_mass_diagonal(X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot);
            }
        }

        // Linear Damping (D_l)
        if (h_node["linear_damping_matrix"] && parse_matrix6d(h_node["linear_damping_matrix"], params.D_l)) {
            // Đã nạp thành công ma trận 6x6 đầy đủ
        } else if (h_node["linear_damping"]) {
            auto ld = h_node["linear_damping"];
            if (ld.IsSequence() && ld.size() == 6) {
                params.set_linear_damping_diagonal(
                    ld[0].as<double>(), ld[1].as<double>(), ld[2].as<double>(),
                    ld[3].as<double>(), ld[4].as<double>(), ld[5].as<double>()
                );
            } else if (ld.IsMap()) {
                double Xu = ld["Xu"] ? ld["Xu"].as<double>() : 4.03;
                double Yv = ld["Yv"] ? ld["Yv"].as<double>() : 6.22;
                double Zw = ld["Zw"] ? ld["Zw"].as<double>() : 11.17;
                double Kp = ld["Kp"] ? ld["Kp"].as<double>() : 0.07;
                double Mq = ld["Mq"] ? ld["Mq"].as<double>() : 0.07;
                double Nr = ld["Nr"] ? ld["Nr"].as<double>() : 0.07;
                params.set_linear_damping_diagonal(Xu, Yv, Zw, Kp, Mq, Nr);
            }
        }

        // Quadratic Damping (D_q)
        if (h_node["quadratic_damping_matrix"] && parse_matrix6d(h_node["quadratic_damping_matrix"], params.D_q)) {
            // Đã nạp thành công ma trận 6x6 đầy đủ
        } else if (h_node["quadratic_damping"]) {
            auto qd = h_node["quadratic_damping"];
            if (qd.IsSequence() && qd.size() == 6) {
                params.set_quadratic_damping_diagonal(
                    qd[0].as<double>(), qd[1].as<double>(), qd[2].as<double>(),
                    qd[3].as<double>(), qd[4].as<double>(), qd[5].as<double>()
                );
            } else if (qd.IsMap()) {
                double Xuu = qd["Xuu"] ? qd["Xuu"].as<double>() : 18.18;
                double Yvv = qd["Yvv"] ? qd["Yvv"].as<double>() : 21.66;
                double Zww = qd["Zww"] ? qd["Zww"].as<double>() : 36.99;
                double Kpp = qd["Kpp"] ? qd["Kpp"].as<double>() : 1.55;
                double Mqq = qd["Mqq"] ? qd["Mqq"].as<double>() : 1.55;
                double Nrr = qd["Nrr"] ? qd["Nrr"].as<double>() : 1.55;
                params.set_quadratic_damping_diagonal(Xuu, Yvv, Zww, Kpp, Mqq, Nrr);
            }
        }
    }
}

DofTransformer parse_dof_transformer_from_node(const YAML::Node& root) {
    if (root["dof_config"]) {
        auto dof_node = root["dof_config"];

        std::vector<DofIndex> active_dofs;
        std::vector<std::string> active_names;
        if (dof_node["active_dofs"] && dof_node["active_dofs"].IsSequence()) {
            for (const auto& item : dof_node["active_dofs"]) {
                std::string name = item.as<std::string>();
                active_names.push_back(name);
                std::string upper_name = name;
                std::transform(upper_name.begin(), upper_name.end(), upper_name.begin(), ::toupper);
                if (upper_name == "SURGE" || upper_name == "U") active_dofs.push_back(DofIndex::SURGE);
                else if (upper_name == "SWAY" || upper_name == "V") active_dofs.push_back(DofIndex::SWAY);
                else if (upper_name == "HEAVE" || upper_name == "W") active_dofs.push_back(DofIndex::HEAVE);
                else if (upper_name == "ROLL" || upper_name == "P") active_dofs.push_back(DofIndex::ROLL);
                else if (upper_name == "PITCH" || upper_name == "Q") active_dofs.push_back(DofIndex::PITCH);
                else if (upper_name == "YAW" || upper_name == "R") active_dofs.push_back(DofIndex::YAW);
            }
        }

        std::string preset_str;
        if (dof_node["preset"]) {
            preset_str = dof_node["preset"].as<std::string>();
        }

        // Chọn node ma trận phù hợp theo preset (nếu có) hoặc các khóa phổ biến
        YAML::Node mat_node;
        if (preset_str == "ROV_4DOF_CONFIG_1" || preset_str == "ROV_4DOF") {
            if (dof_node["transform_matrix_4dof_1"]) mat_node = dof_node["transform_matrix_4dof_1"];
            else if (dof_node["transform_matrix_4dof"]) mat_node = dof_node["transform_matrix_4dof"];
        } else if (preset_str == "ROV_3DOF_CONFIG_1" || preset_str == "ROV_3DOF_SURGE_HEAVE_YAW") {
            if (dof_node["transform_matrix_3dof_1"]) mat_node = dof_node["transform_matrix_3dof_1"];
            else if (dof_node["transform_matrix_3dof"]) mat_node = dof_node["transform_matrix_3dof"];
            else if (dof_node["transform_matrix_3DOF_"]) mat_node = dof_node["transform_matrix_3DOF_"];
        } else if (preset_str == "ROV_6DOF_FULL" || preset_str == "FULL_6DOF") {
            if (dof_node["transform_matrix_6dof"]) mat_node = dof_node["transform_matrix_6dof"];
        }

        // Nếu chưa tìm thấy theo preset, thử tìm các khóa chung
        if (!mat_node) {
            if (dof_node["transform_matrix_3dof_1"]) {
                mat_node = dof_node["transform_matrix_3dof_1"];
            } else if (dof_node["transform_matrix_3DOF_"]) {
                mat_node = dof_node["transform_matrix_3DOF_"];
            } else if (dof_node["transform_matrix_4dof_1"]) {
                mat_node = dof_node["transform_matrix_4dof_1"];
            } else if (dof_node["transform_matrix"]) {
                mat_node = dof_node["transform_matrix"];
            } else if (dof_node["transform_matrix_3dof"]) {
                mat_node = dof_node["transform_matrix_3dof"];
            } else if (dof_node["transform_matrix_4dof"]) {
                mat_node = dof_node["transform_matrix_4dof"];
            } else if (dof_node["transformation_matrix"]) {
                mat_node = dof_node["transformation_matrix"];
            } else if (dof_node["projection_matrix"]) {
                mat_node = dof_node["projection_matrix"];
            }
        }

        MatrixNd T_mat;
        if (mat_node && parse_matrix_nx6(mat_node, T_mat)) {
            if (!active_dofs.empty() && static_cast<size_t>(T_mat.rows()) == active_dofs.size()) {
                return DofTransformer(T_mat, active_dofs);
            }
            return DofTransformer(T_mat);
        }

        if (!active_names.empty()) {
            return DofTransformer::from_dof_names(active_names);
        }

        if (!preset_str.empty()) {
            if (preset_str == "ROV_6DOF_FULL" || preset_str == "FULL_6DOF") {
                return DofTransformer::make_6dof();
            } else if (preset_str == "ROV_4DOF_CONFIG_1" || preset_str == "ROV_4DOF") {
                return DofTransformer::make_rov_4dof();
            } else if (preset_str == "ROV_3DOF_CONFIG_1" || preset_str == "ROV_3DOF_SURGE_HEAVE_YAW") {
                return DofTransformer::make_rov_3dof();
            } else if (preset_str == "PLANAR_3DOF" || preset_str == "PLANAR_3DOF_SURGE_SWAY_YAW") {
                return DofTransformer::make_planar_3dof();
            }
        }
    }
    return DofTransformer::make_rov_3dof();
}

} // anonymous namespace

RovConfig ConfigLoader::load_from_yaml(const std::string& filepath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("ConfigLoader::load_from_yaml failed to open file '" + filepath + "': " + e.what());
    }

    RovConfig config;

    // --- 1. Tên phương tiện ---
    if (root["vehicle"] && root["vehicle"]["name"]) {
        config.vehicle_name = root["vehicle"]["name"].as<std::string>();
    }

    // --- 2. Nạp toàn bộ thông số vật lý & ma trận 6x6 (M_A, D_l, D_q, I_b) trong 1 lần gọi ---
    parse_vehicle_parameters_from_node(root, config.vehicle_params);

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
    config.dof_transformer = parse_dof_transformer_from_node(root);
    config.dof_config = config.dof_transformer.to_dof_config();

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

VehicleParameters ConfigLoader::load_vehicle_parameters(const std::string& filepath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("ConfigLoader::load_vehicle_parameters failed to open file '" + filepath + "': " + e.what());
    }

    VehicleParameters params;
    parse_vehicle_parameters_from_node(root, params);

    // Đồng bộ gia tốc trọng trường và khối lượng riêng nếu có khai báo trong ned_environment
    if (root["ned_environment"]) {
        auto n_node = root["ned_environment"];
        if (n_node["gravity"]) params.gravity = n_node["gravity"].as<double>();
        if (n_node["fluid_density"]) params.fluid_density = n_node["fluid_density"].as<double>();
    }

    return params;
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

DofTransformer ConfigLoader::load_dof_transformer(const std::string& filepath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(filepath);
    } catch (const std::exception& e) {
        throw std::runtime_error("ConfigLoader::load_dof_transformer failed to open file '" + filepath + "': " + e.what());
    }
    return parse_dof_transformer_from_node(root);
}

DofConfig ConfigLoader::load_dof_config(const std::string& filepath) {
    return load_dof_transformer(filepath).to_dof_config();
}

} // namespace nav_dynamics
