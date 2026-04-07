#include "robot_parameter.hpp"

RobotParameter::RobotParameter() {
    string file_name("robot_parameter.cpp");

    path_json_folder_ = __FILE__;
    path_json_folder_.resize(path_json_folder_.length() - file_name.length());
    path_json_folder_ += "../../../../robot_control_config/controller_config/";
}

RobotParameter::~RobotParameter() {

}

bool RobotParameter::init() {
    bool success = true;

    success &= loadByJson_gain();
    success &= loadByJson_motorParam();
    success &= loadByJson_dynamicsParam();

    return success;
}

bool RobotParameter::updateParams() {
    return loadByJson_gain();
}

bool RobotParameter::loadByJson_gain() {
    bool success = true;

    // Read json file
    string path = path_json_folder_ + "ctrl_gain.json";

    ifstream ifs;
    Json::Value data;

    ifs.open(path);
    ifs >> data;

    // JS parameters
    vector<string> str_data_js = {"gain_cst_p", "gain_cst_i", "gain_cst_d",
                                  "gain_csp_p", "gain_csv_p", "gain_csv_i", "gain_csv_b",
                                  "lower_threshold", "upper_threshold",
                                  "lower_threshold_jts", "upper_threshold_jts",
                                  "gravity_coeff", "friction_coeff",
                                  "gravity_coeff_jts", "friction_coeff_jts",
                                  "coulomb", "viscous",
                                  "friction_deadzone", "number_of_elements",
                                  "jts_bias"};

    int dof = data["js_dof"].asInt();

    if (dof != JS_DOF) {
        ROS_LOG_ERROR("Json parameter, JS_DOF size error check the \"ctrl_gain.json\"\n\tJS_DOF in code: %d\n\tJS_DOF in json: %d", JS_DOF, dof);
        success = false;
    }

    for (int i = 0; i < str_data_js.size(); i++) {
        if (data[str_data_js[i]].size() != dof) {
            ROS_LOG_ERROR("File: ctrl_gain.json, check the %s, size must be %d (JS_DOF)", str_data_js[i].c_str(), JS_DOF);
            success = false;
        }
    }

    for (int i = 0; i < dof; i++) {
        if (data["viscous"][i].size() != 5) {
            ROS_LOG_ERROR("File: ctrl_gain.json, check the %s, size must be 5", "viscous friction coeff.");
            success = false;
            break;
        }
    }

    if (success) {
        for (int i = 0; i < JS_DOF; i++) {
            gain_cst_p_[i] = data["gain_cst_p"][i].asDouble();
            gain_cst_i_[i] = data["gain_cst_i"][i].asDouble();
            gain_cst_d_[i] = data["gain_cst_d"][i].asDouble();

            gain_csp_p_[i] = data["gain_csp_p"][i].asDouble();
            gain_csv_p_[i] = data["gain_csv_p"][i].asDouble();
            gain_csv_i_[i] = data["gain_csv_i"][i].asDouble();
            gain_csv_b_[i] = data["gain_csv_b"][i].asDouble();

            col_threshold_lower_    [i] = data["lower_threshold"][i].asDouble();
            col_threshold_upper_    [i] = data["upper_threshold"][i].asDouble();
            col_threshold_lower_jts_[i] = data["lower_threshold_jts"][i].asDouble();
            col_threshold_upper_jts_[i] = data["upper_threshold_jts"][i].asDouble();

            hg_coeff_gravity_     [i] = data["gravity_coeff"][i].asDouble();
            hg_coeff_gravity_jts_ [i] = data["gravity_coeff_jts"][i].asDouble();
            hg_coeff_friction_    [i] = data["friction_coeff"][i].asDouble();
            hg_coeff_friction_jts_[i] = data["friction_coeff_jts"][i].asDouble();

            friction_deadzone_  [i] = data["friction_deadzone"][i].asDouble();
            number_of_elements_[i] = data["number_of_elements"][i].asInt();

            friction_coulomb_[i] = data["coulomb"][i].asDouble();
            friction_viscous_[i][0] = data["viscous"][i][0].asDouble();
            friction_viscous_[i][1] = data["viscous"][i][1].asDouble();
            friction_viscous_[i][2] = data["viscous"][i][2].asDouble();
            friction_viscous_[i][3] = data["viscous"][i][3].asDouble();
            friction_viscous_[i][4] = data["viscous"][i][4].asDouble();

            jts_bias_[i] = data["jts_bias"][i].asDouble();
        }
    }

    // CS parameters
    vector<string> str_data_cs = {"gain_cs_p", "gain_cs_i", "gain_cs_d",
                                  "gain_cs_force_p", "gain_cs_force_i",
                                  "fts_bias", "impedance_deadzone"};

    for (int i = 0; i < str_data_cs.size(); i++) {
        if (data[str_data_cs[i]].size() != CS_DOF) {
            ROS_LOG_ERROR("File: ctrl_gain.json, check the %s, size must be %d (CS_DOF)", str_data_cs[i].c_str(), CS_DOF);
            success = false;
        }
    }

    if (success) {
        for (int i = 0; i < CS_DOF; i++) {
            gain_cs_p_[i] = data["gain_cs_p"][i].asDouble();
            gain_cs_i_[i] = data["gain_cs_i"][i].asDouble();
            gain_cs_d_[i] = data["gain_cs_d"][i].asDouble();

            gain_cs_force_p_[i] = data["gain_cs_force_p"][i].asDouble();
            gain_cs_force_i_[i] = data["gain_cs_force_i"][i].asDouble();

            fts_bias_[i] = data["fts_bias"][i].asDouble();
            imped_deadzone_[i] = data["impedance_deadzone"][i].asDouble();
        }
    }

    // The others
    vector<string> str_data_scalar = {"collision_sensitivity", "hg_damping", "w_threshold", "w_init"};

    for (int i = 0; i < str_data_scalar.size(); i++) {
        if (data[str_data_scalar[i]].isNull()) {
            ROS_LOG_ERROR("File: ctrl_gain.json, check the %s, data is NULL", str_data_scalar[i].c_str());
            success = false;
        }
    }

    if (success) {
        col_sensitivity_  = data["collision_sensitivity"].asDouble();
        hg_coeff_damping_ = data["hg_damping"].asDouble();
        w_threshold_      = data["w_threshold"].asDouble();
        w_init_           = data["w_init"].asDouble();
    }

    return success;
}

bool RobotParameter::loadByJson_motorParam() {
    bool success = true;

    // Read json file
    string path = path_json_folder_ + "motor_param.json";

    ifstream ifs;
    Json::Value data;

    ifs.open(path);
    ifs >> data;

    // JS parameters
    vector<vector<string>> str_data_js = {{"motor_torque_const", "value"},
                                          {"jts_torque_const", "value"},
                                          {"jts_torque_const", "multiply"},
                                          {"jts_torque_const", "divide"},
                                          {"reduction_ratio", "value"},
                                          {"js_torque_limit", "value"},
                                          {"input_encoder_resolution", "value"},
                                          {"input_encoder_resolution_abs", "value"},
                                          {"input_encoder_direction", "value"},
                                          {"output_encoder_resolution_abs", "value"},
                                          {"output_encoder_direction", "value"},
                                          {"js_limit_angle", "value_min"},
                                          {"js_limit_angle", "value_max"},
                                          {"js_init_angle", "value"},
                                          {"rotor_inertia", "wave_generator"},
                                          {"rotor_inertia", "motor_rotor"},
                                          {"rotor_inertia", "etc"},
                                          {"rated_current", "value"},
                                          {"output_encoder_pulse_bias", "value"},
                                          {"input_encoder_pulse_bias", "value"}};

    int dof = data["js_dof"].asInt();

    if (dof != JS_DOF) {
        ROS_LOG_ERROR("Json parameter, JS_DOF size error check the \"motor_param.json\"\n\tJS_DOF in code: %d\n\tJS_DOF in json: %d", JS_DOF, dof);
        success = false;
    }

    for (int i = 0; i < str_data_js.size(); i++) {
        if (data[str_data_js[i][0]][str_data_js[i][1]].size() != dof) {
            ROS_LOG_ERROR("File: motor_param.json, check the %s/%s, size must be %d (JS_DOF)",
                          str_data_js[i][0].c_str(), str_data_js[i][1].c_str(), JS_DOF);
            success = false;
        }
    }

    if (success) {
        for (int i = 0; i < JS_DOF; i++) {
            rated_current_[i] = data["rated_current"]["value"][i].asDouble();

            // Nm/A -> Nm/mA
            motor_torque_const_[i] = data["motor_torque_const"]["value"][i].asDouble() / 1000;
            jts_torque_const_  [i] = data["jts_torque_const"]["value"][i].asDouble()
                                     * data["jts_torque_const"]["multiply"][i].asDouble()
                                     / data["jts_torque_const"]["divide"][i].asDouble();

            reduction_ratio_[i] = data["reduction_ratio"]["value"][i].asDouble();
            torque_limit_   [i] = data["js_torque_limit"]["value"][i].asDouble();

            rotor_pulse_to_deg_     [i] = 360.0 / data["input_encoder_resolution"]["value"][i].asDouble();
            rotor_abs_pulse_to_deg_ [i] = 360.0 / data["input_encoder_resolution_abs"]["value"][i].asDouble();
            rotor_encoder_direction_[i] = data["input_encoder_direction"]["value"][i].asDouble();

            abs_pulse_to_deg_ [i] = 360.0 / data["output_encoder_resolution_abs"]["value"][i].asDouble();
            encoder_direction_[i] = data["output_encoder_direction"]["value"][i].asDouble();

            angle_limit_min_[i] = data["js_limit_angle"]["value_min"][i].asDouble();
            angle_limit_max_[i] = data["js_limit_angle"]["value_max"][i].asDouble();
            init_angle_bias_[i] = data["js_init_angle"]["value"][i].asDouble();

            double inertia_1 = data["rotor_inertia"]["wave_generator"][i].asDouble();
            double inertia_2 = data["rotor_inertia"]["motor_rotor"][i].asDouble();
            double inertia_3 = data["rotor_inertia"]["etc"][i].asDouble();

            // kg*m^2 * 10^4 -> kg*m^2
            rotor_inertia_[i] = (inertia_1 + inertia_2 + inertia_3) * 0.0001
                                * reduction_ratio_[i] * reduction_ratio_[i];

            abs_pulse_bias       [i] = data["output_encoder_pulse_bias"]["value"][i].asDouble();
            rotor_abs_pulse_bias_[i] = data["input_encoder_pulse_bias"]["value"][i].asDouble();
        }
    }

    return success;
}

bool RobotParameter::loadByJson_dynamicsParam() {
    bool success = true;

    // Read json file
    string path = path_json_folder_ + "dynamics_param.json";

    ifstream ifs;
    Json::Value data;

    ifs.open(path);
    ifs >> data;

    // JS parameters
    vector<vector<string>> str_data_js = {{"dh", "alpha"},
                                          {"dh", "a"},
                                          {"dh", "d"},
                                          {"dh", "theta"},
                                          {"link_mass", "value"},
                                          {"center_of_mass", "value"},
                                          {"inertia", "value"}};

    int dof = data["js_dof"].asInt();

    if (dof != JS_DOF) {
        ROS_LOG_ERROR("Json parameter, JS_DOF size error check the \"dynamics_param.json\"\n\tJS_DOF in code: %d\n\tJS_DOF in json: %d", JS_DOF, dof);
        success = false;
    }

    for (int i = 0; i < str_data_js.size(); i++) {
        if (data[str_data_js[i][0]][str_data_js[i][1]].size() != dof) {
            ROS_LOG_ERROR("File: dynamics_param.json, check the %s/%s, size must be %d (JS_DOF)",
                          str_data_js[i][0].c_str(), str_data_js[i][1].c_str(), JS_DOF);
            success = false;
        }
    }

    for (int i = 0; i < JS_DOF; i++) {
        if (data["center_of_mass"]["value"][i].size() != 3) {
            ROS_LOG_ERROR("File: dynamics_param.json, check the [center_of_mass][value]-%d, size must be 3 (COM)", i + 1);
            success = false;
        }

        if (data["inertia"]["value"][i].size() != 9) {
            ROS_LOG_ERROR("File: dynamics_param.json, check the [inertia][value]-%d, size must be 9 (Inertia)", i + 1);
            success = false;
        }
    }

    if (success) {
        for (int i = 0; i < JS_DOF; i++) {
            // alpha, a, d, theta
            dh_param_[i][0] = data["dh"]["alpha"][i].asDouble() * kDeg2Rad; // deg -> rad
            dh_param_[i][1] = data["dh"]["a"]    [i].asDouble() / 1000;     // mm -> m
            dh_param_[i][2] = data["dh"]["d"]    [i].asDouble() / 1000;     // mm -> m
            dh_param_[i][3] = data["dh"]["theta"][i].asDouble() * kDeg2Rad; // deg -> rad

            link_mass_[i] = data["link_mass"]["value"][i].asDouble() / 1000; // g -> kg

            for (int j = 0; j < 3; j++) {
                // mm -> m
                center_mass_[i][j] = data["center_of_mass"]["value"][i][j].asDouble() / 1000;
            }

            for (int j = 0; j < 9; j++) {
                // g*mm^2 -> kg*m^2
                inertia_[i][j] = data["inertia"]["value"][i][j].asDouble() / 1000000000;
            }
        }
    }

    return success;
}

string RobotParameter::getJsonFolderPath() {
    return path_json_folder_;
}
