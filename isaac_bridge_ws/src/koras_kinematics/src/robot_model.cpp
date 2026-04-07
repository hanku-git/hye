#include "robot_model.hpp"

RobotModel::RobotModel(void) : RobotModelDynamics() {
    //	const int JS_DOF
    for (int i = 0; i < JS_DOF; i++) {
        q_[i] = 0.0;
        q_prev_[i] = 0.0;
        qd_prev_[i] = 0.0;
        coulomb_friction_direction_[i] = 0;
        coulomb_friction_direction_hg_[i] = 0;
    }

    residual_torque_vec_ = new ModelMatrix(JS_DOF, 1);
    residual_torque_current_vec_ = new ModelMatrix(JS_DOF, 1);
    friction_torque_observer_vec_ = new ModelMatrix(JS_DOF, 1);

    residual_torque_integral_ = new ModelMatrix(JS_DOF, 1);
    residual_torque_current_integral_ = new ModelMatrix(JS_DOF, 1);
    friction_torque_observer_integral_ = new ModelMatrix(JS_DOF, 1);

    observer_gain_ = new ModelMatrix(JS_DOF, JS_DOF);

    inertia_mat_  = new ModelMatrix(JS_DOF, JS_DOF);
    coriolis_mat_ = new ModelMatrix(JS_DOF, JS_DOF);
    gravity_vec_  = new ModelMatrix(JS_DOF, 1);
    inertia_rotor_mat_ = new ModelMatrix(JS_DOF, JS_DOF);
#if CBM_FLAG
    cbm_vec_ = new ModelMatrix(JS_DOF, 1);
#endif

    for (int i = 0; i < 6; i++) {
        tcp_[i] = 0.0;
    }
}

RobotModel::~RobotModel(void) {
}

void RobotModel::setDH(double dh[JS_DOF][4]) {
    for (int i = 0; i < JS_DOF; i++) {
        for (int j = 0; j < 4; j++) {
            dh_[i][j] = dh[i][j];
        }
    }
}

void RobotModel::getDH(double dh[JS_DOF][4]) {
    for (int i = 0; i < JS_DOF; i++) {
        for (int j = 0; j < 4; j++) {
            dh[i][j] = dh_[i][j];
        }
    }
}

void RobotModel::getq(double q[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        q[i] = q_[i] * kRad2Deg;
    }
}

void RobotModel::getqd(double qd[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        qd[i] = qd_[i] * kRad2Deg;
    }
}

void RobotModel::setq(double q[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        q_[i] = q[i] * kDeg2Rad;
    }
}

void RobotModel::setqd(double qd[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        qd_[i] = qd[i] * kDeg2Rad;
    }
}

void RobotModel::setTcp(double tcp[CS_DOF]) {
    for (int i = 0; i < CS_DOF; i++) {
        tcp_[i] = tcp[i];
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// Inverse Kinematics //////////////////////////////////////////

bool RobotModel::inverseKinematics(vector<double> x_target, vector<double> q_current, vector<double> &q_target) {
    for (int i = 0; i < 3; i++) {
        x_target[i + 3] *= kDeg2Rad;
    }

    for (int i = 0; i < JS_DOF; i++) {
        q_current[i] *= kDeg2Rad;
    }

    LMMethod jaco_method = LEVEN;
    int iter = 0;
    double lambda = kInverseKineLambda;

    static ModelMatrix pose_target(6, 1);
    static ModelMatrix tr_target(4, 4, true);
    static ModelMatrix tr_current(4, 4, true);
    static ModelMatrix q(JS_DOF, 1);
    static ModelMatrix err(6, 1);

    ModelMatrix jacobian(6, 6);
    ModelMatrix JTJ(6, 6);

    for (int i = 0; i < 6; i++) {
        pose_target.element_[i] = x_target[i];
    }

    tr_target = pose2tr(pose_target);

    for (int i = 0; i < JS_DOF; i++) {
        q.element_[i] = q_current[i];
    }

    tr_current = fwdKine_q2Tr(q);

    tr_current = tr_current * makeTcpMatrix().inverse();
    tr_target = tr_target * makeTcpMatrix().inverse();

    err = tr2delta(tr_current, tr_target);

    double err_norm_prev = norm(err);

    if (err_norm_prev <= kInverseKineTol) {
        for (int i = 0; i < JS_DOF; i++) {
            q_target[i] = q.element_[i] * kRad2Deg;
        }
        return true;
    }

    if (JS_DOF != 6 && JS_DOF != 7) {
        for (int i = 0; i < JS_DOF; i++) {
            q_target[i] = q_current[i] * kRad2Deg;
        }
        return false;
    } else {
        while (true) {
            jacobian = calJacobian(q);

            JTJ = jacobian.transpose() * jacobian;

            if (jaco_method == LEVEN) {
                // Levenberg metod
                for (int i = 0; i < 6; i++) {
                    JTJ.element_[6 * i + i] += lambda;
                }
            } else if (jaco_method == LEVEN_MARQ) {
                // Levenberg - Marquadt method
                for (int i = 0; i < 6; i++) {
                    JTJ.element_[6 * i + i] += lambda * JTJ.element_[6 * i + i];
                }
            }

            ModelMatrix dq = JTJ.inverse() * jacobian.transpose() * err;

            if (JS_DOF == 6) {
                q = q + dq;
            }
            else if (JS_DOF == 7) {
                for (int i = 0; i < JS_DOF; i++) {
                    if (i == 2) {
                        q.element_[i] = q_current[i];
                    } else if (i > 2) {
                        q.element_[i] += dq.element_[i - 1];
                    } else {
                        q.element_[i] += dq.element_[i];
                    }
                }
            }

            tr_current = fwdKine_q2Tr(q);
            tr_current = tr_current * makeTcpMatrix().inverse();
            err = tr2delta(tr_current, tr_target);

            double err_norm = norm(err);

            if (err_norm <= kInverseKineTol) {
                for (int i = 0; i < JS_DOF; i++) {
                    q_target[i] = q.element_[i] * kRad2Deg;
                }
                return true;
            } else if (err_norm < err_norm_prev) {
                err_norm_prev = err_norm;
                lambda /= 2;
            } else {
                err_norm_prev = err_norm;
                lambda *= 2;
            }

            if (iter++ >= kInverseKineIterLimit) {
                ROS_LOG_ERROR("Inverse kinematics error: Can't converge the value");
                for (int i = 0; i < JS_DOF; i++) {
                    q_target[i] = q_current[i] * kRad2Deg;
                }
                return false;
            }

            if (iter >= 7) {
                ROS_LOG_WARN("Inverse kinematics: iteration is %d", iter);
            }
        }
    }
}

bool RobotModel::inverseKinematicsTemp(vector<double> x_target, vector<double> q_current, vector<double> &q_target, int &iter, double &err_norm_output) {
    for (int i = 0; i < 3; i++) {
        x_target[i + 3] *= kDeg2Rad;
    }

    for (int i = 0; i < JS_DOF; i++) {
        q_current[i] *= kDeg2Rad;
    }

    LMMethod jaco_method = LEVEN;
    // int iter = 0;
    double lambda = kInverseKineLambda;

    static ModelMatrix pose_target(6, 1);
    static ModelMatrix tr_target(4, 4, true);
    static ModelMatrix tr_current(4, 4, true);
    static ModelMatrix q(JS_DOF, 1);
    static ModelMatrix err(6, 1);

    ModelMatrix jacobian(6, 6);
    ModelMatrix JTJ(6, 6);

    for (int i = 0; i < 6; i++) {
        pose_target.element_[i] = x_target[i];
    }

    tr_target = pose2tr(pose_target);

    for (int i = 0; i < JS_DOF; i++) {
        q.element_[i] = q_current[i];
    }

    tr_current = fwdKine_q2Tr(q);

    tr_current = tr_current * makeTcpMatrix().inverse();
    tr_target = tr_target * makeTcpMatrix().inverse();

    err = tr2delta(tr_current, tr_target);

    double err_norm_prev = norm(err);
    err_norm_output = err_norm_prev;

    if (err_norm_prev <= kInverseKineTol) {
        for (int i = 0; i < JS_DOF; i++) {
            q_target[i] = q.element_[i] * kRad2Deg;
        }
        return true;
    }

    if (JS_DOF != 6 && JS_DOF != 7) {
        for (int i = 0; i < JS_DOF; i++) {
            q_target[i] = q_current[i] * kRad2Deg;
        }
        return false;
    } else {
        while (true) {
            jacobian = calJacobian(q);

            JTJ = jacobian.transpose() * jacobian;

            if (jaco_method == LEVEN) {
                // Levenberg metod
                for (int i = 0; i < 6; i++) {
                    JTJ.element_[6 * i + i] += lambda;
                }
            } else if (jaco_method == LEVEN_MARQ) {
                // Levenberg - Marquadt method
                for (int i = 0; i < 6; i++) {
                    JTJ.element_[6 * i + i] += lambda * JTJ.element_[6 * i + i];
                }
            }

            ModelMatrix dq = JTJ.inverse() * jacobian.transpose() * err;

            if (JS_DOF == 6) {
                q = q + dq;
            }
            else if (JS_DOF == 7) {
                for (int i = 0; i < JS_DOF; i++) {
                    if (i == 2) {
                        q.element_[i] = q_current[i];
                    } else if (i > 2) {
                        q.element_[i] += dq.element_[i - 1];
                    } else {
                        q.element_[i] += dq.element_[i];
                    }
                }
            }

            tr_current = fwdKine_q2Tr(q);
            tr_current = tr_current * makeTcpMatrix().inverse();
            err = tr2delta(tr_current, tr_target);

            double err_norm = norm(err);
            err_norm_output = err_norm;

            if (err_norm <= kInverseKineTol) {
                for (int i = 0; i < JS_DOF; i++) {
                    q_target[i] = q.element_[i] * kRad2Deg;
                }
                return true;
            } else if (err_norm < err_norm_prev) {
                err_norm_prev = err_norm;
                lambda /= 2;
            } else {
                err_norm_prev = err_norm;
                lambda *= 2;
            }

            if (iter++ >= kInverseKineIterLimit) {
                ROS_LOG_ERROR("Inverse kinematics error: Can't converge the value");
                for (int i = 0; i < JS_DOF; i++) {
                    q_target[i] = q_current[i] * kRad2Deg;
                }
                return false;
            }

            if (iter >= 7) {
                ROS_LOG_WARN("Inverse kinematics: iteration is %d", iter);
            }
        }
    }
}

ModelMatrix RobotModel::fwdKine_q2Tr(ModelMatrix q) {
    ModelMatrix tr = transformationMatrix(0, q);
    for (int i = 1; i < JS_DOF; i++) tr = tr * transformationMatrix(i, q);
    tr = tr * makeTcpMatrix();

    return tr;
}

ModelMatrix RobotModel::tr2delta(ModelMatrix tr_old, ModelMatrix tr_target) {
    ModelMatrix TD(4, 4, true);
    ModelMatrix delta(6, 1);

    TD = invTrMat(tr_old) * tr_target;

    delta.element_[0] = TD.element_[3];
    delta.element_[1] = TD.element_[7];
    delta.element_[2] = TD.element_[11];

    delta.element_[3] = 0.5 * (TD.element_[9] - TD.element_[6]);
    delta.element_[4] = 0.5 * (TD.element_[2] - TD.element_[8]);
    delta.element_[5] = 0.5 * (TD.element_[4] - TD.element_[1]);

    return delta;
}

ModelMatrix RobotModel::pose2tr(ModelMatrix xyzrpy) {
    double rpy[3];
    double rot[3][3];

    ModelMatrix tr(4, 4, true);

    for (int i = 0; i < 3; i++) rpy[i] = xyzrpy.element_[i + 3];
    rpy2rot(rpy, rot, true);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) tr.element_[4 * i + j] = rot[i][j];
    }
    for (int i = 0; i < 3; i++) tr.element_[4 * i + 3] = xyzrpy.element_[i];

    tr.element_[12] = 0;
    tr.element_[13] = 0;
    tr.element_[14] = 0;
    tr.element_[15] = 1;

    return tr;
}

ModelMatrix RobotModel::tr2pose(ModelMatrix trans_mat) {
    double rot[3][3];
    double rpy[3];

    for (std::size_t i = 0; i < 3; i++)
        for (std::size_t  j = 0; j < 3; j++) rot[i][j] = trans_mat.element_[4*i + j];

    rot2rpy(rot, rpy);

    ModelMatrix pose(6, 1);

    for (std::size_t i = 0; i < 3; i++) pose.element_[i] = trans_mat.element_[4 * i + 3];
    for (std::size_t i = 3; i < 6; i++) pose.element_[i] = rpy[i - 3];

    return pose;
}

ModelMatrix RobotModel::invTrMat(const ModelMatrix &tr) {
    ModelMatrix result(4, 4, true);
    ModelMatrix rot_transpose(3, 3);
    ModelMatrix xyz(3, 1);

    // TODO: 복잡한 과정 생략
    for (std::size_t i = 0; i < 3; i++) rot_transpose.element_[i] = tr.element_[i];
    for (std::size_t i = 3; i < 6; i++) rot_transpose.element_[i] = tr.element_[i + 1];
    for (std::size_t i = 6; i < 9; i++) rot_transpose.element_[i] = tr.element_[i + 2];
    rot_transpose = rot_transpose.transpose();

    for (std::size_t i = 0; i < 3; i++) xyz.element_[i] = (-1)*tr.element_[4*i + 3];
    xyz = rot_transpose * xyz;

    for (std::size_t i = 0; i < 3; i++) result.element_[i]       = rot_transpose.element_[i];
    for (std::size_t i = 3; i < 6; i++) result.element_[i + 1]   = rot_transpose.element_[i];
    for (std::size_t i = 6; i < 9; i++) result.element_[i + 2]   = rot_transpose.element_[i];
    for (std::size_t i = 0; i < 3; i++) result.element_[4*i + 3] = xyz.element_[i];

    result.element_[12] = 0;	result.element_[13] = 0;
    result.element_[14] = 0;	result.element_[15] = 1;

    return result;
}

double RobotModel::norm(ModelMatrix xyzrpyMat) {
    double a = 0;
    for (int i = 0; i < 6; i++) a += xyzrpyMat.element_[i] * xyzrpyMat.element_[i];
    double norm = sqrt(a);

    return norm;
}

ModelMatrix RobotModel::calJacobian(ModelMatrix q) {
    ModelMatrix J(6, 6);
    ModelMatrix U(4, 4, true);

    for (int i = 0; i < 4; i++) {
        U.element_[4 * i + i] = 1;
    }

    if (JS_DOF == 6) {
        for (int i = JS_DOF; i > 0; i--) {
            switch (i) {
                case 6: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 5: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 4: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 3: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 2: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 1: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
            }

            double d[3] = {-U.element_[0] * U.element_[7] + U.element_[4] * U.element_[3],
                           -U.element_[1] * U.element_[7] + U.element_[5] * U.element_[3],
                           -U.element_[2] * U.element_[7] + U.element_[6] * U.element_[3]};

            for (int j = 0; j < 3; j++) {
                J.element_[6 * j + (i - 1)] = d[j];
                J.element_[6 * (j + 3) + (i - 1)] = U.element_[8 + j];
            }
        }
    } else if (JS_DOF == 7) {
        for (int i = JS_DOF; i > 1; i--) {
            switch (i) {
                case 7: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 6: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 5: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 4: {
                    U = transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 3: {
                    U = transformationMatrix(i - 2, q) * transformationMatrix(i - 1, q) * U;
                    break;
                }
                case 2: {
                    U = transformationMatrix(i - 2, q) * U;
                    break;
                }
            }

            double d[3] = {-U.element_[0] * U.element_[7] + U.element_[4] * U.element_[3],
                           -U.element_[1] * U.element_[7] + U.element_[5] * U.element_[3],
                           -U.element_[2] * U.element_[7] + U.element_[6] * U.element_[3]};

            for (int j = 0; j < 3; j++) {
                J.element_[6 * j + (i - 2)] = d[j];
                J.element_[6 * (j + 3) + (i - 2)] = U.element_[8 + j];
            }
        }
    }

    return J;
}

ModelMatrix RobotModel::transformationMatrix(int joint_num, ModelMatrix q) {
    double q_tmp = q.element_[joint_num];
    return transformationMatrix(joint_num, q_tmp);
}

/////////////////////////////////////// Inverse Kinematics //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void RobotModel::fwdKine(double Output_x[6]) {
    ModelMatrix mat = transformationMatrix(0);
    for (int i = 1; i < JS_DOF; i++) mat = mat * transformationMatrix(i);

    ModelMatrix tcpMat = makeTcpMatrix();

    mat = mat * tcpMat;

    Output_x[0] = mat.element_[0 * 4 + 3];
    Output_x[1] = mat.element_[1 * 4 + 3];
    Output_x[2] = mat.element_[2 * 4 + 3];

    double r11 = mat.element_[0 * 4 + 0];
    double r21 = mat.element_[1 * 4 + 0];
    double r31 = mat.element_[2 * 4 + 0];
    double r32 = mat.element_[2 * 4 + 1];
    double r33 = mat.element_[2 * 4 + 2];

    // Roll-Pitch-Yaw - Euler ZYX
    Output_x[3] = atan2(r32, r33) * kRad2Deg;
    Output_x[4] = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) * kRad2Deg;
    Output_x[5] = atan2(r21, r11) * kRad2Deg;
}

void RobotModel::fwdKine(int joint, double Output_x[6]) {
    ModelMatrix mat = transformationMatrix(0);
    for (int i = 1; i < joint; i++) mat = mat * transformationMatrix(i);

    Output_x[0] = mat.element_[0 * 4 + 3];
    Output_x[1] = mat.element_[1 * 4 + 3];
    Output_x[2] = mat.element_[2 * 4 + 3];

    double r11 = mat.element_[0 * 4 + 0];
    double r21 = mat.element_[1 * 4 + 0];
    double r31 = mat.element_[2 * 4 + 0];
    double r32 = mat.element_[2 * 4 + 1];
    double r33 = mat.element_[2 * 4 + 2];

    // Roll-Pitch-Yaw - Euler ZYX
    Output_x[3] = atan2(r32, r33) * kRad2Deg;
    Output_x[4] = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) * kRad2Deg;
    Output_x[5] = atan2(r21, r11) * kRad2Deg;
}

void RobotModel::fwdKine(double q[6], double Output_x[6]) {
    ModelMatrix mat = transformationMatrix(0, q[0]);
    for (int i = 1; i < JS_DOF; i++) mat = mat * transformationMatrix(i, q[i]);

    Output_x[0] = mat.element_[0 * 4 + 3];
    Output_x[1] = mat.element_[1 * 4 + 3];
    Output_x[2] = mat.element_[2 * 4 + 3];

    double r11 = mat.element_[0 * 4 + 0];
    double r21 = mat.element_[1 * 4 + 0];
    double r31 = mat.element_[2 * 4 + 0];
    double r32 = mat.element_[2 * 4 + 1];
    double r33 = mat.element_[2 * 4 + 2];

    // Roll-Pitch-Yaw - Euler ZYX
    Output_x[3] = atan2(r32, r33) * kRad2Deg;
    Output_x[4] = atan2(-r31, sqrt(r32 * r32 + r33 * r33)) * kRad2Deg;
    Output_x[5] = atan2(r21, r11) * kRad2Deg;
}

void RobotModel::fwdKineZYZ(double Output_x[6]) {
    ModelMatrix mat = transformationMatrix(0);
    for (int i = 1; i < JS_DOF; i++) mat = mat * transformationMatrix(i);

    ModelMatrix tcpMat = makeTcpMatrix();

    mat = mat * tcpMat;

    Output_x[0] = mat.element_[0 * 4 + 3];
    Output_x[1] = mat.element_[1 * 4 + 3];
    Output_x[2] = mat.element_[2 * 4 + 3];

    double r13 = mat.element_[0 * 4 + 2];
    double r23 = mat.element_[1 * 4 + 2];
    double r31 = mat.element_[2 * 4 + 0];
    double r32 = mat.element_[2 * 4 + 1];
    double r33 = mat.element_[2 * 4 + 2];

    // Roll-Pitch-Yaw - Euler ZYZ
    Output_x[3] = atan2(r32, -r31) * kRad2Deg;
    Output_x[4] = atan2(sqrt(r13 * r13 + r23 * r23), r33) * kRad2Deg;
    Output_x[5] = atan2(r23, r13) * kRad2Deg;
}

void RobotModel::fwdKineZYZ(int joint, double Output_x[6]) {
    ModelMatrix matT07 = transformationMatrix(0);
    for (int i = 1; i < joint; i++) matT07 = matT07 * transformationMatrix(i);

    Output_x[0] = matT07.element_[0 * 4 + 3];
    Output_x[1] = matT07.element_[1 * 4 + 3];
    Output_x[2] = matT07.element_[2 * 4 + 3];

    double r13 = matT07.element_[0 * 4 + 2];
    double r23 = matT07.element_[1 * 4 + 2];
    double r31 = matT07.element_[2 * 4 + 0];
    double r32 = matT07.element_[2 * 4 + 1];
    double r33 = matT07.element_[2 * 4 + 2];

    // Roll-Pitch-Yaw - Euler ZYZ
    Output_x[3] = atan2(r32, -r31) * kRad2Deg;
    Output_x[4] = atan2(sqrt(r13 * r13 + r23 * r23), r33) * kRad2Deg;
    Output_x[5] = atan2(r23, r13) * kRad2Deg;
}

void RobotModel::fwdDiffKine(double qd[JS_DOF], double xd[6]) {
    ModelMatrix matQdot(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) matQdot.element_[i] = qd[i] * kDeg2Rad;

    ModelMatrix matJacobianRPY(6, JS_DOF);
    calJacobianRPY(&matJacobianRPY);

    ModelMatrix matXdot(6, 1);
    matXdot = matJacobianRPY * matQdot;

    for (int i = 0; i < 3; i++) {
        xd[i] = matXdot.element_[i];
        xd[i + 3] = matXdot.element_[i + 3] * kRad2Deg;
    }
}

void RobotModel::invDiffKine(double xd[6], double qd[JS_DOF]) {
    ModelMatrix matJacobian(6, JS_DOF);
    calJacobianGeo(&matJacobian);

    ModelMatrix matXdot(6, 1);
    for (int i = 0; i < 3; i++) matXdot.element_[i] = xd[i];
    for (int i = 3; i < 6; i++) matXdot.element_[i] = xd[i] * kDeg2Rad;

    ModelMatrix matQdot(JS_DOF, 1);
    matQdot = (matJacobian.inverse()) * matXdot;

    for (int i = 0; i < JS_DOF; i++) qd[i] = matQdot.element_[i] * kRad2Deg;
}

void RobotModel::invDiffKine_DLS(double xd[6], double sigma, double qd[JS_DOF]) {
    ModelMatrix matJacobian(6, JS_DOF);
    calJacobianGeo(&matJacobian);

    ModelMatrix matXdot(6, 1);
    for (int i = 0; i < 3; i++) matXdot.element_[i] = xd[i];
    for (int i = 3; i < 6; i++) matXdot.element_[i] = xd[i] * kDeg2Rad;

    ModelMatrix matQdot(JS_DOF, 1);
    matQdot = (matJacobian.inverse(sigma)) * matXdot;

    for (int i = 0; i < JS_DOF; i++) qd[i] = matQdot.element_[i] * kRad2Deg;
}

void RobotModel::invDiffKine_rpy(double xd[6], double qd[JS_DOF]) {
    ModelMatrix matJacobianRPY(6, JS_DOF);
    calJacobianRPY(&matJacobianRPY);

    ModelMatrix matXdot(6, 1);
    for (int i = 0; i < 3; i++) matXdot.element_[i] = xd[i];
    for (int i = 3; i < 6; i++) matXdot.element_[i] = xd[i] * kDeg2Rad;

    ModelMatrix matQdot(JS_DOF, 1);
    matQdot = (matJacobianRPY.inverse()) * matXdot;

    for (int i = 0; i < JS_DOF; i++) qd[i] = matQdot.element_[i] * kRad2Deg;
}

void RobotModel::invDiffKine_rpy_DLS(double xd[6], double sigma, double qd[JS_DOF]) {
    ModelMatrix matJacobianRPY(6, JS_DOF);
    calJacobianRPY(&matJacobianRPY);

    ModelMatrix matXdot(6, 1);
    for (int i = 0; i < 3; i++) matXdot.element_[i] = xd[i];
    for (int i = 3; i < 6; i++) matXdot.element_[i] = xd[i] * kDeg2Rad;

    ModelMatrix matQdot(JS_DOF, 1);
    matQdot = (matJacobianRPY.inverse(sigma)) * matXdot;

    for (int i = 0; i < JS_DOF; i++) qd[i] = matQdot.element_[i] * kRad2Deg;
}

void RobotModel::fwdAccKine(double qdd[JS_DOF], double xdd[6]) {
    ModelMatrix matQdot(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) matQdot.element_[i] = qd_[i];

    ModelMatrix matQdoubledot(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) matQdoubledot.element_[i] = qdd[i] * kDeg2Rad;

    ModelMatrix matJacobianGeo(6, JS_DOF);
    calJacobianGeo(&matJacobianGeo);

    ModelMatrix matJacobianGeo_dot(6, JS_DOF);
    calJacobianTimeDerivate(&matJacobianGeo_dot);

    ModelMatrix matXdoubledot(6, 1);
    matXdoubledot = matJacobianGeo * matQdoubledot + matJacobianGeo_dot * matQdot;

    for (int i = 0; i < 3; i++) {
        xdd[i] = matXdoubledot.element_[i];
        xdd[i] = matXdoubledot.element_[i + 3] * kRad2Deg;
    }
}

void RobotModel::invAccKine([[maybe_unused]] double xd[6], double xdd[6], double qdd[JS_DOF]) {
    ModelMatrix matXdoubledot(6, 1);
    for (int i = 0; i < 3; i++) {
        matXdoubledot.element_[i] = xdd[i];
        matXdoubledot.element_[i + 3] = xdd[i + 3] * kDeg2Rad;
    }

    ModelMatrix matQdot(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) matQdot.element_[i] = qd_[i];

    ModelMatrix matJacobian(6, JS_DOF);
    calJacobianGeo(&matJacobian);

    ModelMatrix matJacobian_dot(6, JS_DOF);
    calJacobianTimeDerivate(&matJacobian_dot);

    ModelMatrix matQdoubledot(JS_DOF, 1);
    matQdoubledot = matJacobian.inverse(0.00001) * (matXdoubledot - matJacobian_dot * matQdot);

    for (int i = 0; i < 6; i++) {
        qdd[i] = matQdoubledot.element_[i] * kRad2Deg;
    }
}

void RobotModel::fwdStatics(double torque[JS_DOF], double force[6]) {
    // Input torque
    ModelMatrix matTorque(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) matTorque.element_[i] = torque[i];

    // Cal Geomatric Jacobian
    ModelMatrix matJacobGeo(6, JS_DOF);
    calJacobianGeo(&matJacobGeo);

    // Cal End-effector force
    ModelMatrix matForce(6, 1);
    matForce = ((matJacobGeo.transpose()).inverse()) * matTorque;

    // Output force
    for (int i = 0; i < 6; i++) force[i] = matForce.element_[i];
}

void RobotModel::fwdStatics_DLS(double torque[JS_DOF], double sigma, double force[6]) {
    // Input torque
    ModelMatrix matTorque(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) matTorque.element_[i] = torque[i];

    // Cal Geomatric Jacobian
    ModelMatrix matJacobGeo(6, JS_DOF);
    calJacobianGeo(&matJacobGeo);

    // Cal End-effector force
    ModelMatrix matForce(6, 1);
    matForce = ((matJacobGeo.transpose()).inverse(sigma)) * matTorque;

    // Output force
    for (int i = 0; i < 6; i++) force[i] = matForce.element_[i];
}

void RobotModel::invStatics(double force[6], double torque[JS_DOF]) {
    // Input force
    ModelMatrix matForce(6, 1);
    for (int i = 0; i < 6; i++) matForce.element_[i] = force[i];

    // Cal Geomatric Jacobian
    ModelMatrix matJacobGeo(6, JS_DOF);
    calJacobianGeo(&matJacobGeo);

    // Cal Joint torque
    ModelMatrix matTorque(JS_DOF, 1);
    matTorque = (matJacobGeo.transpose()) * matForce;

    // OutPut torque
    for (int i = 0; i < JS_DOF; i++) torque[i] = matTorque.element_[i];
}

double RobotModel::calDeterminant() {
    ModelMatrix matJacobianRPY(6, JS_DOF);
    calJacobianRPY(&matJacobianRPY);

    return matJacobianRPY.determinant();
}

void RobotModel::calJacobianGeo(ModelMatrix* jacobian_geo) {
    ModelMatrix matTrans[JS_DOF];
    for (int i = 0; i < JS_DOF; i++) {
        if (i == 0)
            matTrans[i] = transformationMatrix(i);
        else
            matTrans[i] = matTrans[i - 1] * transformationMatrix(i);
    }

    ModelMatrix tcpMat = makeTcpMatrix();

    matTrans[JS_DOF - 1] = matTrans[JS_DOF - 1] * tcpMat;

    ModelMatrix matP7(3, 1);
    for (int i = 0; i < 3; i++) matP7.element_[i] = matTrans[JS_DOF - 1].element_[i * 4 + 3];
    ModelMatrix matZ(3, 1);
    ModelMatrix matZCross(3, 1);
    ModelMatrix matP(3, 1);
    ModelMatrix matJacobianGeo(6, JS_DOF);
    for (int i = 0; i < JS_DOF; i++) {
        if (i == 0) {
            matZ.element_[0] = 0.0;
            matZ.element_[1] = 0.0;
            matZ.element_[2] = 1.0;
            matP.element_[0] = 0.0;
            matP.element_[1] = 0.0;
            matP.element_[2] = 0.0;
        }
        else {
            for (int j = 0; j < 3; j++) {
                matZ.element_[j] = matTrans[i - 1].element_[j * 4 + 2];
                matP.element_[j] = matTrans[i - 1].element_[j * 4 + 3];
            }
        }
        matZCross = ((matZ.cross()) * (matP7 - matP));
        for (int j = 0; j < 3; j++) {
            matJacobianGeo.element_[j * JS_DOF + i] = matZCross.element_[j];
            matJacobianGeo.element_[(j + 3) * JS_DOF + i] = matZ.element_[j];
        }
    }
    (*jacobian_geo) = matJacobianGeo;
}

void RobotModel::calJacobianRPY(ModelMatrix* jacobian_rpy) {
    double dX[6] = { 0.0 };
    fwdKine(dX);

    double dThetaY = dX[4] * kDeg2Rad;
    double dThetaZ = dX[5] * kDeg2Rad;

    ModelMatrix matTransRPY(3, 3);

    matTransRPY.element_[0 * 3 + 0] = cos(dThetaY) * cos(dThetaZ);
    matTransRPY.element_[0 * 3 + 1] = -sin(dThetaZ);
    matTransRPY.element_[0 * 3 + 2] = 0.0;

    matTransRPY.element_[1 * 3 + 0] = cos(dThetaY) * sin(dThetaZ);
    matTransRPY.element_[1 * 3 + 1] = cos(dThetaZ);
    matTransRPY.element_[1 * 3 + 2] = 0.0;

    matTransRPY.element_[2 * 3 + 0] = -sin(dThetaY);
    matTransRPY.element_[2 * 3 + 1] = 0.0;
    matTransRPY.element_[2 * 3 + 2] = 1.0;

    ModelMatrix matJacobianGeo(6, JS_DOF);
    ModelMatrix matJacobianRPY(3, JS_DOF);

    calJacobianGeo(&matJacobianGeo);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < JS_DOF; j++) {
            matJacobianRPY.element_[i * JS_DOF + j] = matJacobianGeo.element_[(i + 3) * JS_DOF + j];
        }
    }

    // Test of matrix inverse
    matJacobianRPY = (matTransRPY.inverse()) * matJacobianRPY;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < JS_DOF; j++) {
            jacobian_rpy->element_[i * JS_DOF + j] = matJacobianGeo.element_[i * JS_DOF + j];
            jacobian_rpy->element_[(i + 3) * JS_DOF + j] = matJacobianRPY.element_[i * JS_DOF + j];
        }
    }
}

void RobotModel::calJacobianTimeDerivate(ModelMatrix* jacobian_dot) {
    [[maybe_unused]] double q1, q2, q3, q4, q5, q6, q7;
    [[maybe_unused]] double d1, d2, d3, d4, d5, d6, d7;
    [[maybe_unused]] double a1, a2, a3, a4, a5, a6, a7;
    [[maybe_unused]] double Jdq[6][JS_DOF][JS_DOF] = { {{0.0,},}, };

    q1 = q_[0];
    q2 = q_[1];
    q3 = q_[2];
    q4 = q_[3];
    q5 = q_[4];
    q6 = q_[5];
    q7 = q_[6];
    d1 = dh_[0][2];
    d2 = dh_[1][2];
    d3 = dh_[2][2];
    d4 = dh_[3][2];
    d5 = dh_[4][2];
    d6 = dh_[5][2];
    d7 = dh_[6][2];
    a1 = dh_[0][1];
    a2 = dh_[1][1];
    a3 = dh_[2][1];
    a4 = dh_[3][1];
    a5 = dh_[4][1];
    a6 = dh_[5][1];
    a7 = dh_[6][1];

    [[maybe_unused]] double s23 = sin(q2 + q3);
    [[maybe_unused]] double c23 = cos(q2 + q3);
    [[maybe_unused]] double s234 = sin(q2 + q3 + q4);
    [[maybe_unused]] double c234 = cos(q2 + q3 + q4);

    Jdq[0][0][0] = d3 * cos(q1) * sin(q2) - d7 * (cos(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)))) - d5 * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2));
    Jdq[0][0][1] = d5 * sin(q1) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)) - d7 * (sin(q1) * sin(q6) * (cos(q2) * cos(q5) * sin(q4) - sin(q2) * sin(q3) * sin(q5) + cos(q3) * cos(q4) * cos(q5) * sin(q2)) - cos(q6) * sin(q1) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) + d3 * cos(q2) * sin(q1);
    Jdq[0][0][2] = d5 * sin(q4) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) - d7 * (sin(q6) * (sin(q5) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))) - cos(q6) * sin(q4) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)));
    Jdq[0][0][3] = d7 * (cos(q6) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2))) + d5 * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4));
    Jdq[0][0][4] = -d7 * sin(q6) * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)));
    Jdq[0][0][5] = -d7 * (sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))));
    Jdq[0][0][6] = 0;

    Jdq[0][1][0] = sin(q1) * (d5 * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)) - d7 * (sin(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) + d3 * cos(q2));
    Jdq[0][1][1] = cos(q1) * (d5 * (cos(q4) * sin(q2) + cos(q2) * cos(q3) * sin(q4)) + d3 * sin(q2) - d7 * (sin(q6) * (cos(q5) * (sin(q2) * sin(q4) - cos(q2) * cos(q3) * cos(q4)) + cos(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q4) * sin(q2) + cos(q2) * cos(q3) * sin(q4))));
    Jdq[0][1][2] = -cos(q1) * (d7 * (sin(q6) * (cos(q3) * sin(q2) * sin(q5) + cos(q4) * cos(q5) * sin(q2) * sin(q3)) + cos(q6) * sin(q2) * sin(q3) * sin(q4)) + d5 * sin(q2) * sin(q3) * sin(q4));
    Jdq[0][1][3] = cos(q1) * (d5 * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + d7 * (cos(q6) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))));
    Jdq[0][1][4] = -d7 * cos(q1) * sin(q6) * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3));
    Jdq[0][1][5] = d7 * cos(q1) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)));
    Jdq[0][1][6] = 0;

    Jdq[0][2][0] = d5 * cos(q1) * cos(q3) * sin(q4) + d7 * cos(q1) * cos(q3) * cos(q6) * sin(q4) - d5 * cos(q2) * sin(q1) * sin(q3) * sin(q4) - d7 * cos(q1) * sin(q3) * sin(q5) * sin(q6) - d7 * cos(q2) * cos(q6) * sin(q1) * sin(q3) * sin(q4) - d7 * cos(q2) * cos(q3) * sin(q1) * sin(q5) * sin(q6) + d7 * cos(q1) * cos(q3) * cos(q4) * cos(q5) * sin(q6) - d7 * cos(q2) * cos(q4) * cos(q5) * sin(q1) * sin(q3) * sin(q6);
    Jdq[0][2][1] = -cos(q1) * sin(q2) * (d5 * sin(q3) * sin(q4) + d7 * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q3) * sin(q5) * sin(q6) + d7 * cos(q4) * cos(q5) * sin(q3) * sin(q6));
    Jdq[0][2][2] = d5 * cos(q1) * cos(q2) * cos(q3) * sin(q4) - d5 * sin(q1) * sin(q3) * sin(q4) - d7 * cos(q6) * sin(q1) * sin(q3) * sin(q4) - d7 * cos(q3) * sin(q1) * sin(q5) * sin(q6) - d7 * cos(q1) * cos(q2) * sin(q3) * sin(q5) * sin(q6) - d7 * cos(q4) * cos(q5) * sin(q1) * sin(q3) * sin(q6) + d7 * cos(q1) * cos(q2) * cos(q3) * cos(q6) * sin(q4) + d7 * cos(q1) * cos(q2) * cos(q3) * cos(q4) * cos(q5) * sin(q6);
    Jdq[0][2][3] = (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) * (d5 * cos(q4) + d7 * cos(q4) * cos(q6) - d7 * cos(q5) * sin(q4) * sin(q6));
    Jdq[0][2][4] = -d7 * sin(q6) * (cos(q5) * sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3) * cos(q5) + cos(q3) * cos(q4) * sin(q1) * sin(q5) + cos(q1) * cos(q2) * cos(q4) * sin(q3) * sin(q5));
    Jdq[0][2][5] = -d7 * (cos(q3) * sin(q1) * sin(q4) * sin(q6) + cos(q6) * sin(q1) * sin(q3) * sin(q5) - cos(q1) * cos(q2) * cos(q3) * cos(q6) * sin(q5) - cos(q3) * cos(q4) * cos(q5) * cos(q6) * sin(q1) + cos(q1) * cos(q2) * sin(q3) * sin(q4) * sin(q6) - cos(q1) * cos(q2) * cos(q4) * cos(q5) * cos(q6) * sin(q3));
    Jdq[0][2][6] = 0;

    Jdq[0][3][0] = -(cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) * (d7 * (sin(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - d5 * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - sin(q2) * sin(q3) * (d5 * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) + d7 * (cos(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)))));
    Jdq[0][3][1] = cos(q1) * (d5 * cos(q2) * sin(q4) + d5 * cos(q3) * cos(q4) * sin(q2) + d7 * cos(q2) * cos(q6) * sin(q4) + d7 * cos(q3) * cos(q4) * cos(q6) * sin(q2) + d7 * cos(q2) * cos(q4) * cos(q5) * sin(q6) - d7 * cos(q3) * cos(q5) * sin(q2) * sin(q4) * sin(q6));
    Jdq[0][3][2] = (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) * (d5 * cos(q4) + d7 * cos(q4) * cos(q6) - d7 * cos(q5) * sin(q4) * sin(q6));
    Jdq[0][3][3] = (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) * (d5 * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + d7 * (cos(q6) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)))) + sin(q2) * sin(q3) * (d7 * (cos(q6) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2))) + d5 * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)));
    Jdq[0][3][4] = -d7 * sin(q5) * sin(q6) * (cos(q1) * cos(q4) * sin(q2) - sin(q1) * sin(q3) * sin(q4) + cos(q1) * cos(q2) * cos(q3) * sin(q4));
    Jdq[0][3][5] = d7 * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) - d7 * sin(q2) * sin(q3) * (sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))));
    Jdq[0][3][6] = 0;

    Jdq[0][4][0] = -d7 * sin(q6) * (cos(q2) * cos(q5) * sin(q1) * sin(q3) - cos(q1) * cos(q3) * cos(q5) + cos(q1) * cos(q4) * sin(q3) * sin(q5) - sin(q1) * sin(q2) * sin(q4) * sin(q5) + cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q5));
    Jdq[0][4][1] = -d7 * cos(q1) * sin(q6) * (cos(q5) * sin(q2) * sin(q3) + cos(q2) * sin(q4) * sin(q5) + cos(q3) * cos(q4) * sin(q2) * sin(q5));
    Jdq[0][4][2] = -d7 * sin(q6) * (cos(q5) * sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3) * cos(q5) + cos(q3) * cos(q4) * sin(q1) * sin(q5) + cos(q1) * cos(q2) * cos(q4) * sin(q3) * sin(q5));
    Jdq[0][4][3] = -d7 * sin(q5) * sin(q6) * (cos(q1) * cos(q4) * sin(q2) - sin(q1) * sin(q3) * sin(q4) + cos(q1) * cos(q2) * cos(q3) * sin(q4));
    Jdq[0][4][4] = -d7 * sin(q6) * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)) - d7 * sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3));
    Jdq[0][4][5] = d7 * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - d7 * (sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)))) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[0][4][6] = 0;

    Jdq[0][5][0] = d7 * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) * (sin(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - d7 * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3)) * (cos(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))));
    Jdq[0][5][1] = d7 * cos(q1) * (cos(q2) * cos(q4) * sin(q6) + cos(q2) * cos(q5) * cos(q6) * sin(q4) - cos(q3) * sin(q2) * sin(q4) * sin(q6) - cos(q6) * sin(q2) * sin(q3) * sin(q5) + cos(q3) * cos(q4) * cos(q5) * cos(q6) * sin(q2));
    Jdq[0][5][2] = d7 * cos(q1) * cos(q2) * cos(q3) * cos(q6) * sin(q5) - d7 * cos(q6) * sin(q1) * sin(q3) * sin(q5) - d7 * cos(q1) * cos(q2) * sin(q3) * sin(q4) * sin(q6) - d7 * cos(q3) * sin(q1) * sin(q4) * sin(q6) + d7 * cos(q3) * cos(q4) * cos(q5) * cos(q6) * sin(q1) + d7 * cos(q1) * cos(q2) * cos(q4) * cos(q5) * cos(q6) * sin(q3);
    Jdq[0][5][3] = d7 * cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(q6) - d7 * cos(q4) * sin(q1) * sin(q3) * sin(q6) - d7 * cos(q5) * cos(q6) * sin(q1) * sin(q3) * sin(q4) - d7 * cos(q1) * sin(q2) * sin(q4) * sin(q6) + d7 * cos(q1) * cos(q4) * cos(q5) * cos(q6) * sin(q2) + d7 * cos(q1) * cos(q2) * cos(q3) * cos(q5) * cos(q6) * sin(q4);
    Jdq[0][5][4] = d7 * cos(q6) * (cos(q3) * cos(q5) * sin(q1) + cos(q1) * cos(q2) * cos(q5) * sin(q3) - cos(q1) * sin(q2) * sin(q4) * sin(q5) - cos(q4) * sin(q1) * sin(q3) * sin(q5) + cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(q5));
    Jdq[0][5][5] = -d7 * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - d7 * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3)) * (sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))));
    Jdq[0][5][6] = 0;

    Jdq[0][6][0] = 0;
    Jdq[0][6][1] = 0;
    Jdq[0][6][2] = 0;
    Jdq[0][6][3] = 0;
    Jdq[0][6][4] = 0;
    Jdq[0][6][5] = 0;
    Jdq[0][6][6] = 0;

    Jdq[1][0][0] = d5 * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) + d7 * (cos(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)))) + d3 * sin(q1) * sin(q2);
    Jdq[1][0][1] = -d7 * (cos(q6) * (cos(q1) * cos(q2) * cos(q4) - cos(q1) * cos(q3) * sin(q2) * sin(q4)) - sin(q6) * (cos(q5) * (cos(q1) * cos(q2) * sin(q4) + cos(q1) * cos(q3) * cos(q4) * sin(q2)) - cos(q1) * sin(q2) * sin(q3) * sin(q5))) - d5 * (cos(q1) * cos(q2) * cos(q4) - cos(q1) * cos(q3) * sin(q2) * sin(q4)) - d3 * cos(q1) * cos(q2);
    Jdq[1][0][2] = d5 * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) - d7 * (sin(q6) * (sin(q5) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q4) * cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) - cos(q6) * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)));
    Jdq[1][0][3] = d7 * (cos(q6) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2))) + d5 * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4));
    Jdq[1][0][4] = -d7 * sin(q6) * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)));
    Jdq[1][0][5] = -d7 * (sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))));
    Jdq[1][0][6] = 0;

    Jdq[1][1][0] = -cos(q1) * (d5 * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)) - d7 * (sin(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) + d3 * cos(q2));
    Jdq[1][1][1] = sin(q1) * (d5 * (cos(q4) * sin(q2) + cos(q2) * cos(q3) * sin(q4)) + d3 * sin(q2) - d7 * (sin(q6) * (cos(q5) * (sin(q2) * sin(q4) - cos(q2) * cos(q3) * cos(q4)) + cos(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q4) * sin(q2) + cos(q2) * cos(q3) * sin(q4))));
    Jdq[1][1][2] = -sin(q1) * (d7 * (sin(q6) * (cos(q3) * sin(q2) * sin(q5) + cos(q4) * cos(q5) * sin(q2) * sin(q3)) + cos(q6) * sin(q2) * sin(q3) * sin(q4)) + d5 * sin(q2) * sin(q3) * sin(q4));
    Jdq[1][1][3] = sin(q1) * (d5 * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + d7 * (cos(q6) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))));
    Jdq[1][1][4] = -d7 * sin(q1) * sin(q6) * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3));
    Jdq[1][1][5] = d7 * sin(q1) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)));
    Jdq[1][1][6] = 0;

    Jdq[1][2][0] = d5 * cos(q3) * sin(q1) * sin(q4) + d5 * cos(q1) * cos(q2) * sin(q3) * sin(q4) + d7 * cos(q3) * cos(q6) * sin(q1) * sin(q4) - d7 * sin(q1) * sin(q3) * sin(q5) * sin(q6) + d7 * cos(q1) * cos(q2) * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q1) * cos(q2) * cos(q3) * sin(q5) * sin(q6) + d7 * cos(q3) * cos(q4) * cos(q5) * sin(q1) * sin(q6) + d7 * cos(q1) * cos(q2) * cos(q4) * cos(q5) * sin(q3) * sin(q6);
    Jdq[1][2][1] = -sin(q1) * sin(q2) * (d5 * sin(q3) * sin(q4) + d7 * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q3) * sin(q5) * sin(q6) + d7 * cos(q4) * cos(q5) * sin(q3) * sin(q6));
    Jdq[1][2][2] = d5 * cos(q1) * sin(q3) * sin(q4) + d5 * cos(q2) * cos(q3) * sin(q1) * sin(q4) + d7 * cos(q1) * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q1) * cos(q3) * sin(q5) * sin(q6) + d7 * cos(q2) * cos(q3) * cos(q6) * sin(q1) * sin(q4) + d7 * cos(q1) * cos(q4) * cos(q5) * sin(q3) * sin(q6) - d7 * cos(q2) * sin(q1) * sin(q3) * sin(q5) * sin(q6) + d7 * cos(q2) * cos(q3) * cos(q4) * cos(q5) * sin(q1) * sin(q6);
    Jdq[1][2][3] = -(cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) * (d5 * cos(q4) + d7 * cos(q4) * cos(q6) - d7 * cos(q5) * sin(q4) * sin(q6));
    Jdq[1][2][4] = d7 * sin(q6) * (cos(q1) * cos(q5) * sin(q3) + cos(q2) * cos(q3) * cos(q5) * sin(q1) + cos(q1) * cos(q3) * cos(q4) * sin(q5) - cos(q2) * cos(q4) * sin(q1) * sin(q3) * sin(q5));
    Jdq[1][2][5] = d7 * (cos(q1) * cos(q3) * sin(q4) * sin(q6) + cos(q1) * cos(q6) * sin(q3) * sin(q5) - cos(q1) * cos(q3) * cos(q4) * cos(q5) * cos(q6) + cos(q2) * cos(q3) * cos(q6) * sin(q1) * sin(q5) - cos(q2) * sin(q1) * sin(q3) * sin(q4) * sin(q6) + cos(q2) * cos(q4) * cos(q5) * cos(q6) * sin(q1) * sin(q3));
    Jdq[1][2][6] = 0;

    Jdq[1][3][0] = (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) * (d7 * (sin(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - d5 * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) + sin(q2) * sin(q3) * (d5 * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) + d7 * (cos(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)))));
    Jdq[1][3][1] = sin(q1) * (d5 * cos(q2) * sin(q4) + d5 * cos(q3) * cos(q4) * sin(q2) + d7 * cos(q2) * cos(q6) * sin(q4) + d7 * cos(q3) * cos(q4) * cos(q6) * sin(q2) + d7 * cos(q2) * cos(q4) * cos(q5) * sin(q6) - d7 * cos(q3) * cos(q5) * sin(q2) * sin(q4) * sin(q6));
    Jdq[1][3][2] = -(cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) * (d5 * cos(q4) + d7 * cos(q4) * cos(q6) - d7 * cos(q5) * sin(q4) * sin(q6));
    Jdq[1][3][3] = (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) * (d5 * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + d7 * (cos(q6) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)))) + sin(q2) * sin(q3) * (d7 * (cos(q6) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2))) + d5 * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)));
    Jdq[1][3][4] = -d7 * sin(q5) * sin(q6) * (cos(q4) * sin(q1) * sin(q2) + cos(q1) * sin(q3) * sin(q4) + cos(q2) * cos(q3) * sin(q1) * sin(q4));
    Jdq[1][3][5] = d7 * cos(q1) * cos(q4) * sin(q3) * sin(q6) - d7 * sin(q1) * sin(q2) * sin(q4) * sin(q6) + d7 * cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q6) + d7 * cos(q4) * cos(q5) * cos(q6) * sin(q1) * sin(q2) + d7 * cos(q1) * cos(q5) * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q2) * cos(q3) * cos(q5) * cos(q6) * sin(q1) * sin(q4);
    Jdq[1][3][6] = 0;

    Jdq[1][4][0] = d7 * sin(q6) * (cos(q3) * cos(q5) * sin(q1) + cos(q1) * cos(q2) * cos(q5) * sin(q3) - cos(q1) * sin(q2) * sin(q4) * sin(q5) - cos(q4) * sin(q1) * sin(q3) * sin(q5) + cos(q1) * cos(q2) * cos(q3) * cos(q4) * sin(q5));
    Jdq[1][4][1] = -d7 * sin(q1) * sin(q6) * (cos(q5) * sin(q2) * sin(q3) + cos(q2) * sin(q4) * sin(q5) + cos(q3) * cos(q4) * sin(q2) * sin(q5));
    Jdq[1][4][2] = d7 * sin(q6) * (cos(q1) * cos(q5) * sin(q3) + cos(q2) * cos(q3) * cos(q5) * sin(q1) + cos(q1) * cos(q3) * cos(q4) * sin(q5) - cos(q2) * cos(q4) * sin(q1) * sin(q3) * sin(q5));
    Jdq[1][4][3] = -d7 * sin(q5) * sin(q6) * (cos(q4) * sin(q1) * sin(q2) + cos(q1) * sin(q3) * sin(q4) + cos(q2) * cos(q3) * sin(q1) * sin(q4));
    Jdq[1][4][4] = -d7 * sin(q6) * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)) - d7 * sin(q6) * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3)) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2));
    Jdq[1][4][5] = d7 * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) - d7 * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)) * (sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))));
    Jdq[1][4][6] = 0;

    Jdq[1][5][0] = d7 * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3)) * (cos(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)))) - d7 * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))) * (sin(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4)));
    Jdq[1][5][1] = d7 * sin(q1) * (cos(q2) * cos(q4) * sin(q6) + cos(q2) * cos(q5) * cos(q6) * sin(q4) - cos(q3) * sin(q2) * sin(q4) * sin(q6) - cos(q6) * sin(q2) * sin(q3) * sin(q5) + cos(q3) * cos(q4) * cos(q5) * cos(q6) * sin(q2));
    Jdq[1][5][2] = d7 * cos(q1) * cos(q3) * sin(q4) * sin(q6) + d7 * cos(q1) * cos(q6) * sin(q3) * sin(q5) + d7 * cos(q2) * cos(q3) * cos(q6) * sin(q1) * sin(q5) - d7 * cos(q2) * sin(q1) * sin(q3) * sin(q4) * sin(q6) - d7 * cos(q1) * cos(q3) * cos(q4) * cos(q5) * cos(q6) + d7 * cos(q2) * cos(q4) * cos(q5) * cos(q6) * sin(q1) * sin(q3);
    Jdq[1][5][3] = d7 * cos(q1) * cos(q4) * sin(q3) * sin(q6) - d7 * sin(q1) * sin(q2) * sin(q4) * sin(q6) + d7 * cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q6) + d7 * cos(q4) * cos(q5) * cos(q6) * sin(q1) * sin(q2) + d7 * cos(q1) * cos(q5) * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q2) * cos(q3) * cos(q5) * cos(q6) * sin(q1) * sin(q4);
    Jdq[1][5][4] = d7 * cos(q6) * (cos(q2) * cos(q5) * sin(q1) * sin(q3) - cos(q1) * cos(q3) * cos(q5) + cos(q1) * cos(q4) * sin(q3) * sin(q5) - sin(q1) * sin(q2) * sin(q4) * sin(q5) + cos(q2) * cos(q3) * cos(q4) * sin(q1) * sin(q5));
    Jdq[1][5][5] = -d7 * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) * (cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) + sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4))) - d7 * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3)) * (sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))));
    Jdq[1][5][6] = 0;

    Jdq[1][6][0] = 0;
    Jdq[1][6][1] = 0;
    Jdq[1][6][2] = 0;
    Jdq[1][6][3] = 0;
    Jdq[1][6][4] = 0;
    Jdq[1][6][5] = 0;
    Jdq[1][6][6] = 0;

    Jdq[2][0][0] = 0;
    Jdq[2][0][1] = 0;
    Jdq[2][0][2] = 0;
    Jdq[2][0][3] = 0;
    Jdq[2][0][4] = 0;
    Jdq[2][0][5] = 0;
    Jdq[2][0][6] = 0;

    Jdq[2][1][0] = 0;
    Jdq[2][1][1] = d5 * cos(q3) * sin(q2) * sin(q4) - d5 * cos(q2) * cos(q4) - d7 * cos(q2) * cos(q4) * cos(q6) - d3 * cos(q2) + d7 * cos(q3) * cos(q6) * sin(q2) * sin(q4) + d7 * cos(q2) * cos(q5) * sin(q4) * sin(q6) - d7 * sin(q2) * sin(q3) * sin(q5) * sin(q6) + d7 * cos(q3) * cos(q4) * cos(q5) * sin(q2) * sin(q6);
    Jdq[2][1][2] = cos(q2) * (d5 * sin(q3) * sin(q4) + d7 * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q3) * sin(q5) * sin(q6) + d7 * cos(q4) * cos(q5) * sin(q3) * sin(q6));
    Jdq[2][1][3] = d5 * sin(q2) * sin(q4) - d5 * cos(q2) * cos(q3) * cos(q4) + d7 * cos(q6) * sin(q2) * sin(q4) - d7 * cos(q2) * cos(q3) * cos(q4) * cos(q6) + d7 * cos(q4) * cos(q5) * sin(q2) * sin(q6) + d7 * cos(q2) * cos(q3) * cos(q5) * sin(q4) * sin(q6);
    Jdq[2][1][4] = d7 * sin(q6) * (cos(q2) * cos(q5) * sin(q3) - sin(q2) * sin(q4) * sin(q5) + cos(q2) * cos(q3) * cos(q4) * sin(q5));
    Jdq[2][1][5] = d7 * (cos(q4) * sin(q2) * sin(q6) + cos(q2) * cos(q3) * sin(q4) * sin(q6) + cos(q2) * cos(q6) * sin(q3) * sin(q5) + cos(q5) * cos(q6) * sin(q2) * sin(q4) - cos(q2) * cos(q3) * cos(q4) * cos(q5) * cos(q6));
    Jdq[2][1][6] = 0;

    Jdq[2][2][0] = 0;
    Jdq[2][2][1] = cos(q2) * (d5 * sin(q3) * sin(q4) + d7 * cos(q6) * sin(q3) * sin(q4) + d7 * cos(q3) * sin(q5) * sin(q6) + d7 * cos(q4) * cos(q5) * sin(q3) * sin(q6));
    Jdq[2][2][2] = sin(q2) * (d5 * cos(q3) * sin(q4) + d7 * cos(q3) * cos(q6) * sin(q4) - d7 * sin(q3) * sin(q5) * sin(q6) + d7 * cos(q3) * cos(q4) * cos(q5) * sin(q6));
    Jdq[2][2][3] = sin(q2) * sin(q3) * (d5 * cos(q4) + d7 * cos(q4) * cos(q6) - d7 * cos(q5) * sin(q4) * sin(q6));
    Jdq[2][2][4] = d7 * sin(q2) * sin(q6) * (cos(q3) * cos(q5) - cos(q4) * sin(q3) * sin(q5));
    Jdq[2][2][5] = d7 * sin(q2) * (cos(q3) * cos(q6) * sin(q5) - sin(q3) * sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6) * sin(q3));
    Jdq[2][2][6] = 0;

    Jdq[2][3][0] = 0;
    Jdq[2][3][1] = d5 * sin(q2) * sin(q4) - d5 * cos(q2) * cos(q3) * cos(q4) + d7 * cos(q6) * sin(q2) * sin(q4) - d7 * cos(q2) * cos(q3) * cos(q4) * cos(q6) + d7 * cos(q4) * cos(q5) * sin(q2) * sin(q6) + d7 * cos(q2) * cos(q3) * cos(q5) * sin(q4) * sin(q6);
    Jdq[2][3][2] = sin(q2) * sin(q3) * (d5 * cos(q4) + d7 * cos(q4) * cos(q6) - d7 * cos(q5) * sin(q4) * sin(q6));
    Jdq[2][3][3] = d5 * cos(q3) * sin(q2) * sin(q4) - d7 * cos(q2) * cos(q4) * cos(q6) - d5 * cos(q2) * cos(q4) + d7 * cos(q3) * cos(q6) * sin(q2) * sin(q4) + d7 * cos(q2) * cos(q5) * sin(q4) * sin(q6) + d7 * cos(q3) * cos(q4) * cos(q5) * sin(q2) * sin(q6);
    Jdq[2][3][4] = d7 * sin(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[2][3][5] = d7 * (cos(q2) * sin(q4) * sin(q6) - cos(q2) * cos(q4) * cos(q5) * cos(q6) + cos(q3) * cos(q4) * sin(q2) * sin(q6) + cos(q3) * cos(q5) * cos(q6) * sin(q2) * sin(q4));
    Jdq[2][3][6] = 0;

    Jdq[2][4][0] = 0;
    Jdq[2][4][1] = d7 * sin(q6) * (cos(q2) * cos(q5) * sin(q3) - sin(q2) * sin(q4) * sin(q5) + cos(q2) * cos(q3) * cos(q4) * sin(q5));
    Jdq[2][4][2] = d7 * sin(q2) * sin(q6) * (cos(q3) * cos(q5) - cos(q4) * sin(q3) * sin(q5));
    Jdq[2][4][3] = d7 * sin(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[2][4][4] = d7 * sin(q6) * (cos(q2) * cos(q5) * sin(q4) - sin(q2) * sin(q3) * sin(q5) + cos(q3) * cos(q4) * cos(q5) * sin(q2));
    Jdq[2][4][5] = d7 * cos(q6) * (cos(q5) * sin(q2) * sin(q3) + cos(q2) * sin(q4) * sin(q5) + cos(q3) * cos(q4) * sin(q2) * sin(q5));
    Jdq[2][4][6] = 0;

    Jdq[2][5][0] = 0;
    Jdq[2][5][1] = d7 * (cos(q4) * sin(q2) * sin(q6) + cos(q2) * cos(q3) * sin(q4) * sin(q6) + cos(q2) * cos(q6) * sin(q3) * sin(q5) + cos(q5) * cos(q6) * sin(q2) * sin(q4) - cos(q2) * cos(q3) * cos(q4) * cos(q5) * cos(q6));
    Jdq[2][5][2] = d7 * sin(q2) * (cos(q3) * cos(q6) * sin(q5) - sin(q3) * sin(q4) * sin(q6) + cos(q4) * cos(q5) * cos(q6) * sin(q3));
    Jdq[2][5][3] = d7 * (cos(q2) * sin(q4) * sin(q6) - cos(q2) * cos(q4) * cos(q5) * cos(q6) + cos(q3) * cos(q4) * sin(q2) * sin(q6) + cos(q3) * cos(q5) * cos(q6) * sin(q2) * sin(q4));
    Jdq[2][5][4] = d7 * cos(q6) * (cos(q5) * sin(q2) * sin(q3) + cos(q2) * sin(q4) * sin(q5) + cos(q3) * cos(q4) * sin(q2) * sin(q5));
    Jdq[2][5][5] = d7 * (cos(q3) * cos(q6) * sin(q2) * sin(q4) - cos(q2) * cos(q4) * cos(q6) + cos(q2) * cos(q5) * sin(q4) * sin(q6) - sin(q2) * sin(q3) * sin(q5) * sin(q6) + cos(q3) * cos(q4) * cos(q5) * sin(q2) * sin(q6));
    Jdq[2][5][6] = 0;

    Jdq[2][6][0] = 0;
    Jdq[2][6][1] = 0;
    Jdq[2][6][2] = 0;
    Jdq[2][6][3] = 0;
    Jdq[2][6][4] = 0;
    Jdq[2][6][5] = 0;
    Jdq[2][6][6] = 0;

    Jdq[3][0][0] = 0;
    Jdq[3][0][1] = 0;
    Jdq[3][0][2] = 0;
    Jdq[3][0][3] = 0;
    Jdq[3][0][4] = 0;
    Jdq[3][0][5] = 0;
    Jdq[3][0][6] = 0;

    Jdq[3][1][0] = cos(q1);
    Jdq[3][1][1] = 0;
    Jdq[3][1][2] = 0;
    Jdq[3][1][3] = 0;
    Jdq[3][1][4] = 0;
    Jdq[3][1][5] = 0;
    Jdq[3][1][6] = 0;

    Jdq[3][2][0] = sin(q1) * sin(q2);
    Jdq[3][2][1] = -cos(q1) * cos(q2);
    Jdq[3][2][2] = 0;
    Jdq[3][2][3] = 0;
    Jdq[3][2][4] = 0;
    Jdq[3][2][5] = 0;
    Jdq[3][2][6] = 0;

    Jdq[3][3][0] = cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3);
    Jdq[3][3][1] = -cos(q1) * sin(q2) * sin(q3);
    Jdq[3][3][2] = cos(q1) * cos(q2) * cos(q3) - sin(q1) * sin(q3);
    Jdq[3][3][3] = 0;
    Jdq[3][3][4] = 0;
    Jdq[3][3][5] = 0;
    Jdq[3][3][6] = 0;

    Jdq[3][4][0] = sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2);
    Jdq[3][4][1] = -cos(q1) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[3][4][2] = sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3));
    Jdq[3][4][3] = cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4);
    Jdq[3][4][4] = 0;
    Jdq[3][4][5] = 0;
    Jdq[3][4][6] = 0;

    Jdq[3][5][0] = cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)) - sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4));
    Jdq[3][5][1] = -sin(q5) * (cos(q1) * cos(q2) * sin(q4) + cos(q1) * cos(q3) * cos(q4) * sin(q2)) - cos(q1) * cos(q5) * sin(q2) * sin(q3);
    Jdq[3][5][2] = -cos(q5) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q4) * sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3));
    Jdq[3][5][3] = sin(q5) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2));
    Jdq[3][5][4] = -cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3));
    Jdq[3][5][5] = 0;
    Jdq[3][5][6] = 0;

    Jdq[3][6][0] = cos(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)));
    Jdq[3][6][1] = cos(q1) * sin(q6) * (cos(q2) * cos(q5) * sin(q4) - sin(q2) * sin(q3) * sin(q5) + cos(q3) * cos(q4) * cos(q5) * sin(q2)) - cos(q1) * cos(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[3][6][2] = cos(q6) * sin(q4) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) - sin(q6) * (sin(q5) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q4) * cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)));
    Jdq[3][6][3] = cos(q6) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2));
    Jdq[3][6][4] = -sin(q6) * (sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)));
    Jdq[3][6][5] = cos(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3))) - sin(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2));
    Jdq[3][6][6] = 0;

    Jdq[4][0][0] = 0;
    Jdq[4][0][1] = 0;
    Jdq[4][0][2] = 0;
    Jdq[4][0][3] = 0;
    Jdq[4][0][4] = 0;
    Jdq[4][0][5] = 0;
    Jdq[4][0][6] = 0;

    Jdq[4][1][0] = sin(q1);
    Jdq[4][1][1] = 0;
    Jdq[4][1][2] = 0;
    Jdq[4][1][3] = 0;
    Jdq[4][1][4] = 0;
    Jdq[4][1][5] = 0;
    Jdq[4][1][6] = 0;

    Jdq[4][2][0] = -cos(q1) * sin(q2);
    Jdq[4][2][1] = -cos(q2) * sin(q1);
    Jdq[4][2][2] = 0;
    Jdq[4][2][3] = 0;
    Jdq[4][2][4] = 0;
    Jdq[4][2][5] = 0;
    Jdq[4][2][6] = 0;

    Jdq[4][3][0] = cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3);
    Jdq[4][3][1] = -sin(q1) * sin(q2) * sin(q3);
    Jdq[4][3][2] = cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1);
    Jdq[4][3][3] = 0;
    Jdq[4][3][4] = 0;
    Jdq[4][3][5] = 0;
    Jdq[4][3][6] = 0;

    Jdq[4][4][0] = sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2);
    Jdq[4][4][1] = -sin(q1) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[4][4][2] = -sin(q4) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3));
    Jdq[4][4][3] = sin(q1) * sin(q2) * sin(q4) - cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1));
    Jdq[4][4][4] = 0;
    Jdq[4][4][5] = 0;
    Jdq[4][4][6] = 0;

    Jdq[4][5][0] = cos(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)) - sin(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4));
    Jdq[4][5][1] = -sin(q5) * (cos(q2) * sin(q1) * sin(q4) + cos(q3) * cos(q4) * sin(q1) * sin(q2)) - cos(q5) * sin(q1) * sin(q2) * sin(q3);
    Jdq[4][5][2] = cos(q5) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3));
    Jdq[4][5][3] = -sin(q5) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2));
    Jdq[4][5][4] = cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3));
    Jdq[4][5][5] = 0;
    Jdq[4][5][6] = 0;

    Jdq[4][6][0] = cos(q6) * (sin(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) - cos(q1) * cos(q4) * sin(q2)) + sin(q6) * (cos(q5) * (cos(q4) * (sin(q1) * sin(q3) - cos(q1) * cos(q2) * cos(q3)) + cos(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q3) * sin(q1) + cos(q1) * cos(q2) * sin(q3)));
    Jdq[4][6][1] = sin(q1) * sin(q6) * (cos(q2) * cos(q5) * sin(q4) - sin(q2) * sin(q3) * sin(q5) + cos(q3) * cos(q4) * cos(q5) * sin(q2)) - cos(q6) * sin(q1) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[4][6][2] = sin(q6) * (sin(q5) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - cos(q4) * cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3))) - cos(q6) * sin(q4) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3));
    Jdq[4][6][3] = cos(q5) * sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) - cos(q6) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4));
    Jdq[4][6][4] = sin(q6) * (sin(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) - cos(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)));
    Jdq[4][6][5] = sin(q6) * (sin(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) + cos(q4) * sin(q1) * sin(q2)) - cos(q6) * (cos(q5) * (cos(q4) * (cos(q1) * sin(q3) + cos(q2) * cos(q3) * sin(q1)) - sin(q1) * sin(q2) * sin(q4)) + sin(q5) * (cos(q1) * cos(q3) - cos(q2) * sin(q1) * sin(q3)));
    Jdq[4][6][6] = 0;

    Jdq[5][0][0] = 0;
    Jdq[5][0][1] = 0;
    Jdq[5][0][2] = 0;
    Jdq[5][0][3] = 0;
    Jdq[5][0][4] = 0;
    Jdq[5][0][5] = 0;
    Jdq[5][0][6] = 0;

    Jdq[5][1][0] = 0;
    Jdq[5][1][1] = 0;
    Jdq[5][1][2] = 0;
    Jdq[5][1][3] = 0;
    Jdq[5][1][4] = 0;
    Jdq[5][1][5] = 0;
    Jdq[5][1][6] = 0;

    Jdq[5][2][0] = 0;
    Jdq[5][2][1] = -sin(q2);
    Jdq[5][2][2] = 0;
    Jdq[5][2][3] = 0;
    Jdq[5][2][4] = 0;
    Jdq[5][2][5] = 0;
    Jdq[5][2][6] = 0;

    Jdq[5][3][0] = 0;
    Jdq[5][3][1] = cos(q2) * sin(q3);
    Jdq[5][3][2] = cos(q3) * sin(q2);
    Jdq[5][3][3] = 0;
    Jdq[5][3][4] = 0;
    Jdq[5][3][5] = 0;
    Jdq[5][3][6] = 0;

    Jdq[5][4][0] = 0;
    Jdq[5][4][1] = -cos(q4) * sin(q2) - cos(q2) * cos(q3) * sin(q4);
    Jdq[5][4][2] = sin(q2) * sin(q3) * sin(q4);
    Jdq[5][4][3] = -cos(q2) * sin(q4) - cos(q3) * cos(q4) * sin(q2);
    Jdq[5][4][4] = 0;
    Jdq[5][4][5] = 0;
    Jdq[5][4][6] = 0;

    Jdq[5][5][0] = 0;
    Jdq[5][5][1] = cos(q2) * cos(q5) * sin(q3) - sin(q5) * (sin(q2) * sin(q4) - cos(q2) * cos(q3) * cos(q4));
    Jdq[5][5][2] = sin(q2) * (cos(q3) * cos(q5) - cos(q4) * sin(q3) * sin(q5));
    Jdq[5][5][3] = sin(q5) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[5][5][4] = cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5);
    Jdq[5][5][5] = 0;
    Jdq[5][5][6] = 0;

    Jdq[5][6][0] = 0;
    Jdq[5][6][1] = sin(q6) * (cos(q5) * (sin(q2) * sin(q4) - cos(q2) * cos(q3) * cos(q4)) + cos(q2) * sin(q3) * sin(q5)) - cos(q6) * (cos(q4) * sin(q2) + cos(q2) * cos(q3) * sin(q4));
    Jdq[5][6][2] = sin(q6) * (cos(q3) * sin(q2) * sin(q5) + cos(q4) * cos(q5) * sin(q2) * sin(q3)) + cos(q6) * sin(q2) * sin(q3) * sin(q4);
    Jdq[5][6][3] = -cos(q6) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - cos(q5) * sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[5][6][4] = sin(q6) * (sin(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) + cos(q5) * sin(q2) * sin(q3));
    Jdq[5][6][5] = -cos(q6) * (cos(q5) * (cos(q2) * sin(q4) + cos(q3) * cos(q4) * sin(q2)) - sin(q2) * sin(q3) * sin(q5)) - sin(q6) * (cos(q2) * cos(q4) - cos(q3) * sin(q2) * sin(q4));
    Jdq[5][6][6] = 0;

    double MatrixWithNoName[6][JS_DOF] = { {0.0,}, };

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < JS_DOF; j++) {
            MatrixWithNoName[i][j] = 0.0;
            for (int k = 0; k < JS_DOF; k++)	MatrixWithNoName[i][j] += Jdq[i][j][k] * qd_[k];
        }
    }

    ModelMatrix matJacobianGeo_dot(6, JS_DOF);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < JS_DOF; j++)	matJacobianGeo_dot.setElement_plus(i + 1, j + 1, MatrixWithNoName[i][j]);
    }

    (*jacobian_dot) = matJacobianGeo_dot;	// Time derivate of geometric Jacobian
}

double RobotModel::calManipulability(std::vector<double> q) {
    ModelMatrix q_rad(JS_DOF, 1);
    for (int i = 0; i < JS_DOF; i++) {
        q_rad.element_[i] = q[i] * kDeg2Rad;
    }

    ModelMatrix jacobian(6, 6);
    ModelMatrix JTJ(6, 6);
    jacobian = calJacobian(q_rad);
    JTJ = jacobian.transpose() * jacobian;

    // for (int i = 0; i < JS_DOF; i++) {
    //     diag[i] = jacobian.get(i, i);
    // }
    // diag[0] = sqrt(JTJ.determinant());
    double manipulability = 0.0;
    manipulability = sqrt(JTJ.determinant());
    return manipulability;
}

void RobotModel::setDynamicParameters(double link_mass[JS_DOF], double com[JS_DOF][3], double inertia[JS_DOF][9], double inertia_rotor[JS_DOF]) {
    m1 = link_mass[0];    m2 = link_mass[1];    m3 = link_mass[2];
    m4 = link_mass[3];    m5 = link_mass[4];    m6 = link_mass[5];    m7 = link_mass[6];

    r1cx = com[0][0];    r1cy = com[0][1];    r1cz = com[0][2];
    r2cx = com[1][0];    r2cy = com[1][1];    r2cz = com[1][2];
    r3cx = com[2][0];    r3cy = com[2][1];    r3cz = com[2][2];
    r4cx = com[3][0];    r4cy = com[3][1];    r4cz = com[3][2];
    r5cx = com[4][0];    r5cy = com[4][1];    r5cz = com[4][2];
    r6cx = com[5][0];    r6cy = com[5][1];    r6cz = com[5][2];
    r7cx = com[6][0];    r7cy = com[6][1];    r7cz = com[6][2];

    I1xx = inertia[0][0];    I1xy = inertia[0][1];    I1xz = inertia[0][2];
    I1yx = inertia[0][3];    I1yy = inertia[0][4];    I1yz = inertia[0][5];
    I1zx = inertia[0][6];    I1zy = inertia[0][7];    I1zz = inertia[0][8];

    I2xx = inertia[1][0];    I2xy = inertia[1][1];    I2xz = inertia[1][2];
    I2yx = inertia[1][3];    I2yy = inertia[1][4];    I2yz = inertia[1][5];
    I2zx = inertia[1][6];    I2zy = inertia[1][7];    I2zz = inertia[1][8];

    I3xx = inertia[2][0];    I3xy = inertia[2][1];    I3xz = inertia[2][2];
    I3yx = inertia[2][3];    I3yy = inertia[2][4];    I3yz = inertia[2][5];
    I3zx = inertia[2][6];    I3zy = inertia[2][7];    I3zz = inertia[2][8];

    I4xx = inertia[3][0];    I4xy = inertia[3][1];    I4xz = inertia[3][2];
    I4yx = inertia[3][3];    I4yy = inertia[3][4];    I4yz = inertia[3][5];
    I4zx = inertia[3][6];    I4zy = inertia[3][7];    I4zz = inertia[3][8];

    I5xx = inertia[4][0];    I5xy = inertia[4][1];    I5xz = inertia[4][2];
    I5yx = inertia[4][3];    I5yy = inertia[4][4];    I5yz = inertia[4][5];
    I5zx = inertia[4][6];    I5zy = inertia[4][7];    I5zz = inertia[4][8];

    I6xx = inertia[5][0];    I6xy = inertia[5][1];    I6xz = inertia[5][2];
    I6yx = inertia[5][3];    I6yy = inertia[5][4];    I6yz = inertia[5][5];
    I6zx = inertia[5][6];    I6zy = inertia[5][7];    I6zz = inertia[5][8];

    I7xx = inertia[6][0];    I7xy = inertia[6][1];    I7xz = inertia[6][2];
    I7yx = inertia[6][3];    I7yy = inertia[6][4];    I7yz = inertia[6][5];
    I7zx = inertia[6][6];    I7zy = inertia[6][7];    I7zz = inertia[6][8];

    for (int i = 0; i < JS_DOF; i++) {
        inertia_rotor_mat_->setElement_plus(i + 1, i + 1, inertia_rotor[i]);
    }
}

void RobotModel::setResidualObserver(double ctrl_period, double gain[JS_DOF]) {
    ctrl_period_ = ctrl_period;
    for (unsigned int i = 0; i < JS_DOF; i++) {
        observer_gain_->setElement_plus(i + 1, i + 1, gain[i]);

        residual_torque_vec_->setElement_plus(i + 1, 1, 0.0);
        residual_torque_current_vec_->setElement_plus(i + 1, 1, 0.0);
        friction_torque_observer_vec_->setElement_plus(i + 1, 1, 0.0);

        residual_torque_integral_->setElement_plus(i + 1, 1, 0.0);
        residual_torque_current_integral_->setElement_plus(i + 1, 1, 0.0);
        friction_torque_observer_integral_->setElement_plus(i + 1, 1, 0.0);
    }
}

void RobotModel::calResidualObserver(double jts_value[JS_DOF], double residual_torque[JS_DOF]) {
    ModelMatrix vecQdot(JS_DOF, 1, qd_);
    ModelMatrix vecJTS(JS_DOF, 1, jts_value);

#if CBM_FLAG
    *residual_torque_integral_ = *residual_torque_integral_ + (coriolis_mat_->transpose() * vecQdot - *gravity_vec_ + *cbm_vec_ - vecJTS - *residual_torque_vec_);
#else
    *residual_torque_integral_ = *residual_torque_integral_ + (coriolis_mat_->transpose() * vecQdot - *gravity_vec_ - vecJTS - *residual_torque_vec_);
#endif

    static ModelMatrix residual_initial = *residual_torque_integral_; // qd(0) 는 초기값 0이라고 가정

    *residual_torque_vec_ = *observer_gain_ * ((*residual_torque_integral_ * ctrl_period_) - (*inertia_mat_ * vecQdot)) + residual_initial;

    for (unsigned int i = 0; i < JS_DOF; i++) {
        residual_torque[i] = residual_torque_vec_->getElement_plus(i + 1, 1);
    }
}

void RobotModel::calResidualObserverCurrent(double torque[JS_DOF], double residual_torque[JS_DOF]) {
    ModelMatrix vecQdot(JS_DOF, 1, qd_);
    ModelMatrix vecTorque(JS_DOF, 1, torque); // 마찰 토크는 뺀 값 입력

#if CBM_FLAG
    *residual_torque_current_integral_ = *residual_torque_current_integral_ + (vecTorque + coriolis_mat_->transpose() * vecQdot - *gravity_vec_ + *cbm_vec_ - *residual_torque_current_vec_);
#else
    *residual_torque_current_integral_ = *residual_torque_current_integral_ + (vecTorque + coriolis_mat_->transpose() * vecQdot - *gravity_vec_ - *residual_torque_current_vec_);
#endif

    static ModelMatrix residual_current_initial = *residual_torque_current_integral_; // qd(0) 는 초기값 0이라고 가정

    *residual_torque_current_vec_ = *observer_gain_ * ((*residual_torque_current_integral_ * ctrl_period_) - (*inertia_mat_ + *inertia_rotor_mat_) * vecQdot) + residual_current_initial;

    for (unsigned int i = 0; i < JS_DOF; i++) {
        residual_torque[i] = residual_torque_current_vec_->getElement_plus(i + 1, 1);
    }
}

// TODO: 오버로딩된 calFrictionObserver 두 함수 계산 과정을 하나로 통합
void RobotModel::calFrictionObserver(double motor_torque[JS_DOF], double jts_value[JS_DOF], double friction_torque[JS_DOF]) {
    // 마찰을 외력으로 생각
    ModelMatrix vecQdot(JS_DOF, 1, qd_);
    ModelMatrix vecMotorTorque(JS_DOF, 1, motor_torque);
    ModelMatrix vecJTS(JS_DOF, 1, jts_value);

    *friction_torque_observer_integral_ = *friction_torque_observer_integral_ + (vecMotorTorque + vecJTS - *friction_torque_observer_vec_);
    *friction_torque_observer_vec_ = *observer_gain_ * (*friction_torque_observer_integral_ * ctrl_period_ - *inertia_rotor_mat_ * vecQdot);

    for (unsigned int i = 0; i < JS_DOF; i++) {
        friction_torque[i] = friction_torque_observer_vec_->getElement_plus(i + 1, 1);
    }
}

void RobotModel::calFrictionObserver(double qd[JS_DOF], double motor_torque[JS_DOF], double jts_value[JS_DOF], double friction_torque[JS_DOF]) {
    double dqdot_rad[JS_DOF];
    for (int i = 0; i < JS_DOF; i++) {
        dqdot_rad[i] = qd[i] * kDeg2Rad;
    }

    // 마찰을 외력으로 생각
    ModelMatrix vecQdot(JS_DOF, 1, dqdot_rad);
    ModelMatrix vecMotorTorque(JS_DOF, 1, motor_torque);
    ModelMatrix vecJTS(JS_DOF, 1, jts_value);

    *friction_torque_observer_integral_ = *friction_torque_observer_integral_ + (vecMotorTorque + vecJTS - *friction_torque_observer_vec_) * ctrl_period_;
    *friction_torque_observer_vec_ = *observer_gain_ * (*friction_torque_observer_integral_ - *inertia_rotor_mat_ * vecQdot);

    for (unsigned int i = 0; i < JS_DOF; i++) {
        friction_torque[i] = friction_torque_observer_vec_->getElement_plus(i + 1, 1);
    }
}

void RobotModel::calInertiaTorque(double dq[JS_DOF], double dqddot[JS_DOF], double inertia_torque[JS_DOF]) {
    double dq_rad[JS_DOF], dqddot_rad[JS_DOF];
    for (int i = 0; i < JS_DOF; i++) {
        dq_rad[i] = dq[i] * kDeg2Rad;
        dqddot_rad[i] = dqddot[i] * kDeg2Rad;
    }
    calInertiaMatrix(dq_rad);
    ModelMatrix vecQddot(JS_DOF, 1, dqddot_rad);
    ModelMatrix vecTorque(JS_DOF, 1);

    vecTorque = (*inertia_mat_ + *inertia_rotor_mat_) * vecQddot;

    for (unsigned int i = 0; i < JS_DOF; i++) {
        inertia_torque[i] = vecTorque.getElement_plus(i + 1, 1);
    }
}

void RobotModel::calCoriolisTorque(double dq[JS_DOF], double qd[JS_DOF], double coriolis_torque[JS_DOF]) {
    double dq_rad[JS_DOF], dqdot_rad[JS_DOF];
    for (int i = 0; i < JS_DOF; i++) {
        dq_rad[i] = dq[i] * kDeg2Rad;
        dqdot_rad[i] = qd[i] * kDeg2Rad;
    }

    calCoriolisMatrix(dq_rad, dqdot_rad);

    ModelMatrix vecQdot(JS_DOF, 1, dqdot_rad);
    ModelMatrix vecCTorque(JS_DOF, 1);

    vecCTorque = *coriolis_mat_ * vecQdot;

    for (unsigned int i = 0; i < JS_DOF; i++)
        coriolis_torque[i] = vecCTorque.getElement_plus(i + 1, 1);
}

void RobotModel::calGravityTorque(double dq[JS_DOF], double gravity_torque[JS_DOF]) {
    double dq_rad[JS_DOF];
    for (int i = 0; i < JS_DOF; i++) {
        dq_rad[i] = dq[i] * kDeg2Rad;
    }

#if CBM_FLAG
    calGravityVector(dq_rad);
    calCBMVector(dq_rad);
    for (unsigned int i = 0; i < JS_DOF; i++) {
        gravity_torque[i] = gravity_vec_->getElement_plus(i + 1, 1) - cbm_vec_->getElement_plus(i + 1, 1);
    }
#else
    calGravityVector(dq_rad);
    for (unsigned int i = 0; i < JS_DOF; i++) {
        gravity_torque[i] = gravity_vec_->getElement_plus(i + 1, 1);
    }
#endif
}

void RobotModel::getInertia(double inertia[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++)
        inertia[i] = inertia_mat_->getElement_plus(i + 1, i + 1) + inertia_rotor_mat_->getElement_plus(i + 1, i + 1);
}

#if CBM_FLAG
void RobotModel::getCBM(double torque_cbm[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        torque_cbm[i] = cbm_vec_->getElement_plus(i + 1, 1);
    }
}
#endif

void RobotModel::rot2rpy(const double rot[3][3], double rpy[3]) {
    rpy[0] = atan2(rot[2][1], rot[2][2]) * kRad2Deg;
    rpy[1] = atan2(-rot[2][0], sqrt(rot[2][1] * rot[2][1] + rot[2][2] * rot[2][2])) * kRad2Deg;
    rpy[2] = atan2(rot[1][0], rot[0][0]) * kRad2Deg;
}

void RobotModel::rpy2rot(const double rpy[3], double rot[3][3], bool is_rad) {
    double rpy_temp[3];

    for (int i = 0; i < 3; i++) {
        rpy_temp[i] = is_rad ? rpy[i] : rpy[i] * kDeg2Rad;
    }

    rot[0][0] = cos(rpy_temp[2]) * cos(rpy_temp[1]);
    rot[0][1] = cos(rpy_temp[2]) * sin(rpy_temp[1]) * sin(rpy_temp[0]) - sin(rpy_temp[2]) * cos(rpy_temp[0]);
    rot[0][2] = cos(rpy_temp[2]) * sin(rpy_temp[1]) * cos(rpy_temp[0]) + sin(rpy_temp[2]) * sin(rpy_temp[0]);

    rot[1][0] = sin(rpy_temp[2]) * cos(rpy_temp[1]);
    rot[1][1] = sin(rpy_temp[2]) * sin(rpy_temp[1]) * sin(rpy_temp[0]) + cos(rpy_temp[2]) * cos(rpy_temp[0]);
    rot[1][2] = sin(rpy_temp[2]) * sin(rpy_temp[1]) * cos(rpy_temp[0]) - cos(rpy_temp[2]) * sin(rpy_temp[0]);

    rot[2][0] = -sin(rpy_temp[1]);
    rot[2][1] = cos(rpy_temp[1]) * sin(rpy_temp[0]);
    rot[2][2] = cos(rpy_temp[1]) * cos(rpy_temp[0]);
}

void RobotModel::rotSum(double rot_1[3][3], double rot_2[3][3], double rot_sum[3][3]) {
    // Rotate rotation matrix
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_sum[i][j] = 0.0;
            for (int k = 0; k < 3; k++) rot_sum[i][j] += rot_1[i][k] * rot_2[k][j];
        }
    }
}

void RobotModel::orientationError(double rot_d[3][3], double rot_m[3][3], double err_orientation[3]) {
    double rot[3][3];
    rot[0][0] = rot_d[0][0] * rot_m[0][0] + rot_d[0][1] * rot_m[0][1] + rot_d[0][2] * rot_m[0][2];
    rot[1][0] = rot_d[1][0] * rot_m[0][0] + rot_d[1][1] * rot_m[0][1] + rot_d[1][2] * rot_m[0][2];
    rot[2][0] = rot_d[2][0] * rot_m[0][0] + rot_d[2][1] * rot_m[0][1] + rot_d[2][2] * rot_m[0][2];
    rot[2][1] = rot_d[2][0] * rot_m[1][0] + rot_d[2][1] * rot_m[1][1] + rot_d[2][2] * rot_m[1][2];
    rot[2][2] = rot_d[2][0] * rot_m[2][0] + rot_d[2][1] * rot_m[2][1] + rot_d[2][2] * rot_m[2][2];

    // Roll-Pitch-Yaw - Euler ZYX
    err_orientation[0] = atan2(rot[2][1], rot[2][2]) * kRad2Deg;
    err_orientation[1] = atan2(-rot[2][0], sqrt(rot[2][1] * rot[2][1] + rot[2][2] * rot[2][2])) * kRad2Deg;
    err_orientation[2] = atan2(rot[1][0], rot[0][0]) * kRad2Deg;
}

void RobotModel::rpyDot2AngularVel(double rpy[3], double rpy_dot[3], double angular_vel[3]) {
    angular_vel[0] = (cos(rpy[1] * kDeg2Rad) * cos(rpy[2] * kDeg2Rad) * rpy_dot[0] * kDeg2Rad - sin(rpy[2] * kDeg2Rad) * rpy_dot[1] * kDeg2Rad) * kRad2Deg;
    angular_vel[1] = (cos(rpy[1] * kDeg2Rad) * sin(rpy[2] * kDeg2Rad) * rpy_dot[0] * kDeg2Rad + cos(rpy[2] * kDeg2Rad) * rpy_dot[1] * kDeg2Rad) * kRad2Deg;
    angular_vel[2] = (-sin(rpy[1] * kDeg2Rad) * rpy_dot[0] * kDeg2Rad + rpy_dot[2] * kDeg2Rad) * kRad2Deg;
}

void RobotModel::angularVel2RPYdot(double rpy[3], double angular_vel[3], double rpy_dot[3]) {
    rpy_dot[0] = (cos(rpy[2] * kDeg2Rad) / cos(rpy[1] * kDeg2Rad) * angular_vel[0] * kDeg2Rad + sin(rpy[2] * kDeg2Rad) / cos(rpy[1] * kDeg2Rad) * angular_vel[1] * kDeg2Rad) * kRad2Deg;
    rpy_dot[1] = (-sin(rpy[2] * kDeg2Rad) * angular_vel[0] * kDeg2Rad + cos(rpy[2] * kDeg2Rad) * angular_vel[1] * kDeg2Rad) * kRad2Deg;
    rpy_dot[2] = (sin(rpy[1] * kDeg2Rad) * cos(rpy[2] * kDeg2Rad) / cos(rpy[1] * kDeg2Rad) * angular_vel[0] * kDeg2Rad + sin(rpy[1] * kDeg2Rad) * sin(rpy[2] * kDeg2Rad) / cos(rpy[1] * kDeg2Rad) * angular_vel[1] * kDeg2Rad) * kRad2Deg;
}

void RobotModel::rpyAcc2AngularAcc(double rpy[3], double rpy_dot[3], double rpy_acc[3], double angular_acc[3]) {
    double dRPYrad[3], dRPYdotrad[3], dRPYAccrad[3];
    for (int i = 0; i < 3; i++) {
        dRPYrad[i] = rpy[i] * kDeg2Rad;
        dRPYdotrad[i] = rpy_dot[i] * kDeg2Rad;
        dRPYAccrad[i] = rpy_acc[i] * kDeg2Rad;
    }
    ModelMatrix vecRPYrad(3, 1, dRPYrad);
    ModelMatrix vecRPYdotrad(3, 1, dRPYdotrad);
    ModelMatrix vecRPYAccrad(3, 1, dRPYAccrad);

    ModelMatrix matB(3, 3);
    matB.element_[0] = cos(dRPYrad[1]) * cos(dRPYrad[2]);
    matB.element_[1] = -sin(dRPYrad[2]);
    matB.element_[2] = 0.0;

    matB.element_[3] = cos(dRPYrad[1]) * sin(dRPYrad[2]);
    matB.element_[4] = cos(dRPYrad[2]);
    matB.element_[5] = 0.0;

    matB.element_[6] = -sin(dRPYrad[1]);
    matB.element_[7] = 0.0;
    matB.element_[8] = 1.0;

    ModelMatrix matBdot(3, 3);
    matBdot.element_[0] = -dRPYdotrad[1] * sin(dRPYrad[1]) * cos(dRPYrad[2]) - dRPYdotrad[2] * cos(dRPYrad[1]) * sin(dRPYrad[2]);
    matBdot.element_[1] = -dRPYdotrad[2] * cos(dRPYrad[2]);
    matBdot.element_[2] = 0.0;

    matBdot.element_[3] = -dRPYdotrad[1] * sin(dRPYrad[1]) * sin(dRPYrad[2]) + dRPYdotrad[2] * cos(dRPYrad[1]) * cos(dRPYrad[2]);
    matBdot.element_[4] = -dRPYdotrad[2] * sin(dRPYrad[2]);
    matBdot.element_[5] = 0.0;

    matBdot.element_[6] = -dRPYdotrad[1] * cos(dRPYrad[1]);
    matBdot.element_[7] = 0.0;
    matBdot.element_[8] = 0.0;

    ModelMatrix vecAngularAcc(3, 1);
    vecAngularAcc = matBdot * vecRPYdotrad + matB * vecRPYAccrad;

    for (int i = 0; i < 3; i++) angular_acc[i] = vecAngularAcc.element_[i] * kRad2Deg;
}

void RobotModel::angularAcc2RPYAcc(double rpy[3], double rpy_dot[3], double angular_acc[3], double rpy_acc[3]) {
    double dRPYrad[3], dRPYdotrad[3], dAngularAccrad[3];
    for (int i = 0; i < 3; i++) {
        dRPYrad[i] = rpy[i] * kDeg2Rad;
        dRPYdotrad[i] = rpy_dot[i] * kDeg2Rad;
        dAngularAccrad[i] = angular_acc[i] * kDeg2Rad;
    }
    ModelMatrix vecRPYrad(3, 1, dRPYrad);
    ModelMatrix vecRPYdotrad(3, 1, dRPYdotrad);
    ModelMatrix vecAngularAccrad(3, 1, dAngularAccrad);

    ModelMatrix matB(3, 3);
    matB.element_[0] = cos(dRPYrad[1]) * cos(dRPYrad[2]);
    matB.element_[1] = -sin(dRPYrad[2]);
    matB.element_[2] = 0.0;

    matB.element_[3] = cos(dRPYrad[1]) * sin(dRPYrad[2]);
    matB.element_[4] = cos(dRPYrad[2]);
    matB.element_[5] = 0.0;

    matB.element_[6] = -sin(dRPYrad[1]);
    matB.element_[7] = 0.0;
    matB.element_[8] = 1.0;

    ModelMatrix matBdot(3, 3);
    matBdot.element_[0] = -dRPYdotrad[1] * sin(dRPYrad[1]) * cos(dRPYrad[2]) - dRPYdotrad[2] * cos(dRPYrad[1]) * sin(dRPYrad[2]);
    matBdot.element_[1] = -dRPYdotrad[2] * cos(dRPYrad[2]);
    matBdot.element_[2] = 0.0;

    matBdot.element_[3] = -dRPYdotrad[1] * sin(dRPYrad[1]) * sin(dRPYrad[2]) + dRPYdotrad[2] * cos(dRPYrad[1]) * cos(dRPYrad[2]);
    matBdot.element_[4] = -dRPYdotrad[2] * sin(dRPYrad[2]);
    matBdot.element_[5] = 0.0;

    matBdot.element_[6] = -dRPYdotrad[1] * cos(dRPYrad[1]);
    matBdot.element_[7] = 0.0;
    matBdot.element_[8] = 0.0;

    ModelMatrix vecRPYAcc(3, 1);
    vecRPYAcc = matB.inverse() * (vecAngularAccrad - matBdot * vecRPYdotrad);

    for (int i = 0; i < 3; i++) rpy_acc[i] = vecRPYAcc.element_[i] * kRad2Deg;
}

void RobotModel::end2RPY(double dPoseEndFrame[6], double dPoseRPY[6]) {
    ModelMatrix matRotz(4, 4);
    ModelMatrix matRotx(4, 4);
    ModelMatrix matRoty(4, 4);

    // Rz +0 회전
    matRotz.element_[0] = 1.0;
    matRotz.element_[1] = 0.0;
    matRotz.element_[2] = 0.0;
    matRotz.element_[3] = 0.0;
    matRotz.element_[4] = 0.0;
    matRotz.element_[5] = 1.0;
    matRotz.element_[6] = 0.0;
    matRotz.element_[7] = 0.0;
    matRotz.element_[8] = 0.0;
    matRotz.element_[9] = 0.0;
    matRotz.element_[10] = 1.0;
    matRotz.element_[11] = 0.0;
    matRotz.element_[12] = 0.0;
    matRotz.element_[13] = 0.0;
    matRotz.element_[14] = 0.0;
    matRotz.element_[15] = 1.0;

    // Transformation matrix
    ModelMatrix matTrans = transformationMatrix(0);
    for (int i = 1; i < JS_DOF; i++) matTrans = matTrans * transformationMatrix(i);

    ModelMatrix tcpMat = makeTcpMatrix();

    matTrans = matTrans * tcpMat;

    // 말단 좌표계 업데이트 (이 함수 내에서는 의미없음)
    ModelMatrix matTransB2E(4, 4);
    matTransB2E = matTrans * matRotz;

    ModelMatrix vecPosEF(4, 1);
    for (int i = 0; i < 3; i++) vecPosEF.setElement_plus(i + 1, 1, dPoseEndFrame[i]);
    vecPosEF.element_[3] = 1.0;

    // 말단 좌표 기준의 목표점을 RPY 기준으로 변환
    ModelMatrix vecPosBF(4, 1);
    vecPosBF = matTransB2E * vecPosEF;

    // Translation vector
    for (int i = 0; i < 3; i++) dPoseRPY[i] = vecPosBF.getElement_plus(i + 1, 1);

    // Orientation 변환
    double Rx = dPoseEndFrame[3] * kDeg2Rad;
    double Ry = dPoseEndFrame[4] * kDeg2Rad;
    double Rz = dPoseEndFrame[5] * kDeg2Rad;

    matRotx.element_[0] = 1.0;
    matRotx.element_[1] = 0.0;
    matRotx.element_[2] = 0.0;
    matRotx.element_[3] = 0.0;
    matRotx.element_[4] = 0.0;
    matRotx.element_[5] = cos(Rx);
    matRotx.element_[6] = -sin(Rx);
    matRotx.element_[7] = 0.0;
    matRotx.element_[8] = 0.0;
    matRotx.element_[9] = sin(Rx);
    matRotx.element_[10] = cos(Rx);
    matRotx.element_[11] = 0.0;
    matRotx.element_[12] = 0.0;
    matRotx.element_[13] = 0.0;
    matRotx.element_[14] = 0.0;
    matRotx.element_[15] = 1.0;

    matRoty.element_[0] = cos(Ry);
    matRoty.element_[1] = 0.0;
    matRoty.element_[2] = sin(Ry);
    matRoty.element_[3] = 0.0;
    matRoty.element_[4] = 0.0;
    matRoty.element_[5] = 1.0;
    matRoty.element_[6] = 0.0;
    matRoty.element_[7] = 0.0;
    matRoty.element_[8] = -sin(Ry);
    matRoty.element_[9] = 0.0;
    matRoty.element_[10] = cos(Rx);
    matRoty.element_[11] = 0.0;
    matRoty.element_[12] = 0.0;
    matRoty.element_[13] = 0.0;
    matRoty.element_[14] = 0.0;
    matRoty.element_[15] = 1.0;

    matRotz.element_[0] = cos(Rz);
    matRotz.element_[1] = -sin(Rz);
    matRotz.element_[2] = 0.0;
    matRotz.element_[3] = 0.0;
    matRotz.element_[4] = sin(Rz);
    matRotz.element_[5] = cos(Rz);
    matRotz.element_[6] = 0.0;
    matRotz.element_[7] = 0.0;
    matRotz.element_[8] = 0.0;
    matRotz.element_[9] = 0.0;
    matRotz.element_[10] = 1.0;
    matRotz.element_[11] = 0.0;
    matRotz.element_[12] = 0.0;
    matRotz.element_[13] = 0.0;
    matRotz.element_[14] = 0.0;
    matRotz.element_[15] = 1.0;

    // 말단 좌표계를 기준으로 회전
    matTransB2E = matTrans * matRotz * matRoty * matRotx;

    // Orientation vector
    dPoseRPY[3] = atan2(matTransB2E.getElement_plus(3, 2), matTransB2E.getElement_plus(3, 3)) * kRad2Deg;
    dPoseRPY[4] = atan2(-matTransB2E.getElement_plus(3, 1), sqrt(matTransB2E.getElement_plus(3, 2) * matTransB2E.getElement_plus(3, 2) + matTransB2E.getElement_plus(3, 3) * matTransB2E.getElement_plus(3, 3))) * kRad2Deg;
    dPoseRPY[5] = atan2(matTransB2E.getElement_plus(2, 1), matTransB2E.getElement_plus(1, 1)) * kRad2Deg;
}

void RobotModel::base2RPY(double dPoseBaseFrame[6], double dPoseRPY[6]) {
    // x,y,z 회전행렬
    ModelMatrix matRotz(4, 4);
    ModelMatrix matRotx(4, 4);
    ModelMatrix matRoty(4, 4);

    double Rx = dPoseBaseFrame[3] * kDeg2Rad;
    double Ry = dPoseBaseFrame[4] * kDeg2Rad;
    double Rz = dPoseBaseFrame[5] * kDeg2Rad;

    matRotx.element_[0] = 1.0;
    matRotx.element_[1] = 0.0;
    matRotx.element_[2] = 0.0;
    matRotx.element_[3] = 0.0;
    matRotx.element_[4] = 0.0;
    matRotx.element_[5] = cos(Rx);
    matRotx.element_[6] = -sin(Rx);
    matRotx.element_[7] = 0.0;
    matRotx.element_[8] = 0.0;
    matRotx.element_[9] = sin(Rx);
    matRotx.element_[10] = cos(Rx);
    matRotx.element_[11] = 0.0;
    matRotx.element_[12] = 0.0;
    matRotx.element_[13] = 0.0;
    matRotx.element_[14] = 0.0;
    matRotx.element_[15] = 1.0;

    matRoty.element_[0] = cos(Ry);
    matRoty.element_[1] = 0.0;
    matRoty.element_[2] = sin(Ry);
    matRoty.element_[3] = 0.0;
    matRoty.element_[4] = 0.0;
    matRoty.element_[5] = 1.0;
    matRoty.element_[6] = 0.0;
    matRoty.element_[7] = 0.0;
    matRoty.element_[8] = -sin(Ry);
    matRoty.element_[9] = 0.0;
    matRoty.element_[10] = cos(Rx);
    matRoty.element_[11] = 0.0;
    matRoty.element_[12] = 0.0;
    matRoty.element_[13] = 0.0;
    matRoty.element_[14] = 0.0;
    matRoty.element_[15] = 1.0;

    matRotz.element_[0] = cos(Rz);
    matRotz.element_[1] = -sin(Rz);
    matRotz.element_[2] = 0.0;
    matRotz.element_[3] = 0.0;
    matRotz.element_[4] = sin(Rz);
    matRotz.element_[5] = cos(Rz);
    matRotz.element_[6] = 0.0;
    matRotz.element_[7] = 0.0;
    matRotz.element_[8] = 0.0;
    matRotz.element_[9] = 0.0;
    matRotz.element_[10] = 1.0;
    matRotz.element_[11] = 0.0;
    matRotz.element_[12] = 0.0;
    matRotz.element_[13] = 0.0;
    matRotz.element_[14] = 0.0;
    matRotz.element_[15] = 1.0;

    // Transformation matrix
    ModelMatrix matTrans = transformationMatrix(0);
    for (int i = 1; i < JS_DOF; i++) matTrans = matTrans * transformationMatrix(i);

    ModelMatrix tcpMat = makeTcpMatrix();

    matTrans = matTrans * tcpMat;

    // 절대 좌표계 기준 회전
    ModelMatrix matRPY2Base(4, 4);
    matRPY2Base = matRotz * matRoty * matRotx * matTrans;

    // Orientation vector (RPY)
    dPoseRPY[0] = dPoseBaseFrame[0];
    dPoseRPY[1] = dPoseBaseFrame[1];
    dPoseRPY[2] = dPoseBaseFrame[2];
    dPoseRPY[3] = atan2(matRPY2Base.getElement_plus(3, 2), matRPY2Base.getElement_plus(3, 3)) * kRad2Deg;
    dPoseRPY[4] = atan2(-matRPY2Base.getElement_plus(3, 1), sqrt(matRPY2Base.getElement_plus(3, 2) * matRPY2Base.getElement_plus(3, 2) + matRPY2Base.getElement_plus(3, 3) * matRPY2Base.getElement_plus(3, 3))) * kRad2Deg;
    dPoseRPY[5] = atan2(matRPY2Base.getElement_plus(2, 1), matRPY2Base.getElement_plus(1, 1)) * kRad2Deg;
}

void RobotModel::setFrictionModel(double coeff_coulomb[JS_DOF], double coeff_viscous[JS_DOF][5], double deadzone[JS_DOF], int number_of_elements[JS_DOF]) {
    friction_operator_.resize(JS_DOF);
    friction_operator_max_.resize(JS_DOF);
    friction_operator_coeff_.resize(JS_DOF);

    for (int i = 0; i < JS_DOF; i++) {
        friction_model_param_[i][0] = coeff_coulomb[i];
        friction_model_param_[i][1] = coeff_viscous[i][0];
        friction_model_param_[i][2] = coeff_viscous[i][1];
        friction_model_param_[i][3] = coeff_viscous[i][2];
        friction_model_param_[i][4] = coeff_viscous[i][3];
        friction_model_param_[i][5] = coeff_viscous[i][4];

        friction_operator_number_[i] = number_of_elements[i];
        friction_operator_[i].resize(number_of_elements[i]);
        friction_operator_max_[i].resize(number_of_elements[i]);
        friction_operator_coeff_[i].resize(number_of_elements[i]);

        for (int j = 0; j < number_of_elements[i]; j++) {
            friction_operator_max_[i][j] = 2 * deadzone[i] / number_of_elements[i] / (number_of_elements[i] + 1) * (j + 1);
            friction_operator_coeff_[i][j] = coeff_coulomb[i] / friction_operator_max_[i][j] / number_of_elements[i];
        }
    }
}

void RobotModel::calFrictionTorque([[maybe_unused]] double q_des[JS_DOF], double qd_des[JS_DOF]) {
    double dQ_d[JS_DOF] = { 0.0, };
    // double dQ[JS_DOF] = {0.0, };
    for (int i = 0; i < JS_DOF; i++) {
        dQ_d[i] = qd_des[i];
        // dQ[i] = q_des[i];
    }

    for (int i = 0; i < JS_DOF; i++) {
        if (dQ_d[i] > 0.0) {
            coulomb_friction_direction_[i] = 1;
        } else if (dQ_d[i] < 0.0) {
            coulomb_friction_direction_[i] = -1;
        } else {
            coulomb_friction_direction_[i] = 0;
        }
    }

    for (int i = 0; i < JS_DOF; i++) {
        friction_torque_[i] = friction_model_param_[i][0];

        double qd_temp = 1.0;

        for (int j = 0; j < 5; j++) {
            qd_temp *= fabs(dQ_d[i]);
            friction_torque_[i] += friction_model_param_[i][j + 1] * qd_temp;
        }

        friction_torque_[i] *= coulomb_friction_direction_[i];
    }
}

void RobotModel::getFrictionTorque(double torque[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        torque[i] = friction_torque_[i];
    }
}

void RobotModel::calFrictionTorqueHG(double q_des[JS_DOF], double q_des_prev[JS_DOF], double qd_des[JS_DOF]) {
    double dQ_d[JS_DOF] = { 0.0, };
	double dQ[JS_DOF] = { 0.0, };
	double dQp[JS_DOF] = { 0.0, };
	for (int i = 0; i < JS_DOF; i++) {
		dQ_d[i] = qd_des[i];
		dQ[i] = q_des[i];
		dQp[i] = q_des_prev[i];
	}

    for (int i = 0; i < JS_DOF; i++) {
        friction_torque_hg_[i] = 0.0;
        int sgn[friction_operator_number_[i]];
        double mag[friction_operator_number_[i]];
        for (int j = 0; j < friction_operator_number_[i]; j++) {
            mag[j] = dQ[i] - dQp[i] + friction_operator_[i][j];

            if (mag[j] > friction_operator_max_[i][j]) {
                friction_operator_[i][j] = friction_operator_max_[i][j];
            } else if (mag[j] < -friction_operator_max_[i][j]) {
                friction_operator_[i][j] = -friction_operator_max_[i][j];
            } else {
                friction_operator_[i][j] = mag[j];
            }

            friction_torque_hg_[i] += friction_operator_[i][j] * friction_operator_coeff_[i][j];
        }

        // Viscous
        double qd_temp = 1.0;
        double friction_viscous = 0;

        for (int j = 0; j < 5; j++) {
            qd_temp *= fabs(dQ_d[i]);
            friction_viscous += friction_model_param_[i][j + 1] * qd_temp;
        }

        if (dQ_d[i] > 0) {
            friction_torque_hg_[i] += friction_viscous;
        } else if (dQ_d[i] < 0) {
            friction_torque_hg_[i] -= friction_viscous;
        }
    }
}

void RobotModel::getFrictionTorqueHG(double torque[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        torque[i] = friction_torque_hg_[i];
    }
}

void RobotModel::calEffectiveMass(double dq[JS_DOF], double xd_des[6], double effective_mass[1]) {
    double dq_rad[JS_DOF];
    for (int i = 0; i < JS_DOF; i++) {
        dq_rad[i] = dq[i] * kDeg2Rad;
    }
    ModelMatrix matM = calInertiaMatrix(dq_rad);

    ModelMatrix matT = matM + *inertia_rotor_mat_;

    ModelMatrix matJ(6, JS_DOF);
    ModelMatrix matJvel(3, JS_DOF);
    calJacobianGeo(&matJ);
    ModelMatrix matC(3, 1);


    double unitvec[3] = { 0.0 };

    if (xd_des[0] * xd_des[0] + xd_des[1] * xd_des[1] + xd_des[2] * xd_des[2] == 0.0) {
        effective_mass[0] = 0.0;
    }
    else {
        unitvec[0] = xd_des[0] / (sqrt(xd_des[0] * xd_des[0] + xd_des[1] * xd_des[1] + xd_des[2] * xd_des[2]));
        unitvec[1] = xd_des[1] / (sqrt(xd_des[0] * xd_des[0] + xd_des[1] * xd_des[1] + xd_des[2] * xd_des[2]));
        unitvec[2] = xd_des[2] / (sqrt(xd_des[0] * xd_des[0] + xd_des[1] * xd_des[1] + xd_des[2] * xd_des[2]));


        for (int i = 0; i < 3; i++) matC.setElement_plus(i + 1, 1, unitvec[i]);

        double djacobian[3][6] = { {0.0,} };

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < JS_DOF; j++) {
                djacobian[i][j] = matJ.getElement_plus(i + 1, j + 1);
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < JS_DOF; j++) {
                matJvel.setElement_plus(i + 1, j + 1, djacobian[i][j]);
            }
        }

        ModelMatrix matY = matJvel * (matT.inverse()) * (matJvel.transpose());
        ModelMatrix matK = (matC.transpose()) * matY * matC;
        ModelMatrix matE = matK.inverse();

        effective_mass[0] = matE.getElement_plus(1, 1);
    }
}

void RobotModel::getPreviousQ(double q[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        q[i] = q_prev_[i] * kRad2Deg;
    }
}

void RobotModel::setPreviousQ(double q[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        q_prev_[i] = q[i] * kDeg2Rad;
    }
}

void RobotModel::getPreviousQd(double qd[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        qd[i] = qd_prev_[i] * kRad2Deg;
    }
}

void RobotModel::setPreviousQd(double qd[JS_DOF]) {
    for (int i = 0; i < JS_DOF; i++) {
        qd_prev_[i] = qd[i] * kDeg2Rad;
    }
}

void RobotModel::matrixmultiple(double matrix_a[9], double matrix_b[9], double matrix_c[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                matrix_c[3 * i + j] += matrix_a[3 * i + k] * matrix_b[3 * k + j];
            }
        }
    }
}

void RobotModel::pose2Matrix(double x[6], double t06_temp[16]) {
    double dEndRotation[3][3] = { {0.0, }, };

    rpy2rot(x + 3, dEndRotation);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            t06_temp[4 * i + j] = dEndRotation[i][j];
        }
        t06_temp[4 * i + 3] = x[i];
        t06_temp[4 * 3 + i] = 0;
    }
    t06_temp[4 * 3 + 3] = 1.0;
}

ModelMatrix RobotModel::pose2Matrix(double x[6]) {
    double t06_array[16] = { 0.0, };
    pose2Matrix(x, t06_array);
    return ModelMatrix(4, 4, t06_array);
}

double RobotModel::jointPositionRmsError(double q_des[JS_DOF], double q_meas[JS_DOF]) {
    double rms = 0.0;
    for (int i = 0; i < JS_DOF; i++) {
        double e = q_des[i] - q_meas[i];
        rms += pow(e * kDeg2Rad, 2);
    }
    return sqrt(rms);
}

double RobotModel::poseRmsError(double q_des[6], double q_meas[6]) {
    double desiredRotation[3][3];
    double measuredRotation[3][3];
    double poseError[6] = { 0.0, };
    double rms = 0.0;

    RobotModel::rpy2rot(q_des + 3, desiredRotation);
    RobotModel::rpy2rot(q_meas + 3, measuredRotation);
    for (int i = 0; i < 3; i++) {
        poseError[i] = q_des[i] - q_meas[i];
    }
    RobotModel::orientationError(desiredRotation, measuredRotation, poseError + 3);

    for (int i = 0; i < 3; i++) {
        rms += pow(poseError[i], 2);
        rms += pow(poseError[i + 3] * kDeg2Rad, 2);
    }

    return sqrt(rms);
}

void RobotModel::poseError(double q_des[6], double q_meas[6], double error[6]) {
    double desiredRotation[3][3];
    double measuredRotation[3][3];

    RobotModel::rpy2rot(q_des + 3, desiredRotation);
    RobotModel::rpy2rot(q_meas + 3, measuredRotation);
    for (int i = 0; i < 3; i++) {
        error[i] = q_des[i] - q_meas[i];
    }
    RobotModel::orientationError(desiredRotation, measuredRotation, error + 3);
}

ModelMatrix RobotModel::transformationMatrix(int joint_num) {
    double dsin_Q = sin(q_[joint_num]);
    double dcos_Q = cos(q_[joint_num]);
    double dsin_Alpha = sin(dh_[joint_num][0]);
    double dcos_Alpha = cos(dh_[joint_num][0]);
    double a = dh_[joint_num][1];
    double d = dh_[joint_num][2];

    double temp[4][4] = { {0.0,}, };

    temp[0][0] = dcos_Q;
    temp[0][1] = -dsin_Q * dcos_Alpha;
    temp[0][2] = dsin_Q * dsin_Alpha;
    temp[0][3] = a * dcos_Q;

    temp[1][0] = dsin_Q;
    temp[1][1] = dcos_Q * dcos_Alpha;
    temp[1][2] = -dcos_Q * dsin_Alpha;
    temp[1][3] = a * dsin_Q;

    temp[2][0] = 0;
    temp[2][1] = dsin_Alpha;
    temp[2][2] = dcos_Alpha;
    temp[2][3] = d;

    temp[3][0] = 0;
    temp[3][1] = 0;
    temp[3][2] = 0;
    temp[3][3] = 1;

    ModelMatrix MatTrans(4, 4, true);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            MatTrans.element_[i * 4 + j] = temp[i][j];
        }
    }
    return MatTrans;
}

ModelMatrix RobotModel::transformationMatrix(int joint_num, double q) {
    double dSinQ = sin(q);
    double dCosQ = cos(q);
    double dSinAlpha = sin(dh_[joint_num][0]);
    double dCosAlpha = cos(dh_[joint_num][0]);
    double a = dh_[joint_num][1];
    double d = dh_[joint_num][2];

    double temp[4][4] = { {0.0,}, };

    temp[0][0] = dCosQ;
    temp[0][1] = -dSinQ * dCosAlpha;
    temp[0][2] = dSinQ * dSinAlpha;
    temp[0][3] = a * dCosQ;

    temp[1][0] = dSinQ;
    temp[1][1] = dCosQ * dCosAlpha;
    temp[1][2] = -dCosQ * dSinAlpha;
    temp[1][3] = a * dSinQ;

    temp[2][0] = 0;
    temp[2][1] = dSinAlpha;
    temp[2][2] = dCosAlpha;
    temp[2][3] = d;

    temp[3][0] = 0;
    temp[3][1] = 0;
    temp[3][2] = 0;
    temp[3][3] = 1;

    ModelMatrix MatTrans(4, 4, true);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            MatTrans.element_[i * 4 + j] = temp[i][j];
        }
    }
    return MatTrans;
}

ModelMatrix RobotModel::makeTcpMatrix() {
    ModelMatrix tcp_matrix(4, 4, true);
    double rot[3][3] = { {0.0,}, };

    rpy2rot(tcp_ + 3, rot);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tcp_matrix.element_[i * 4 + j] = rot[i][j];
        }
        tcp_matrix.element_[i * 4 + 3] = tcp_[i];
    }
    tcp_matrix.element_[3 * 4 + 3] = 1.0;

    return tcp_matrix;
}
