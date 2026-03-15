#include "path_planner.hpp"

PathPlanner::PathPlanner() {
    PathPlanner(true, 6);
}

PathPlanner::PathPlanner(bool is_cartesian_space, const unsigned int dof)
    : dof_(dof), degree_(5), is_cartesian_space_(is_cartesian_space), path_type_(PathType::NONE) {

    init_.reset(dof_);
    current_.reset(dof_);
    finish_.reset(dof_);
    finish_.time = init_.time + 1.0;
    target_.reset(dof_);
    desired_.reset(dof_);
    limit_.reset(dof_);
}

PathPlanner::~PathPlanner() {
}

void PathPlanner::setInitPosition(const double init_time, const vector<double> position) {
    path_type_ = PathType::NONE;

    init_.position = position;
    init_.velocity.resize(dof_);
    init_.acceleration.resize(dof_);

    current_.position = position;
    current_.velocity.resize(dof_);
    current_.acceleration.resize(dof_);

    finish_.position = position;
    finish_.velocity.resize(dof_);
    finish_.acceleration.resize(dof_);

    desired_.position = position;
    desired_.velocity.resize(dof_);
    desired_.acceleration.resize(dof_);

    init_.time = init_time;
    current_.time = init_time;
    finish_.time = init_time + 1.0;
}

void PathPlanner::setInitPosition(const double init_time, const double position) {
    if (dof_ == 1) {
        path_type_ = PathType::NONE;

        init_.position[0] = position;
        init_.velocity.resize(dof_);
        init_.acceleration.resize(dof_);

        current_.position[0] = position;
        current_.velocity.resize(dof_);
        current_.acceleration.resize(dof_);

        finish_.position[0] = position;
        finish_.velocity.resize(dof_);
        finish_.acceleration.resize(dof_);

        desired_.position[0] = position;
        desired_.velocity.resize(dof_);
        desired_.acceleration.resize(dof_);

        init_.time = init_time;
        current_.time = init_time;
        finish_.time = init_time + 1.0;
    }
}

void PathPlanner::setTargetVelocity(const vector<double> target_velocity) {
    target_.velocity = target_velocity;
}

void PathPlanner::setTargetAcceleration(const vector<double> target_acceleration) {
    target_.acceleration = target_acceleration;
}

void PathPlanner::setPolynominalDegree(const int degree) {
    degree_ = degree;
    if (!(degree_ == 5) && !(degree_ == 7)) {
        degree_ = 5;
    }
}

void PathPlanner::genNoPath(const vector<double> position) {
    init_.position = position;
    path_type_ = PathType::NONE;
}

void PathPlanner::genStopPath(const double stop_time) {
    vector<vector<double>> coefficient;
    init_.position = current_.position;
    init_.velocity = current_.velocity;
    degree_ = 5;

    double stop_time_temp = 0.0;
    vector<double> zero_velocity(dof_);

    if (stop_time != 0.0) {
        stop_time_temp = stop_time;
        coefficient = calPolynominalCoefficient(init_.position, init_.velocity, zero_velocity, stop_time_temp);
    }
    else {
        for (unsigned int i = 0; i < dof_; i++) {
            double temp = divideWithFloatingError(fabs(init_.velocity[i]), 2.0 / 3.0 * target_.acceleration[i]);
            if (temp >= stop_time_temp) {
                stop_time_temp = temp;
            }
        }
        coefficient = calPolynominalCoefficient(init_.position, init_.velocity, zero_velocity, stop_time_temp);
    }

    vector<double> waypoint(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        waypoint[i] = init_.position[i] + init_.velocity[i] / 2.0 * stop_time_temp;
        if (is_cartesian_space_ && i >= 3) {
            if (waypoint[i] > 180.0) {
                waypoint[i] = waypoint[i] - 360.0;
            }
            else if (waypoint[i] < -180.0) {
                waypoint[i] = waypoint[i] + 360.0;
            }
        }
    }

    stop_parameter_.coefficient = coefficient;

    init_.time = current_.time;
    finish_.time = init_.time + stop_time_temp;
    finish_.position = waypoint;

    path_type_ = PathType::STOP;
}

void PathPlanner::genTrapezoidPath(const vector<double> waypoint) {
    // set init and final state
    init_.position = current_.position;
    init_.velocity = current_.velocity;
    finish_.position = waypoint;

    vector<double> distance(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        distance[i] = finish_.position[i] - current_.position[i];
        // check orientation divergence error in cartesian space
        if (is_cartesian_space_ && i >= 3) {
            if (distance[i] > 180.0) {
                distance[i] = distance[i] - 360.0;
            }
            else if (distance[i] < -180.0) {
                distance[i] = distance[i] + 360.0;
            }
        }
    }

    // if target velocity is zero, generate no path
    vector<double> check_velocity(dof_);
    check_velocity = target_.velocity;
    if (isEqual(check_velocity[0], 0.0) && isEqual(check_velocity[1], 0.0) && isEqual(check_velocity[2], 0.0) && isEqual(check_velocity[3], 0.0) && isEqual(check_velocity[4], 0.0) && isEqual(check_velocity[5], 0.0)) {
        genNoPath(current_.position);
        return;
    }

    // first step: determinate trapezoidal velocity profile or triangle velocity profile
    // max_velocity: v'max  - [joint num or cartesian num(dof_)]
    // uniform_velocity: vc - [joint num or cartesian num(dof_)]
    vector<double> max_velocity;
    vector<double> uniform_velocity;
    max_velocity = calMaxVelocity(init_.velocity, finish_.velocity, distance, target_.acceleration);
    uniform_velocity = compareVelocity(max_velocity, target_.velocity);

    // second, third step: find longest path time along all joint or cartesian space
    // time_sum: Ts
    double time_sum = 0.0;
    time_sum = compareTimeSum(init_.velocity, finish_.velocity, uniform_velocity, distance, target_.acceleration);

    // fourth step: recalculate uniform velocity acording to time_sum
    // uniform_velocity: v'c - [joint num or cartesian num(dof_)]
    uniform_velocity = calUniformVelocity(time_sum, init_.velocity, finish_.velocity, distance, target_.acceleration);

    // fifth step: calculate acc, uni, dec path time segment
    // path_time_segment: path time segment - [joint num or cartesian num(dof_)][acc, uni, dec(3)]
    vector<vector<double>> path_time_segment;
    path_time_segment = calPathTime(init_.velocity, finish_.velocity, uniform_velocity, time_sum, target_.acceleration);

    // sixth step: calculate polynominal coefficient
    // coefficient: ai, li, bi - [acc, uni, dec(3)][joint num or cartesian num(dof_)][polynominal_coefficient(degree_)]
    vector<vector<vector<double>>> coefficient;
    coefficient = calTrapezoidCoefficient(init_.position, finish_.position, init_.velocity, finish_.velocity, uniform_velocity, path_time_segment);

    // set path time
    // path_time: acc start, uni start, dec start, path finish time - [joint num or cartesian num(dof_)][acc start, uni start, dec start, path finish time(4)]
    vector<vector<double>> path_time(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        path_time[i].resize(4);
        path_time[i][0] = current_.time;
        for (int j = 0; j < 3; j++) {
            path_time[i][j + 1] = path_time[i][j] + path_time_segment[i][j];
        }
    }

    // globalize trapezoid parameter to calculate path
    trapezoid_Parameter_.path_time = path_time;
    trapezoid_Parameter_.coefficient = coefficient;

    // set init and final state
    init_.time = trapezoid_Parameter_.path_time[0][0];
    finish_.time = trapezoid_Parameter_.path_time[0][3];

    // set path type
    path_type_ = PathType::TRAPEZOID;
}

PathPlanner::PathError PathPlanner::genPolynomialPath(const vector<double> waypoint) {
    // set init and final state
    init_.position = current_.position;
    init_.velocity = current_.velocity;
    finish_.position = waypoint;

    vector<double> distance(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        distance[i] = finish_.position[i] - current_.position[i];
        // check orientation divergence error in cartesian space
        if (is_cartesian_space_ && i >= 3) {
            if (distance[i] > 180) {
                distance[i] = distance[i] - 360;
            }
            else if (distance[i] < -180) {
                distance[i] = distance[i] + 360;
            }
        }
    }

    PathError path_error;
    //  error_type -> PathErrorType::ZERO_TARGET_VEL
    if (isEqual(target_.velocity[0], 0.0) && isEqual(target_.velocity[1], 0.0) && isEqual(target_.velocity[2], 0.0) && isEqual(target_.velocity[3], 0.0) && isEqual(target_.velocity[4], 0.0) && isEqual(target_.velocity[5], 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_VEL;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genStopPath(0.2);
        return path_error;
    }

    //  error_type -> PathErrorType::ZERO_TARGET_ACC
    if (isEqual(target_.acceleration[0], 0.0) && isEqual(target_.acceleration[1], 0.0) && isEqual(target_.acceleration[2], 0.0) && isEqual(target_.acceleration[3], 0.0) && isEqual(target_.acceleration[4], 0.0) && isEqual(target_.acceleration[5], 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_ACC;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genStopPath(0.2);
        return path_error;
    }

    // reduce target acceleration so that it does not exceed target acceleration
    vector<double> polynominal_acceleration(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        if (degree_ == 5) {
            polynominal_acceleration[i] = target_.acceleration[i] * 2.0 / 3.0;
        }
        else if (degree_ == 7) {
            polynominal_acceleration[i] = target_.acceleration[i] * 8.0 / 15.0;
        }
        else {
            genNoPath(current_.position);
        }
    }

    // first step: determinate trapezoidal velocity profile or triangle velocity profile
    // max_velocity: v'max  - [joint num or cartesian num(dof_)]
    // uniform_velocity: vc - [joint num or cartesian num(dof_)]
    vector<double> max_velocity;
    vector<double> uniform_velocity;
    max_velocity = calMaxVelocity(init_.velocity, finish_.velocity, distance, polynominal_acceleration);
    uniform_velocity = compareVelocity(max_velocity, target_.velocity);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //가장긴 구동 시간 계산
    vector<double> path_time(3);
    path_time = compareTime(init_.velocity, finish_.velocity, uniform_velocity, distance, polynominal_acceleration);

    //시간 동기화
    uniform_velocity = recalUniformVelocity(init_.velocity, finish_.velocity, distance, path_time);

    //5차 가감속 구간, 등속 구간 coefficient 계산
    vector<vector<vector<vector<double>>>> coefficient(1);
    coefficient[0] = calPathCoefficient(init_.position, finish_.position, init_.velocity, finish_.velocity, uniform_velocity, path_time);
    //동기화된 시간을 저장
    vector<double> path_time_sum(4);

    for (int i = 0; i < 3; i++) {
        path_time_sum[i + 1] = path_time_sum[i] + path_time[i];
    }
    for (int i = 0; i < 4; i++) {
        path_time_sum[i] = path_time_sum[i] + current_.time;
    }

    vector<vector<double>> path_time_temp(dof_);

    for (int i = 0; i < dof_; i++) {
        path_time_temp[i] = path_time_sum;
    }

    polynomial_parameter_.path_time.resize(1);
    polynomial_parameter_.path_time[0] = path_time_temp;
    polynomial_parameter_.coefficients = coefficient;
    polynomial_parameter_.count_num = 0;

    //초기, 종료 시간 저장
    init_.time = polynomial_parameter_.path_time[0][0][0];
    finish_.time = polynomial_parameter_.path_time[0][0][3];

    path_type_ = PathType::POLYNOMIAL;

    return path_error;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DEPRECATED: 전체 궤적 시간만을 동기화
    // // second, third step: find longest path time along all joint or cartesian space
    // // time_sum: Ts
    // double time_sum = 0.0;
    // time_sum = compareTimeSum(init_.velocity, finish_.velocity, uniform_velocity, distance, polynominal_acceleration);

    // // fourth step: recalculate uniform velocity acording to time_sum
    // // uniform_velocity: v'c - [joint num or cartesian num(dof_)]
    // uniform_velocity = calUniformVelocity(time_sum, init_.velocity, finish_.velocity, distance, polynominal_acceleration);

    // // fifth step: calculate acc, uni, dec path time segment
    // // path_time_segment: path time segment - [joint num or cartesian num(dof_)][acc, uni, dec(3)]
    // vector<vector<double>> path_time_segment;
    // path_time_segment = calPathTime(init_.velocity, finish_.velocity, uniform_velocity, time_sum, polynominal_acceleration);

    // // sixth step: calculate polynominal coefficient
    // // coefficient: ai, li, bi - [waypoint num(1)][acc, uni, dec(3)][joint num or cartesian num(dof_)][polynominal_coefficient(degree_)]
    // vector<vector<vector<vector<double>>>> coefficient;
    // coefficient.resize(1);
    // coefficient[0] = calPathCoefficient(init_.position, finish_.position, init_.velocity, finish_.velocity, uniform_velocity, path_time_segment);

    // // set path time
    // // path_time: acc start, uni start, dec start, path finish time - [joint num or cartesian num(dof_)][acc start, uni start, dec start, path finish time(4)]
    // vector<vector<double>> path_time(dof_);
    // for (unsigned int i = 0; i < dof_; i++) {
    //     path_time[i].resize(4);
    //     path_time[i][0] = current_.time;
    //     for (int j = 0; j < 3; j++) {
    //         path_time[i][j + 1] = path_time[i][j] + path_time_segment[i][j];
    //     }
    // }

    // // globalize polynominal parameter to calculate path
    // polynomial_parameter_.path_time.resize(1);
    // polynomial_parameter_.path_time[0] = path_time;
    // polynomial_parameter_.coefficients = coefficient;
    // polynomial_parameter_.count_num = 0;

    // // set init and final state
    // init_.time = polynomial_parameter_.path_time[0][0][0];
    // finish_.time = polynomial_parameter_.path_time[0][0][3];

    // // set path type
    // path_type_ = PathType::POLYNOMIAL;

    // return path_error;
}

PathPlanner::PathError PathPlanner::genPolynomialPath(const vector<vector<double>> waypoints, const vector<vector<double>> target_velocities, const vector<vector<double>> target_accelerations, const vector<vector<double>> finish_velocities) {
    // set waypoint number
    unsigned int waypoints_num = waypoints.size();

    // set init and final state
    init_.position = current_.position;
    init_.velocity = current_.velocity;
    finish_.position = waypoints[waypoints_num - 1];

    // check path error
    PathError path_error;
    //  error_type -> PathErrorType::MAX_WAYPOINT
    if (waypoints_num > WAYPOINT_LIMIT) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MAX_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(WAYPOINT_LIMIT);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genStopPath(0.2);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_VEL
    for (unsigned int i = 0; i < waypoints_num; i++) {
        if (isEqual(target_velocities[i][0], 0.0) || isEqual(target_velocities[i][1], 0.0) || isEqual(target_velocities[i][2], 0.0) || isEqual(target_velocities[i][3], 0.0) || isEqual(target_velocities[i][4], 0.0) || isEqual(target_velocities[i][5], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::ZERO_TARGET_VEL;
            path_error_parameter.waypoint_index.push_back(i);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::ZERO_TARGET_VEL) {
        genStopPath(0.2);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_ACC
    for (unsigned int i = 0; i < waypoints_num; i++) {
        if (isEqual(target_accelerations[i][0], 0.0) || isEqual(target_accelerations[i][1], 0.0) || isEqual(target_accelerations[i][2], 0.0) || isEqual(target_accelerations[i][3], 0.0) || isEqual(target_accelerations[i][4], 0.0) || isEqual(target_accelerations[i][5], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::ZERO_TARGET_ACC;
            path_error_parameter.waypoint_index.push_back(i);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::ZERO_TARGET_ACC) {
        genStopPath(0.2);
        return path_error;
    }

    // path_time_segment: path time segment - [waypoints_num][joint num or cartesian num(dof_)][acc, uni, dec(3)]
    vector<vector<vector<double>>> path_time_segment(waypoints_num);
    // coefficient: ai, li, bi - [waypoints num][acc, uni, dec(3)][joint num or cartesian num(dof_)][polynominal_coefficient(degree_)]
    vector<vector<vector<vector<double>>>> coefficient(waypoints_num);

    for (unsigned int i = 0; i < waypoints_num; i++) {
        vector<double> init_position(dof_);
        vector<double> finish_position(dof_);
        vector<double> init_velocity(dof_);
        vector<double> finish_velocity(dof_);
        vector<double> distance(dof_);
        vector<double> target_acceleration(dof_);

        if (i == 0) {
            // current to waypoint[0]
            init_position = init_.position;
            finish_position = waypoints[i];
            init_velocity = init_.velocity;
            finish_velocity = finish_velocities[i];

            for (unsigned int j = 0; j < dof_; j++) {
                distance[j] = finish_position[j] - init_position[j];
                // check orientation divergence error in cartesian space
                if (is_cartesian_space_ && i >= 3) {
                    if (distance[j] > 180) {
                        distance[j] = distance[j] - 360;
                    }
                    else if (distance[i] < -180) {
                        distance[j] = distance[j] + 360;
                    }
                }
            }
        }
        else {
            // waypoint[i - 1] to waypoint[i]
            init_position = waypoints[i - 1];
            finish_position = waypoints[i];
            init_velocity = finish_velocities[i - 1];
            finish_velocity = finish_velocities[i];

            for (unsigned int j = 0; j < dof_; j++) {
                distance[j] = finish_position[j] - init_position[j];
                // check rotation error in cartesian space
                if (is_cartesian_space_ && i >= 3) {
                    if (distance[j] > 180) {
                        distance[j] = distance[j] - 360;
                    }
                    else if (distance[i] < -180) {
                        distance[j] = distance[j] + 360;
                    }
                }
            }
        }

        // reduce target acceleration so that it does not exceed target acceleration
        for (unsigned int j = 0; j < dof_; j++) {
            if (degree_ == 5) {
                target_acceleration[j] = target_accelerations[i][j] * 2.0 / 3.0;
            }
            else if (degree_ == 7) {
                target_acceleration[j] = target_accelerations[i][j] * 8.0 / 15.0;
            }
        }

        // first step: determinate trapezoidal velocity profile or triangle velocity profile
        // max_velocity: v'max  - [joint num or cartesian num(dof_)]
        // uniform_velocity: vc - [joint num or cartesian num(dof_)]
        vector<double> max_velocity(dof_);
        vector<double> uniform_velocity(dof_);
        max_velocity = calMaxVelocity(init_velocity, finish_velocity, distance, target_acceleration);
        uniform_velocity = compareVelocity(max_velocity, target_velocities[i]);

        // second, third step: find longest path time along all joint or cartesian space
        // time_sum: Ts
        double time_sum = 0.0;
        time_sum = compareTimeSum(init_velocity, finish_velocity, uniform_velocity, distance, target_acceleration);

        // fourth step: recalculate uniform velocity acording to time_sum
        // uniform_velocity: v'c - [joint num or cartesian num(dof_)]
        uniform_velocity = calUniformVelocity(time_sum, init_velocity, finish_velocity, distance, target_acceleration);

        // fifth step: calculate acc, uni, dec path time segment
        // path_time_segment[i]: path time segment - [joint num or cartesian num(dof_)][acc, uni, dec(3)]
        path_time_segment[i] = calPathTime(init_velocity, finish_velocity, uniform_velocity, time_sum, target_acceleration);

        // sixth step: calculate polynominal coefficient
        // coefficient[i]: ai, li, bi - [waypoint num(1)][acc, uni, dec(3)][joint num or cartesian num(dof_)][polynominal_coefficient(degree_)]
        coefficient[i] = calPathCoefficient(init_position, finish_position, init_velocity, finish_velocity, uniform_velocity, path_time_segment[i]);
    }

    // set path time
    // path_time: acc start, uni start, dec start, path finish time - [joint num or cartesian num(dof_)][acc start, uni start, dec start, path finish time(4)]
    vector<vector<vector<double>>> path_time(waypoints_num);
    for (unsigned int i = 0; i < waypoints_num; i++) {
        path_time[i].resize(dof_);
        for (unsigned int j = 0; j < dof_; j++) {
            path_time[i][j].resize(4);
            path_time[0][j][0] = current_.time;
        }
    }
    for (unsigned int i = 0; i < waypoints_num; i++) {
        for (unsigned int j = 0; j < dof_; j++) {
            for (int k = 0; k < 3; k++) {
                path_time[i][j][k + 1] = path_time[i][j][k] + path_time_segment[i][j][k];
            }
            if (i != waypoints_num - 1) {
                path_time[i + 1][j][0] = path_time[i][j][3];
            }
        }
    }

    // globalize polynominal parameter to calculate path
    polynomial_parameter_.path_time = path_time;
    polynomial_parameter_.coefficients = coefficient;
    polynomial_parameter_.count_num = 0;

    // set init and final state
    init_.time = path_time[0][0][0];
    finish_.time = path_time[waypoints_num - 1][0][3];

    // set path type
    path_type_ = PathType::POLYNOMIAL;

    return path_error;
}

PathPlanner::PathError PathPlanner::genLinearPath(const vector<double> waypoint) {
    // set init and final state
    init_.position = current_.position;
    init_.velocity = current_.velocity;
    finish_.position = waypoint;

    PathError path_error;
    // error_type -> PathErrorType::INIT_VEL
    for (unsigned int i = 0; i < dof_; i++) {
        if (!isEqual(init_.velocity[i], 0.0, 3)) {
            path_error.type = PathErrorType::INIT_VEL;
        }
    }
    if (path_error.type == PathErrorType::INIT_VEL) {
        genStopPath(0.2);
        return path_error;
    }

    //  error_type -> PathErrorType::ZERO_TARGET_VEL
    if (isEqual(target_.velocity[0], 0.0) || isEqual(target_.velocity[3], 0.0) || isEqual(target_.velocity[4], 0.0) || isEqual(target_.velocity[5], 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_VEL;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    //  error_type -> PathErrorType::ZERO_TARGET_ACC
    if (isEqual(target_.acceleration[0], 0.0) || isEqual(target_.acceleration[3], 0.0) || isEqual(target_.acceleration[4], 0.0) || isEqual(target_.acceleration[5], 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_ACC;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    vector<double> distance(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        distance[i] = finish_.position[i] - init_.position[i];
        // check orientation divergence error in cartesian space
        if (is_cartesian_space_ && i >= 3) {
            if (distance[i] > 180.0) {
                distance[i] = distance[i] - 360.0;
            }
            else if (distance[i] < -180.0) {
                distance[i] = distance[i] + 360.0;
            }
        }
    }

    // eq (4.1.1)
    vector<double> distance_temp(distance.begin(), distance.begin() + 3);
    vector<double> distance_unit_vector = calVectorNomalize(distance_temp);

    // x, y, z, R, P, Y (6) -> linear, R, P, Y (4)
    vector<double> linear_distance(4);
    vector<double> init_position(4);
    vector<double> finish_position(4);
    vector<double> target_velocity(4);
    vector<double> init_velocity(4);
    vector<double> finish_velocity(4);
    vector<double> target_acceleration(4);

    // x, y, z -> linear
    init_position[0] = 0.0;
    linear_distance[0] = sqrt(pow(distance[0], 2.0) + pow(distance[1], 2.0) + pow(distance[2], 2.0));
    finish_position[0] = init_position[0] + linear_distance[0];
    init_velocity[0] = sqrt(pow(init_.velocity[0], 2.0) + pow(init_.velocity[1], 2.0) + pow(init_.velocity[2], 2.0));
    finish_velocity[0] = sqrt(pow(finish_.velocity[0], 2.0) + pow(finish_.velocity[1], 2.0) + pow(finish_.velocity[2], 2.0));
    target_velocity[0] = target_.velocity[0];
    // reduce target acceleration so that it does not exceed target acceleration
    if (degree_ == 5) {
        target_acceleration[0] = target_.acceleration[0] * 2.0 / 3.0;
    }
    else if (degree_ == 7) {
        target_acceleration[0] = target_.acceleration[0] * 8.0 / 15.0;
    }

    // set orientation in linear path
    for (int i = 0; i < 3; i++) {
        init_position[i + 1] = init_.position[i + 3];
        finish_position[i + 1] = finish_.position[i + 3];
        init_velocity[i + 1] = init_.velocity[i + 3];
        finish_velocity[i + 1] = finish_.velocity[i + 3];
        target_velocity[i + 1] = target_.velocity[i + 3];
        linear_distance[i + 1] = distance[i + 3];
        // reduce target acceleration so that it does not exceed target acceleration
        if (degree_ == 5) {
            target_acceleration[i + 1] = target_.acceleration[i + 3] * 2.0 / 3.0;
        }
        else if (degree_ == 7) {
            target_acceleration[i + 1] = target_.acceleration[i + 3] * 8.0 / 15.0;
        }
    }

    // first step: determinate trapezoidal velocity profile or triangle velocity profile
    // max_velocity: v'max  - [linear and orientation(4)]
    // uniform_velocity: vc - [linear and orientation(4)]
    vector<double> max_velocity;
    vector<double> uniform_velocity;
    max_velocity = calMaxVelocity(init_velocity, finish_velocity, linear_distance, target_acceleration);
    uniform_velocity = compareVelocity(max_velocity, target_velocity);

    // second, third step: find longest path time along all joint or cartesian space
    // time_sum: Ts
    double time_sum = 0.0;
    time_sum = compareTimeSum(init_velocity, finish_velocity, uniform_velocity, linear_distance, target_acceleration);

    // fourth step: recalculate uniform velocity acording to time_sum
    // uniform_velocity: v'c - [linear and orientation(4)]
    uniform_velocity = calUniformVelocity(time_sum, init_velocity, finish_velocity, linear_distance, target_acceleration);

    // fifth step: calculate acc, uni, dec path time segment
    // path_time_segment: path time segment - [linear and orientation(4)][acc, uni, dec(3)]
    vector<vector<double>> path_time_segment;
    path_time_segment = calPathTime(init_velocity, finish_velocity, uniform_velocity, time_sum, target_acceleration);

    // sixth step: calculate polynominal coefficient
    // coefficient: ai, li, bi - [acc, uni, dec(3)][linear and orientation(4)][polynominal_coefficient(degree_)]
    vector<vector<vector<double>>> coefficient;
    coefficient = calPathCoefficient(init_position, finish_position, init_velocity, finish_velocity, uniform_velocity, path_time_segment);

    // set path time
    // path_time: acc start, uni start, dec start, path finish time - [linear and orientation(4)][acc start, uni start, dec start, path finish time(4)]
    vector<vector<double>> path_time(4);
    for (int k = 0; k < 4; k++) {
        path_time[k].resize(4);
        path_time[k][0] = current_.time;
        for (int i = 0; i < 3; i++) {
            path_time[k][i + 1] = path_time[k][i] + path_time_segment[k][i];
        }
    }

    // globalize linear parameter to calculate path
    linear_parameter_.path_time = path_time;
    linear_parameter_.coefficients = coefficient;
    linear_parameter_.distance_unit_vector = distance_unit_vector;

    // set init and final state
    init_.time = linear_parameter_.path_time[0][0];
    finish_.time = linear_parameter_.path_time[0][3];

    // set path type
    path_type_ = PathType::LINEAR;

    // return error type and error parameter
    return path_error;
}

PathPlanner::PathError PathPlanner::genCircularPath(vector<vector<double>> waypoints, const CircularType circular_mode) {
    vector<vector<vector<double>>> start_coefficient;
    vector<vector<double>> start_path_time;

    PathError path_error = genCurrentToStartPath(waypoints[0], start_coefficient, start_path_time);
    if (path_error.type == PathErrorType::ZERO_TARGET_VEL || path_error.type == PathErrorType::ZERO_TARGET_ACC) {
        genStopPath(0.2);
        return path_error;
    }

    circular_parameter_.start_coefficient = start_coefficient;
    circular_parameter_.start_path_time = start_path_time;

    // figure 4.2.3 - D0, D1: waypoints distance
    vector<double> vector_d0(3);
    vector<double> vector_d1(3);
    for (int i = 0; i < 3; i++) {
        vector_d0[i] = waypoints[0][i] - waypoints[1][i];
        vector_d1[i] = waypoints[2][i] - waypoints[1][i];
    }

    // if circular mode is UNCONSTRAINT_RPY, consider the orientation distance
    vector<double> distance_rpy(3);
    if (circular_mode == CircularType::UNCONSTRAINT_RPY) {
        for (int i = 0; i < 3; i++) {
            distance_rpy[i] = (waypoints[2][i + 3] - waypoints[0][i + 3]);
        }
        // check orientation divergence error in cartesian space
        for (int i = 0; i < 3; i++) {
            if (distance_rpy[i] > 180.0) {
                distance_rpy[i] = distance_rpy[i] - 360.0;
            }
            else if (distance_rpy[i] < -180.0) {
                distance_rpy[i] = distance_rpy[i] + 360.0;
            }
        }
    }

    // if circular mode is TANGENT_RPY, consider the rotation matrix
    ModelMatrix init_rotation_matrix(3, 3);
    if (circular_mode == CircularType::TANGENT_RPY) {
        vector<double> init_RPY(3);
        for (int i = 0; i < 3; i++) {
            init_RPY[i] = waypoints[0][i + 3];
        }
        init_rotation_matrix = convertRPYtoRotationMatrix(init_RPY);
    }

    // equation (4.2.1) - N: normal vector
    /*
     *    N = D0 x D1
     */
    vector<double> vector_n(3);
    vector<double> unit_vector_n(3);
    vector_n = calCrossProduct(vector_d0, vector_d1);
    unit_vector_n = calVectorNomalize(vector_n);

    // equation (4.2.2) - U0, U1
    /*
     *          N x D0          D1 x N
     *    U0 = -------- , U1 = --------
     *         |N x D0|        |D1 x N|
    */
    vector<double> vector_u0(3);
    vector<double> vector_u1(3);
    vector<double> unit_vector_u0(3);
    vector<double> unit_vector_u1(3);
    vector_u0 = calCrossProduct(vector_n, vector_d0);
    vector_u1 = calCrossProduct(vector_d1, vector_n);
    unit_vector_u0 = calVectorNomalize(vector_u0);
    unit_vector_u1 = calVectorNomalize(vector_u1);

    // calculate midpoint between waypoints
    vector<double> mid_point0(3);
    vector<double> mid_point1(3);
    for (int i = 0; i < 3; i++) {
        mid_point0[i] = divideWithFloatingError(waypoints[0][i] + waypoints[1][i], 2.0);
        mid_point1[i] = divideWithFloatingError(waypoints[2][i] + waypoints[1][i], 2.0);
    }

    // equation (4.2.3), (4.2.4), (4.2.5), (4.2.6) - calculate center point
    /*
     *    alpa * U0 - beta * U1 = mid_point1 - mid_point0
     *
     *            U0[0] * (mid_point1[1] - mid_point0[1]) - U0[1] * (mid_point1[0] - mid_point0[0])
     *    beta = -----------------------------------------------------------------------------------
     *                                   U1[0] * U0[1] - U0[0] * U1[1]
     *
     *    center_point = mid_point1 + beta * U1
    */
    vector<double> center_point(3);
    double beta = divideWithFloatingError(unit_vector_u0[0] * (mid_point1[1] - mid_point0[1]) - (mid_point1[0] - mid_point0[0]) * unit_vector_u0[1], unit_vector_u1[0] * unit_vector_u0[1] - unit_vector_u0[0] * unit_vector_u1[1]);
    if (isEqual(beta, 0.0)) {
        beta = divideWithFloatingError(unit_vector_u0[1] * (mid_point1[2] - mid_point0[2]) - (mid_point1[1] - mid_point0[1]) * unit_vector_u0[2], unit_vector_u1[1] * unit_vector_u0[2] - unit_vector_u0[1] * unit_vector_u1[2]);
        for (int i = 0; i < 3; i++) {
            center_point[i] = mid_point1[i] + beta * unit_vector_u1[i];
        }
    }
    else {
        for (int i = 0; i < 3; i++) {
            center_point[i] = mid_point1[i] + beta * unit_vector_u1[i];
        }
    }

    // equation (4.2.7)
    /*
     *          waypoints0 - center_point          B1 x N
     *    B1 = --------------------------- , B0 = --------
     *         |waypoints0 - center_point|        |B1 x N|
    */
    vector<double> vector_b0(3);
    vector<double> vector_b1(3);
    vector<double> unit_vector_b0(3);
    vector<double> unit_vector_b1(3);
    for (int i = 0; i < 3; i++) {
        vector_b1[i] = waypoints[0][i] - center_point[i];
    }
    unit_vector_b1 = calVectorNomalize(vector_b1);
    vector_b0 = calCrossProduct(unit_vector_b1, vector_n);
    unit_vector_b0 = calVectorNomalize(vector_b0);

    // calculate radius
    /*
     *    radius = |B1|
    */
    double radius = calVectorMagnitude(vector_b1);

    // equation (4.3.9), (4.3.10), (4.3.11) - calculate theta
    /*
     *    T0 = waypoints0 - center_point
     *    T1 = waypoints1 - center_point
     *    T2 = waypoints2 - center_point
     *
     *                       T0^T . T1                         T1^T . T2
     *    theta0 = cos^(-1)(-----------)    theta1 = cos^(-1)(-----------)
     *                       |T0|*|T1|                         |T1|*|T2|
     *
     *    theta = theta0 + theta1
    */
    vector<double> vector_t0(3);
    vector<double> vector_t1(3);
    vector<double> vector_t2(3);
    double inner_product_t0t1 = 0.0;
    double inner_product_t1t2 = 0.0;

    for (int i = 0; i < 3; i++) {
        vector_t0[i] = waypoints[0][i] - center_point[i];
        vector_t1[i] = waypoints[1][i] - center_point[i];
        vector_t2[i] = waypoints[2][i] - center_point[i];
    }

    double t0_magnitued = calVectorMagnitude(vector_t0);
    double t1_magnitued = calVectorMagnitude(vector_t1);
    double t2_magnitued = calVectorMagnitude(vector_t2);

    for (int i = 0; i < 3; i++) {
        inner_product_t0t1 += vector_t0[i] * vector_t1[i];
        inner_product_t1t2 += vector_t1[i] * vector_t2[i];
    }

    double theta_1 = acos(divideWithFloatingError(inner_product_t0t1, t0_magnitued * t1_magnitued));
    double theta_2 = acos(divideWithFloatingError(inner_product_t1t2, t1_magnitued * t2_magnitued));
    double theta = theta_1 + theta_2;

    // error_type -> PathErrorType::SAME_WAYPOINTS
    vector<double> waypoints_distance(2);
    waypoints_distance[0] = calVectorMagnitude(vector_d0);
    waypoints_distance[1] = calVectorMagnitude(vector_d1);
    for (int i = 0; i < 2; i++) {
        if (isEqual(waypoints_distance[i], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::SAME_WAYPOINTS;
            path_error_parameter.waypoint_index.push_back(i);
            path_error_parameter.waypoint_index.push_back(i + 1);
            path_error.path_error_parameter.push_back(path_error_parameter);
            genNoPath(current_.position);
            return path_error;
        }
    }

    // error_type -> PathErrorType::COLINEAR
    if (isEqual(theta, M_PI) || isEqual(theta, 2 * M_PI)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::COLINEAR;
        path_error_parameter.waypoint_index.push_back(0);
        path_error_parameter.waypoint_index.push_back(1);
        path_error_parameter.waypoint_index.push_back(2);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // x, y, z, R, P, Y (6) -> circular, R, P, Y (4)
    vector<double> circular_velocity(4);
    vector<double> circular_acceleration(4);
    vector<double> init_circular_velocity(4);
    vector<double> finish_circular_velocity(4);
    vector<double> circular_distance(4);
    vector<double> init_circular_position(4);
    vector<double> finish_circular_position(4);

    // x, y, z -> circular
    circular_distance[0] = theta * kRad2Deg;
    init_circular_velocity[0] = 0.0;
    finish_circular_velocity[0] = 0.0;
    circular_velocity[0] = divideWithFloatingError(target_.velocity[0] * kRad2Deg, radius);
    if (degree_ == 5) {
        circular_acceleration[0] = divideWithFloatingError(target_.acceleration[0] * 2.0 / 3.0 * kRad2Deg, radius);
    }
    else if (degree_ == 7) {
        circular_acceleration[0] = divideWithFloatingError(target_.acceleration[0] * 8.0 / 15.0 * kRad2Deg, radius);
    }
    if (circular_parameter_.circular_mode == CircularType::TANGENT_RPY) {
        if (circular_velocity[0] > circular_velocity[1]) {
            circular_velocity[0] = circular_velocity[1];
        }
        if (circular_acceleration[0] > circular_acceleration[1]) {
            circular_acceleration[0] = circular_acceleration[1];
        }
    }
    finish_circular_position[0] = init_circular_position[0] + circular_distance[0];

    // set orientation in circular path
    for (int i = 0; i < 3; i++) {
        circular_distance[i + 1] = distance_rpy[i];
        init_circular_velocity[i + 1] = init_.velocity[i + 3];
        finish_circular_velocity[i + 1] = finish_.velocity[i + 3];

        circular_velocity[i + 1] = target_.velocity[i + 3];
        if (degree_ == 5) {
            circular_acceleration[i + 1] = target_.acceleration[i + 3] * 2.0 / 3.0;
        }
        else if (degree_ == 7) {
            circular_acceleration[i + 1] = target_.acceleration[i + 3] * 8.0 / 15.0;
        }
        init_circular_position[i + 1] = waypoints[0][i + 3];
        finish_circular_position[i + 1] = waypoints[2][i + 3];
    }

    // first step: determinate trapezoidal velocity profile or triangle velocity profile
    // max_velocity: v'max  - [circular and orientation(4)]
    // uniform_velocity: vc - [circular and orientation(4)]
    vector<double> max_circular_velocity;
    vector<double> uniform_velocity;
    max_circular_velocity = calMaxVelocity(init_circular_velocity, finish_circular_velocity, circular_distance, circular_acceleration);
    uniform_velocity = compareVelocity(max_circular_velocity, circular_velocity);

    // second, third step: find longest path time along all joint or cartesian space
    // time_sum: Ts
    double time_sum = 0.0;
    time_sum = compareTimeSum(init_circular_velocity, finish_circular_velocity, uniform_velocity, circular_distance, circular_acceleration);

    // fourth step: recalculate uniform velocity acording to time_sum
    // uniform_velocity: v'c - [circular and orientation(4)]
    uniform_velocity = calUniformVelocity(time_sum, init_circular_velocity, finish_circular_velocity, circular_distance, circular_acceleration);

    // fifth step: calculate acc, uni, dec path time segment
    // path_time_segment: path time segment - [circular and orientation(4)][acc, uni, dec(3)]
    vector<vector<double>> path_time_segment;
    path_time_segment = calPathTime(init_circular_velocity, finish_circular_velocity, uniform_velocity, time_sum, circular_acceleration);

    // sixth step: calculate polynominal coefficient
    // coefficient: ai, li, bi - [acc, uni, dec(3)][circular and orientation(4)][polynominal_coefficient(degree_)]
    vector<vector<vector<double>>> coefficient;
    coefficient = calPathCoefficient(init_circular_position, finish_circular_position, init_circular_velocity, finish_circular_velocity, uniform_velocity, path_time_segment);

    // set path time
    // path_time: acc start, uni start, dec start, path finish time - [circular and orientation(4)][acc start, uni start, dec start, path finish time(4)]
    vector<vector<double>> path_time(4);
    for (unsigned int k = 0; k < 4; k++) {
        path_time[k].resize(4);
        for (int j = 0; j < 3; j++) {
            path_time[k][j + 1] = path_time[k][j] + path_time_segment[k][j];
        }
    }

    // globalize circular parameter to calculate path
    circular_parameter_.circular_mode = circular_mode;
    circular_parameter_.path_time = path_time;
    circular_parameter_.init_velocity = init_circular_velocity;
    circular_parameter_.finish_velocity = finish_circular_velocity;
    circular_parameter_.uniform_velocity = uniform_velocity;
    circular_parameter_.acceleration = circular_acceleration;
    circular_parameter_.init_rotation_matrix = init_rotation_matrix;
    circular_parameter_.vector_b0 = unit_vector_b0;
    circular_parameter_.vector_b1 = unit_vector_b1;
    circular_parameter_.vector_n = unit_vector_n;
    circular_parameter_.center_point = center_point;
    circular_parameter_.radius = radius;
    circular_parameter_.coefficient = coefficient;

    // set init and final state
    init_.time = current_.time;
    finish_.time = init_.time + start_path_time[0][3] + path_time[0][3];
    init_.position = current_.position;
    if (circular_parameter_.circular_mode == CircularType::UNCONSTRAINT_RPY) {
        finish_.position = waypoints[2];
    }
    else if (circular_parameter_.circular_mode == CircularType::TANGENT_RPY) {
        ModelMatrix rotation_matrix(3, 3);
        ModelMatrix target_rotation_matrix(3, 3);
        vector<double> finish_RPY(3);

        rotation_matrix = convertNormalVectortoRotationMatrix(circular_distance[0], circular_parameter_.vector_n);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                target_rotation_matrix.set(i, j, rotation_matrix.get(i, 0) * circular_parameter_.init_rotation_matrix.get(0, j) + rotation_matrix.get(i, 1) * circular_parameter_.init_rotation_matrix.get(1, j) + rotation_matrix.get(i, 2) * circular_parameter_.init_rotation_matrix.get(2, j));
            }
        }

        finish_RPY[2] = atan2(target_rotation_matrix.get(1, 0), target_rotation_matrix.get(0, 0)) * kRad2Deg;
        double temp = sqrt(target_rotation_matrix.get(2, 1) * target_rotation_matrix.get(2, 1) + target_rotation_matrix.get(2, 2) * target_rotation_matrix.get(2, 2));
        finish_RPY[1] = atan2(-target_rotation_matrix.get(2, 0), temp) * kRad2Deg;
        finish_RPY[0] = atan2(target_rotation_matrix.get(2, 1), target_rotation_matrix.get(2, 2)) * kRad2Deg;
        for (int i = 0; i < 3; i++) {
            finish_.position[i] = waypoints[2][i];
            finish_.position[i + 3] = finish_RPY[i];
        }
    }

    // set path type
    path_type_ = PathType::CIRCULAR;

    // return error type and error parameter
    return path_error;
}

PathPlanner::PathError PathPlanner::genBSplineTrajectory(const int p_degree, const int path_law, const vector<vector<double>> waypoints, const vector<vector<double>> velocities, const double target_velocity, const double target_acceleration) {
    // bspline degree
    bSpline_parameter_.p_degree = p_degree + 1;
    // path law
    bSpline_parameter_.path_law = path_law;
    // sampling time period: 0.001
    bSpline_parameter_.time_period = 0.001;
    // waypoint number
    bSpline_parameter_.waypoint_num = waypoints.size();

    // check b-spline path error
    PathError path_error;
    // error_type -> PathErrorType::MAX_WAYPOINT, PathErrorType::MIN_WAYPOINT
    if (bSpline_parameter_.waypoint_num < kBSplineWaypointNumMin) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MIN_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(kBSplineWaypointNumMin);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }
    else if (bSpline_parameter_.waypoint_num > WAYPOINT_LIMIT) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MAX_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(WAYPOINT_LIMIT);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_VEL
    if (isEqual(target_velocity, 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_VEL;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_ACC
    if (isEqual(target_acceleration, 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_ACC;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // assume operation time
    double operation_time = 0.0;
    vector<double> waypoints_distance(bSpline_parameter_.waypoint_num - 1);
    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
        for (int j = 0; j < 3; j++) {
            waypoints_distance[i] += pow(waypoints[i + 1][j] - waypoints[i][j], 2.0);
        }
        waypoints_distance[i] = sqrt(waypoints_distance[i]);
        operation_time += divideWithFloatingError(waypoints_distance[i], target_velocity);
    }

    // error_type -> PathErrorType::SAME_WAYPOINTS
    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
        if (isEqual(waypoints_distance[i], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::SAME_WAYPOINTS;
            path_error_parameter.waypoint_index.push_back(i);
            path_error_parameter.waypoint_index.push_back(i + 1);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::SAME_WAYPOINTS) {
        genNoPath(current_.position);
        return path_error;
    }

    // generalize b-spline parameter
    bSpline_parameter_.operation_time = operation_time;
    if (bSpline_parameter_.p_degree % 2 == 0) {
        bSpline_parameter_.knot_number = 2 * (bSpline_parameter_.p_degree) + bSpline_parameter_.waypoint_num - 2;
    }
    else {
        bSpline_parameter_.knot_number = 2 * (bSpline_parameter_.p_degree) + bSpline_parameter_.waypoint_num - 1;
    }
    bSpline_parameter_.controlpoint_num = bSpline_parameter_.knot_number - bSpline_parameter_.p_degree;
    bSpline_parameter_.waypoints = waypoints;
    bSpline_parameter_.tangent_vector = velocities;

    // step 1: generate interpole parameter, knot vector
    vector<double> interpol_parameter(bSpline_parameter_.waypoint_num);
    vector<double> knot_vector;
    genBSplineInterpoleParameter(interpol_parameter, knot_vector);

    // if init velocity and finish velocity are zero, set tangent vector according to waypoints distances
    if (isEqual(pow(velocities[0][0], 2.0) + pow(velocities[0][1], 2.0) + pow(velocities[0][2], 2.0), 0.0)) {
        for (int i = 0; i < 3; i++) {
            bSpline_parameter_.tangent_vector[0][i] = divideWithFloatingError(waypoints[1][i] - waypoints[0][i], interpol_parameter[1] - interpol_parameter[0]);
        }
    }
    if (isEqual(pow(velocities[1][0], 2.0) + pow(velocities[1][1], 2.0) + pow(velocities[1][2], 2.0), 0.0)) {
        for (int i = 0; i < 3; i++) {
            bSpline_parameter_.tangent_vector[1][i] = divideWithFloatingError(waypoints[bSpline_parameter_.waypoint_num - 1][i] - waypoints[bSpline_parameter_.waypoint_num - 2][i], interpol_parameter[interpol_parameter.size() - 1] - interpol_parameter[interpol_parameter.size() - 2]);
        }
    }
    // step 2: generate basis function
    vector<vector<vector<double>>> basis_total;
    vector<vector<double>> basis;
    genBSplineBasisFunction(interpol_parameter, knot_vector, basis, basis_total);

    // step 3: generate basis derivative function
    vector<vector<double>> basis_first_derivative;
    vector<vector<double>> basis_second_derivative;
    vector<vector<double>> basis_thrid_derivative;
    vector<vector<double>> basis_fourth_derivative;
    genBSplineBasisFunctionDerivative(knot_vector, basis_total, basis_first_derivative, basis_second_derivative, basis_thrid_derivative, basis_fourth_derivative);

    // step 4: generate control point
    genBSplineControlPoint(basis, basis_first_derivative, basis_second_derivative, basis_thrid_derivative);

    // set init and final state
    init_.position = waypoints[0];
    finish_.position = waypoints[bSpline_parameter_.waypoint_num - 1];
    init_.velocity = velocities[0];
    finish_.velocity = velocities[1];

    // generalize b-spline parameter
    bSpline_parameter_.length = 0.0;

    //set path type
    path_type_ = PathType::B_SPLINE;

    // return path error and path parameter
    return  path_error;
}

double PathPlanner::calBSplineTrajectoryLength() {
    // calculate b-spline path's length
    double time = 0.0;
    double time_period = bSpline_parameter_.time_period;
    while (isPathOperating()) {
        calBSplineInterpole(time);
        time += time_period;
    }
    return bSpline_parameter_.length;
}

PathPlanner::PathError PathPlanner::genBSplineConstVelocityPath(const int p_degree, const int path_law, const vector<vector<double>> waypoints, const vector<vector<double>> velocities, const double length, const double target_velocity, const double target_acceleration) {
    // bspline degree
    bSpline_parameter_.p_degree = p_degree + 1;
    // path law
    bSpline_parameter_.path_law = path_law;
    // sampling time period: 0.001
    bSpline_parameter_.time_period = 0.001;
    // waypoint number
    bSpline_parameter_.waypoint_num = waypoints.size();

    // check b-spline path error
    PathError path_error;
    // error_type -> PathErrorType::MAX_WAYPOINT, PathErrorType::MIN_WAYPOINT
    if (bSpline_parameter_.waypoint_num < kBSplineWaypointNumMin) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MIN_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(kBSplineWaypointNumMin);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }
    else if (bSpline_parameter_.waypoint_num > WAYPOINT_LIMIT) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MAX_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(WAYPOINT_LIMIT);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_VEL
    if (isEqual(target_velocity, 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_VEL;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_ACC
    if (isEqual(target_acceleration, 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_ACC;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    vector<double> waypoints_distance(bSpline_parameter_.waypoint_num - 1);
    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
        for (int j = 0; j < 3; j++) {
            waypoints_distance[i] += pow(waypoints[i + 1][j] - waypoints[i][j], 2.0);
        }
        waypoints_distance[i] = sqrt(waypoints_distance[i]);
    }

    // error_type -> PathErrorType::SAME_WAYPOINTS
    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
        if (isEqual(waypoints_distance[i], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::SAME_WAYPOINTS;
            path_error_parameter.waypoint_index.push_back(i);
            path_error_parameter.waypoint_index.push_back(i + 1);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::SAME_WAYPOINTS) {
        genNoPath(current_.position);
        return path_error;
    }

    // generate bspline polynominal path
    genBSplinePolynominalPath(length, target_velocity, target_acceleration);

    // generalize b-spline parameter
    bSpline_parameter_.operation_time = bSpline_parameter_.path_time[0][3];
    if (bSpline_parameter_.p_degree % 2 == 0) {
        bSpline_parameter_.knot_number = 2 * (bSpline_parameter_.p_degree) + bSpline_parameter_.waypoint_num - 2;
    }
    else {
        bSpline_parameter_.knot_number = 2 * (bSpline_parameter_.p_degree) + bSpline_parameter_.waypoint_num - 1;
    }
    bSpline_parameter_.controlpoint_num = bSpline_parameter_.knot_number - bSpline_parameter_.p_degree;
    bSpline_parameter_.waypoints = waypoints;
    bSpline_parameter_.tangent_vector = velocities;

    // step 1: generate interpole parameter, knot vector
    vector<double> interpol_parameter(bSpline_parameter_.waypoint_num);
    vector<double> knot_vector(bSpline_parameter_.knot_number);
    genBSplineInterpoleParameter(interpol_parameter, knot_vector);

    // if init velocity and finish velocity are zero, set tangent vector according to waypoints distances
    if (isEqual(pow(velocities[0][0], 2.0) + pow(velocities[0][1], 2.0) + pow(velocities[0][2], 2.0), 0.0)) {
        for (int i = 0; i < 3; i++) {
            bSpline_parameter_.tangent_vector[0][i] = divideWithFloatingError(waypoints[1][i] - waypoints[0][i], interpol_parameter[1] - interpol_parameter[0]);
        }
    }
    if (isEqual(pow(velocities[1][0], 2.0) + pow(velocities[1][1], 2.0) + pow(velocities[1][2], 2.0), 0.0)) {
        for (int i = 0; i < 3; i++) {
            bSpline_parameter_.tangent_vector[1][i] = divideWithFloatingError(waypoints[bSpline_parameter_.waypoint_num - 1][i] - waypoints[bSpline_parameter_.waypoint_num - 2][i], interpol_parameter[interpol_parameter.size() - 1] - interpol_parameter[interpol_parameter.size() - 2]);
        }
    }

    // step 2: generate basis function
    vector<vector<vector<double>>> basis_total;
    vector<vector<double>> basis;
    genBSplineBasisFunction(interpol_parameter, knot_vector, basis, basis_total);

    // step 3: generate basis derivative function
    vector<vector<double>> basis_first_derivative;
    vector<vector<double>> basis_second_derivative;
    vector<vector<double>> basis_thrid_derivative;
    vector<vector<double>> basis_fourth_derivative;
    genBSplineBasisFunctionDerivative(knot_vector, basis_total, basis_first_derivative, basis_second_derivative, basis_thrid_derivative, basis_fourth_derivative);

    // step 4: generate control point
    genBSplineControlPoint(basis, basis_first_derivative, basis_second_derivative, basis_thrid_derivative);

    // set init and final state
    init_.time = current_.time;
    init_.position = waypoints[0];
    finish_.position = waypoints[bSpline_parameter_.waypoint_num - 1];
    init_.velocity = velocities[0];
    finish_.velocity = velocities[1];

    // generalize b-spline parameter
    bSpline_parameter_.time = 0.0;

    //set path type
    path_type_ = PathType::B_SPLINE;

    return path_error;
}

//vector<double> PathPlanner::calculateBlendingWaypointsVelocity(const vector<vector<double>> target_waypoints, const vector<double> target_velocities, const vector<double> target_accelerations, const vector<double> radiuses, const bool radius_option) {
//    init_.position = current_.position;
//    init_.velocity = current_.velocity;
//
//    // set waypoints, paths, arc number
//    int waypoints_num = target_waypoints.size() + 1;
//    int paths_num = waypoints_num - 1;
//    int arc_num = paths_num - 1;
//
//    // check blending path error
//    PathError path_error;
//    // error_type -> PathErrorType::INIT_VEL
//    for (unsigned int i = 0; i < dof_; i++) {
//        if (!isEqual(init_.velocity[i], 0.0, 3)) {
//            path_error.type = PathErrorType::INIT_VEL;
//        }
//    }
//    if (path_error.type == PathErrorType::INIT_VEL) {
//        genStopPath(0.2);
//    }
//
//    // error_type -> PathErrorType::ZERO_TARGET_VEL
//    for (int i = 0; i < waypoints_num - 1; i++) {
//        if (isEqual(target_velocities[i], 0.0)) {
//            PathErrorParameter path_error_parameter;
//            path_error.type = PathErrorType::ZERO_TARGET_VEL;
//            path_error_parameter.waypoint_index.push_back(i);
//            path_error.path_error_parameter.push_back(path_error_parameter);
//        }
//    }
//    if (path_error.type == PathErrorType::ZERO_TARGET_VEL) {
//        genNoPath(current_.position);
//    }
//
//    // error_type -> PathErrorType::ZERO_TARGET_ACC
//    for (int i = 0; i < waypoints_num - 1; i++) {
//        if (isEqual(target_velocities[i], 0.0)) {
//            PathErrorParameter path_error_parameter;
//            path_error.type = PathErrorType::ZERO_TARGET_ACC;
//            path_error_parameter.waypoint_index.push_back(i);
//            path_error.path_error_parameter.push_back(path_error_parameter);
//        }
//    }
//    if (path_error.type == PathErrorType::ZERO_TARGET_ACC) {
//        genNoPath(current_.position);
//    }
//
//    // error_type -> PathErrorType::MAX_WAYPOINT, PathErrorType::MIN_WAYPOINT
//    if (waypoints_num - 1 < 3) {
//        PathErrorParameter path_error_parameter;
//        path_error.type = PathErrorType::MIN_WAYPOINT;
//        path_error_parameter.waypoint_index.push_back(3);
//        path_error.path_error_parameter.push_back(path_error_parameter);
//        genNoPath(current_.position);
//    } else if (waypoints_num - 1 > WAYPOINT_LIMIT) {
//        PathErrorParameter path_error_parameter;
//        path_error.type = PathErrorType::MAX_WAYPOINT;
//        path_error_parameter.waypoint_index.push_back(WAYPOINT_LIMIT);
//        path_error.path_error_parameter.push_back(path_error_parameter);
//        genNoPath(current_.position);
//    }
//
//    // set init radius and final radius zero
//    vector<double> radius(arc_num);
//    vector<double> radius_percent;
//    if (radius_option) {
//        radius_percent = radiuses;
//        radius_percent[0] = 0.0;
//        radius_percent[radiuses.size() - 1] = 0.0;
//    } else {
//        radius = radiuses;
//        radius[0] = 0.0;
//        radius[radiuses.size() - 1] = 0.0;
//    }
//
//    // make waypoints include init position
//    vector<vector<double>> waypoints;
//    waypoints.push_back(init_.position);
//    for (int i = 0; i < waypoints_num - 1; i++) {
//        waypoints.push_back(target_waypoints[i]);
//    }
//
//    // divide cartesian spae into position space and orientation space
//    vector<vector<double>> xyz_distance(paths_num);
//    vector<vector<double>> orientation_distance(paths_num);
//    vector<vector<double>> xyz_unit_distance(paths_num);
//    vector<double> xyz_distance_magnitude(paths_num);
//    for (int i = 0; i < paths_num; i++) {
//        xyz_distance[i].resize(3);
//        orientation_distance[i].resize(3);
//        for (int j = 0; j < 3; j++) {
//            xyz_distance[i][j] = waypoints[i + 1][j] - waypoints[i][j];
//            orientation_distance[i][j] = waypoints[i + 1][j + 3] - waypoints[i][j + 3];
//            // check orientation divergence error in cartesian space
//            if (orientation_distance[i][j] > 180.0) {
//                orientation_distance[i][j] = orientation_distance[i][j] - 360.0;
//            } else if (orientation_distance[i][j] < -180.0) {
//                orientation_distance[i][j] = orientation_distance[i][j] + 360.0;
//            }
//        }
//        xyz_unit_distance[i] = calVectorNomalize(xyz_distance[i]);
//        xyz_distance_magnitude[i] = calVectorMagnitude(xyz_distance[i]);
//    }
//
//    // calculate circular path
//    vector<double> inner_product_distance(arc_num);
//    vector<double> theta_radian(arc_num);
//    vector<double> theta(arc_num);
//    vector<double> vector_d(arc_num);
//    vector<double> xyz_distance_min(arc_num);
//    for (int i = 0; i < arc_num; i++) {
//        // calculate theta
//        for (int j = 0; j < 3; j++) {
//            inner_product_distance[i] -= xyz_distance[i + 1][j] * xyz_distance[i][j];
//        }
//        theta_radian[i] = acos(divideWithFloatingError(inner_product_distance[i], xyz_distance_magnitude[i + 1] * xyz_distance_magnitude[i]));
//        theta[i] = theta_radian[i] * kRad2Deg;
//
//        // calculate minimum distance adjacent to waypoints to calculate radius when radius option is true
//        if (xyz_distance_magnitude[i] > xyz_distance_magnitude[i + 1]) {
//            xyz_distance_min[i] = xyz_distance_magnitude[i + 1];
//        } else {
//            xyz_distance_min[i] = xyz_distance_magnitude[i];
//        }
//
//        // radius_option
//        if (!radius_option) {
//            // radius_option = false -> radius
//            vector_d[i] = divideWithFloatingError(radius[i], tan(theta_radian[i] / 2.0));
//            radius[i] = radius[i];
//        } else {
//            // radius_option = true -> radius_percent
//            vector_d[i] = 0.5 * xyz_distance_min[i] * radius_percent[i] * 0.01;
//            radius[i] = vector_d[i] * tan(theta_radian[i] / 2.0);
//        }
//    }
//
//    // error_type -> PathErrorType::SAME_WAYPOINTS
//    for (int i = 1; i < paths_num; i++) {
//        if (isEqual(xyz_distance_magnitude[i], 0.0)) {
//            PathErrorParameter path_error_parameter;
//            path_error.type = PathErrorType::SAME_WAYPOINTS;
//            path_error_parameter.waypoint_index.push_back(i - 1);
//            path_error_parameter.waypoint_index.push_back(i);
//            path_error.path_error_parameter.push_back(path_error_parameter);
//        }
//    }
//    if (path_error.type == PathErrorType::SAME_WAYPOINTS) {
//        genNoPath(current_.position);
//    }
//
//    // error_type -> PathErrorType::COLINEAR
//    for (int i = 0; i < arc_num; i++) {
//        if (!(radius[i] == 0.0)) {
//            if (isEqual(fabs(theta_radian[i]), M_PI)) {
//                PathErrorParameter path_error_parameter;
//                path_error.type = PathErrorType::COLINEAR;
//                path_error_parameter.waypoint_index.push_back(i - 1);
//                path_error_parameter.waypoint_index.push_back(i);
//                path_error_parameter.waypoint_index.push_back(i + 1);
//                path_error.path_error_parameter.push_back(path_error_parameter);
//            }
//        }
//    }
//    if (path_error.type == PathErrorType::COLINEAR) {
//        genNoPath(current_.position);
//    }
//
//    // error_type -> PathErrorType::LARGE_RADIUS
//    for (int i = 0; i < arc_num; i++) {
//        if (!(radius[i] == 0.0)) {
//            if (vector_d[i] > xyz_distance_min[i] / 2.0) {
//                PathErrorParameter path_error_parameter;
//                path_error.type = PathErrorType::LARGE_RADIUS;
//                path_error_parameter.max_radius = xyz_distance_min[i] / 2.0 - 0.001;
//                path_error_parameter.waypoint_index.push_back(i);
//                path_error.path_error_parameter.push_back(path_error_parameter);
//            }
//        }
//    }
//    if (path_error.type == PathErrorType::LARGE_RADIUS) {
//        genNoPath(current_.position);
//    }
//
//    // calculate circular path
//    vector<vector<double>> vector_dc(arc_num);
//    vector<vector<double>> circle_start_position(arc_num);
//    vector<vector<double>> circle_end_position(arc_num);
//    vector<vector<double>> circle_center_position(arc_num);
//    vector<vector<double>> unit_vector_b0(arc_num);
//    vector<vector<double>> unit_vector_b1(arc_num);
//    for (int i = 0; i < arc_num; i++) {
//        vector_dc[i].resize(3);
//        circle_start_position[i].resize(3);
//        circle_end_position[i].resize(3);
//        circle_center_position[i].resize(3);
//
//        vector<double> vector_b1(3);
//        for (int j = 0; j < 3; j++) {
//            vector_dc[i][j] = divideWithFloatingError(vector_d[i], 2 * pow(cos(theta_radian[i] / 2.0), 2.0)) * (divideWithFloatingError(-xyz_distance[i][j], xyz_distance_magnitude[i]) + divideWithFloatingError(xyz_distance[i + 1][j], xyz_distance_magnitude[i + 1]));
//        }
//        vector<double> vector_dc_normalize = calVectorNomalize(vector_dc[i]);
//
//        for (int j = 0; j < 3; j++) {
//            circle_start_position[i][j] = -vector_d[i] * divideWithFloatingError(xyz_distance[i][j], xyz_distance_magnitude[i]) + waypoints[i + 1][j];
//            circle_end_position[i][j] = vector_d[i] * divideWithFloatingError(xyz_distance[i + 1][j], xyz_distance_magnitude[i + 1]) + waypoints[i + 1][j];
//            circle_center_position[i][j] = vector_dc_normalize[j] * sqrt(pow(vector_d[i], 2.0) + pow(radius[i], 2.0)) + waypoints[i + 1][j];
//            vector_b1[j] = circle_start_position[i][j] - circle_center_position[i][j];
//        }
//        unit_vector_b1[i] = calVectorNomalize(vector_b1);
//        unit_vector_b0[i] = xyz_unit_distance[i];
//    }
//
//    vector<double> linear_distance(paths_num);
//    for (int i = 0; i < 3; i++) {
//        linear_distance[0] += pow(circle_start_position[0][i] - waypoints[0][i], 2.0);
//        linear_distance[paths_num - 1] += pow(waypoints[waypoints_num - 1][i] - circle_end_position[arc_num - 1][i], 2.0);
//    }
//    for (int i = 1; i < paths_num - 1; i++) {
//        for (int j = 0; j < 3; j++) {
//            linear_distance[i] += pow(circle_start_position[i][j] - circle_end_position[i - 1][j], 2.0);
//        }
//    }
//    for (int i = 0; i < paths_num; i++) {
//        linear_distance[i] = sqrt(linear_distance[i]);
//    }
//
//    vector<double> circle_distance(arc_num);
//    for (int i = 0; i < arc_num; i++) {
//        circle_distance[i] = radius[i] * (M_PI - theta_radian[i]);
//    }
//
//    vector<double> linear_legnth_segment(paths_num);
//    linear_legnth_segment[0] = linear_distance[0] + circle_distance[0] / 2.0;
//    for (int i = 0; i < paths_num - 2; i++) {
//        linear_legnth_segment[i + 1] = circle_distance[i] / 2.0 + linear_distance[i + 1] + circle_distance[i + 1] / 2.0;
//    }
//    linear_legnth_segment[paths_num - 1] = circle_distance[arc_num - 1] / 2.0 + linear_distance[paths_num - 1];
//
//    vector<double> linear_legnth(waypoints_num);
//    for (int i = 0; i < paths_num; i++) {
//        linear_legnth[i + 1] = linear_legnth[i] + linear_legnth_segment[i];
//    }
//
//    vector<vector<double>> linear_waypoints(waypoints_num);
//    for (int i = 0; i < waypoints_num; i++) {
//        linear_waypoints[i].resize(4);
//        linear_waypoints[i][0] = linear_legnth[i];
//        for (int j = 0; j < 3; j++) {
//            linear_waypoints[i][j + 1] = waypoints[i][j + 3];
//        }
//    }
//
//    vector<vector<double>> distances(paths_num);
//    vector<vector<double>> distances_linear(paths_num);
//    vector<vector<double>> distances_orientation(paths_num);
//    for (int i = 0; i < paths_num; i++) {
//        distances[i].resize(4);
//        distances_linear[i].resize(1);
//        distances_orientation[i].resize(3);
//
//        distances_linear[i][0] = linear_waypoints[i + 1][0] - linear_waypoints[i][0];
//        distances[i][0] = distances_linear[i][0];
//        for (int j = 0; j < 3; j++) {
//            distances_orientation[i][j] = linear_waypoints[i + 1][j + 1] - linear_waypoints[i][j + 1];
//            if (distances_orientation[i][j] > 180.0) {
//                distances_orientation[i][j] = distances_orientation[i][j] - 360.0;
//            } else if (distances_orientation[i][j] < -180.0) {
//                distances_orientation[i][j] = distances_orientation[i][j] + 360.0;
//            }
//            distances[i][j + 1] = distances_orientation[i][j];
//        }
//    }
//
//    vector<double> zero_velocity_xyz(1);
//    vector<double> zero_velocity_rpy(3);
//    double max_rpy_velocity = 120.0;
//    double max_rpy_acceleration = 480.0;
//    vector<vector<double>> uniform_velocity_xyz(paths_num);
//    vector<vector<double>> uniform_velocity_rpy(paths_num);
//    vector<vector<double>> path_time_segment_xyz(paths_num);
//    vector<vector<double>> path_time_segment_rpy(paths_num);
//    vector<vector<double>> path_minimun_velocity(paths_num);
//    vector<vector<double>> waypoints_velocity_temp(waypoints_num);
//
//    waypoints_velocity_temp.resize(waypoints_num);
//    for (int i = 0; i < waypoints_num; i++) {
//        waypoints_velocity_temp[i].resize(1);
//    }
//
//    for (int number = 0; number < 1000; number++) {
//        for (int i = 0; i < paths_num; i++) {
//            vector<double> target_velocity_xyz(1);
//            vector<double> target_acceleration_xyz(1);
//            vector<double> max_velocity_xyz(1);
//            double path_time_xyz = 0.0;
//
//            vector<double> target_velocity_rpy(3);
//            vector<double> target_acceleration_rpy(3);
//            vector<double> max_velocity_rpy(3);
//            double path_time_rpy = 0.0;
//
//            target_velocity_xyz[0] = target_velocities[i];
//            if (degree_ == 5) {
//                target_acceleration_xyz[0] = target_accelerations[i] * 2.0 / 3.0;
//            } else if (degree_ == 7) {
//                target_acceleration_xyz[0] = target_accelerations[i] * 8.0 / 15.0;
//            }
//
//            for (int j = 0; j < 3; j++) {
//                target_velocity_rpy[j] = max_rpy_velocity;
//                if (degree_ == 5) {
//                    target_acceleration_rpy[j] = max_rpy_acceleration * 2.0 / 3.0;
//                } else if (degree_ == 7) {
//                    target_acceleration_rpy[j] = max_rpy_acceleration * 8.0 / 15.0;
//                }
//            }
//
//            max_velocity_rpy = calMaxVelocity(zero_velocity_rpy, zero_velocity_rpy, distances_orientation[i], target_acceleration_rpy);
//            uniform_velocity_rpy[i] = compareVelocity(max_velocity_rpy, target_velocity_rpy);
//            path_time_rpy = compareTimeSum(zero_velocity_rpy, zero_velocity_rpy, uniform_velocity_rpy[i], distances_orientation[i], target_acceleration_rpy);
//
//            max_velocity_xyz = calMaxVelocity(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], distances_linear[i], target_acceleration_xyz);
//            uniform_velocity_xyz[i] = compareVelocity(max_velocity_xyz, target_velocity_xyz);
//            path_time_xyz = compareTimeSum(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], uniform_velocity_xyz[i], distances_linear[i], target_acceleration_xyz);
//
//            if (path_time_rpy > path_time_xyz) {
//                path_time_xyz = path_time_rpy;
//                uniform_velocity_xyz[i] = calUniformVelocity(path_time_xyz, waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], distances_linear[i], target_acceleration_xyz);
//            }
//
//            path_minimun_velocity[i].resize(4);
//            path_minimun_velocity[i][0] = uniform_velocity_xyz[i][0];
//        }
//
//        vector<vector<double>> waypoints_previous_velocity(waypoints_num);
//        waypoints_previous_velocity = waypoints_velocity_temp;
//        for (int i = 0; i < waypoints_num - 2; i++) {
//            if (path_minimun_velocity[i][0] > path_minimun_velocity[i + 1][0]) {
//                waypoints_velocity_temp[i + 1][0] = path_minimun_velocity[i + 1][0];
//            } else {
//                waypoints_velocity_temp[i + 1][0] = path_minimun_velocity[i][0];
//            }
//        }
//
//        for (int i = 0; i < arc_num; i++) {
//            if (isEqual(radius[i], 0.0)) {
//                waypoints_velocity_temp[i + 1][0] = 0.0;
//            }
//        }
//
//        bool check = true;
//        for (int i = 0; i < waypoints_num; i++) {
//            if(!isEqual(waypoints_velocity_temp[i][0], waypoints_previous_velocity[i][0])) {
//                check = false;
//            }
//        }
//        if (check) {
//            break;
//        }
//    }
//
//    vector<double> waypoints_velocity(waypoints_num);
//    for (int i = 0; i < waypoints_num; i++) {
//        waypoints_velocity[i] = waypoints_velocity_temp[i][0];
//    }
//
//    return waypoints_velocity;
//}

PathPlanner::PathError PathPlanner::checkBlendingParameter(const int number, const vector<vector<double>> target_waypoints, const vector<double> radiuses) {
    init_.position = current_.position;
    int waypoints_num = target_waypoints.size() + 1;

    vector<vector<double>> waypoints;
    vector<double> radius(1);
    waypoints.push_back(init_.position);
    for (int i = 0; i < waypoints_num - 1; i++) {
        waypoints.push_back(target_waypoints[i]);
        radius.push_back(radiuses[i]);
    }
    radius[1] = 0.0;

    vector<double> distance(waypoints_num - 1);
    for (int i = 0; i < waypoints_num - 1; i++) {
        for (int j = 0; j < 3; j++) {
            distance[i] += pow(waypoints[i + 1][j] - waypoints[i][j], 2.0);
        }
        distance[i] = sqrt(distance[i]);
    }
    vector<double> distance_min(waypoints_num);
    for (int i = 1; i < waypoints_num - 1; i++) {
        if (distance[i - 1] > distance[i]) {
            distance_min[i] = distance[i];
        }
        else {
            distance_min[i] = distance[i - 1];
        }
    }

    PathError path_error;
    if ((waypoints_num - 1) == (number + 1)) {
        if (distance[waypoints_num - 2] < radius[waypoints_num - 2]) {
            path_error.type = PathErrorType::CLOSE_WAYPOINTS;
            PathErrorParameter path_error_parameter;
            path_error_parameter.waypoint_index.push_back(waypoints_num - 1);
            path_error_parameter.waypoint_index.push_back(waypoints_num);
        }
        else {
            double remain_distance = distance[waypoints_num - 2] - radius[waypoints_num - 2];
            if (remain_distance < radius[waypoints_num - 1]) {
                path_error.type = PathErrorType::LARGE_RADIUS;
                PathErrorParameter path_error_parameter;
                path_error_parameter.max_radius = remain_distance - 0.001;
                path_error.path_error_parameter.push_back(path_error_parameter);
            }
        }

        if (path_error.type == PathErrorType::CLOSE_WAYPOINTS) {
            genNoPath(current_.position);
            return path_error;
        }

        if (path_error.type == PathErrorType::LARGE_RADIUS) {
            genNoPath(current_.position);
            return path_error;
        }
    }
    else {
        if (distance[number] < radius[number]) {
            path_error.type = PathErrorType::CLOSE_WAYPOINTS;
            PathErrorParameter path_error_parameter;
            path_error_parameter.waypoint_index.push_back(number - 1);
            path_error_parameter.waypoint_index.push_back(number);
        }
        else if (distance[number + 1] < radius[number + 2]) {
            path_error.type = PathErrorType::CLOSE_WAYPOINTS;
            PathErrorParameter path_error_parameter;
            path_error_parameter.waypoint_index.push_back(number);
            path_error_parameter.waypoint_index.push_back(number + 1);
        }
        else {
            double remain_distance = 0.0;
            double remain_distance_1 = distance[number] - radius[number];
            double remain_distance_2 = distance[number + 1] - radius[number + 2];
            if (remain_distance_1 < remain_distance_2) {
                remain_distance = remain_distance_1;
            }
            else {
                remain_distance = remain_distance_2;
            }

            if (remain_distance < radius[number + 1]) {
                path_error.type = PathErrorType::LARGE_RADIUS;
                PathErrorParameter path_error_parameter;
                path_error_parameter.max_radius = remain_distance - 0.001;
                path_error.path_error_parameter.push_back(path_error_parameter);
            }
        }
    }

    return path_error;
}

PathPlanner::PathError PathPlanner::genBlendingPath(const vector<vector<double>> target_waypoints,
                                                         const vector<double> target_velocities,
                                                         const vector<double> target_accelerations,
                                                         const vector<double> radiuses, const bool radius_option,
                                                         vector<double> waypoints_velocity) {
    // set init state
    init_.position = current_.position;
    init_.velocity = current_.velocity;

    // set waypoints, paths, arc number
    int waypoints_num = target_waypoints.size() + 1;
    int paths_num = waypoints_num - 1;
    int arc_num = paths_num - 1;

    // check blending path error
    PathError path_error;
    // error_type -> PathErrorType::INIT_VEL
    for (unsigned int i = 0; i < dof_; i++) {
        if (!isEqual(init_.velocity[i], 0.0, 3)) {
            path_error.type = PathErrorType::INIT_VEL;
        }
    }
    if (path_error.type == PathErrorType::INIT_VEL) {
        genStopPath(0.2);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_VEL
    for (int i = 0; i < waypoints_num - 1; i++) {
        if (isEqual(target_velocities[i], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::ZERO_TARGET_VEL;
            path_error_parameter.waypoint_index.push_back(i);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::ZERO_TARGET_VEL) {
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::ZERO_TARGET_ACC
    for (int i = 0; i < waypoints_num - 1; i++) {
        if (isEqual(target_velocities[i], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::ZERO_TARGET_ACC;
            path_error_parameter.waypoint_index.push_back(i);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::ZERO_TARGET_ACC) {
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::MAX_WAYPOINT, PathErrorType::MIN_WAYPOINT
    if (waypoints_num - 1 < 3) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MIN_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(3);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }
    else if (waypoints_num - 1 > WAYPOINT_LIMIT) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::MAX_WAYPOINT;
        path_error_parameter.waypoint_index.push_back(WAYPOINT_LIMIT);
        path_error.path_error_parameter.push_back(path_error_parameter);
        genNoPath(current_.position);
        return path_error;
    }

    // set init radius and final radius zero
    vector<double> radius(arc_num);
    vector<double> radius_percent;
    if (radius_option) {
        radius_percent = radiuses;
        radius_percent[0] = 0.0;
        radius_percent[radiuses.size() - 1] = 0.0;
    }
    else {
        radius = radiuses;
        radius[0] = 0.0;
        radius[radiuses.size() - 1] = 0.0;
    }

    // make waypoints include init position
    vector<vector<double>> waypoints;
    waypoints.push_back(init_.position);
    for (int i = 0; i < waypoints_num - 1; i++) {
        waypoints.push_back(target_waypoints[i]);
    }

    // divide cartesian spae into position space and orientation space
    vector<vector<double>> xyz_distance(paths_num);
    vector<vector<double>> orientation_distance(paths_num);
    vector<vector<double>> xyz_unit_distance(paths_num);
    vector<double> xyz_distance_magnitude(paths_num);
    for (int i = 0; i < paths_num; i++) {
        xyz_distance[i].resize(3);

        for (int j = 0; j < 3; j++) {
            xyz_distance[i][j] = waypoints[i + 1][j] - waypoints[i][j];
        }

        // for 7-dof case
        orientation_distance[i].resize(dof_ - 3);

        for (int j = 0; j < dof_ - 3; j++) {
            orientation_distance[i][j] = waypoints[i + 1][j + 3] - waypoints[i][j + 3];
            // check orientation divergence error in cartesian space
            if (orientation_distance[i][j] > 180.0) {
                orientation_distance[i][j] = orientation_distance[i][j] - 360.0;
            }
            else if (orientation_distance[i][j] < -180.0) {
                orientation_distance[i][j] = orientation_distance[i][j] + 360.0;
            }
        }

        xyz_unit_distance[i] = calVectorNomalize(xyz_distance[i]);
        xyz_distance_magnitude[i] = calVectorMagnitude(xyz_distance[i]);
    }

    // calculate circular path
    vector<double> inner_product_distance(arc_num);
    vector<double> theta_radian(arc_num);
    vector<double> theta(arc_num);
    vector<double> vector_d(arc_num);
    vector<double> xyz_distance_min(arc_num);
    for (int i = 0; i < arc_num; i++) {
        // calculate theta
        for (int j = 0; j < 3; j++) {
            inner_product_distance[i] -= xyz_distance[i + 1][j] * xyz_distance[i][j];
        }
        theta_radian[i] = acos(divideWithFloatingError(inner_product_distance[i], xyz_distance_magnitude[i + 1] * xyz_distance_magnitude[i]));
        theta[i] = theta_radian[i] * kRad2Deg;

        // calculate minimum distance adjacent to waypoints to calculate radius when radius option is true
        if (xyz_distance_magnitude[i] > xyz_distance_magnitude[i + 1]) {
            xyz_distance_min[i] = xyz_distance_magnitude[i + 1];
        }
        else {
            xyz_distance_min[i] = xyz_distance_magnitude[i];
        }

        // radius_option
        if (!radius_option) {
            // radius_option = false -> radius
            vector_d[i] = divideWithFloatingError(radius[i], tan(theta_radian[i] / 2.0));
            radius[i] = radius[i];
        }
        else {
            // radius_option = true -> radius_percent
            vector_d[i] = 0.5 * xyz_distance_min[i] * radius_percent[i] * 0.01;
            radius[i] = vector_d[i] * tan(theta_radian[i] / 2.0);
        }
    }

    // error_type -> PathErrorType::SAME_WAYPOINTS
    for (int i = 1; i < paths_num; i++) {
        if (isEqual(xyz_distance_magnitude[i], 0.0)) {
            PathErrorParameter path_error_parameter;
            path_error.type = PathErrorType::SAME_WAYPOINTS;
            path_error_parameter.waypoint_index.push_back(i - 1);
            path_error_parameter.waypoint_index.push_back(i);
            path_error.path_error_parameter.push_back(path_error_parameter);
        }
    }
    if (path_error.type == PathErrorType::SAME_WAYPOINTS) {
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::COLINEAR
    for (int i = 0; i < arc_num; i++) {
        if (!(radius[i] == 0.0)) {
            if (isEqual(fabs(theta_radian[i]), M_PI)) {
                PathErrorParameter path_error_parameter;
                path_error.type = PathErrorType::COLINEAR;
                path_error_parameter.waypoint_index.push_back(i - 1);
                path_error_parameter.waypoint_index.push_back(i);
                path_error_parameter.waypoint_index.push_back(i + 1);
                path_error.path_error_parameter.push_back(path_error_parameter);
            }
        }
    }
    if (path_error.type == PathErrorType::COLINEAR) {
        genNoPath(current_.position);
        return path_error;
    }

    // error_type -> PathErrorType::LARGE_RADIUS
    for (int i = 0; i < arc_num; i++) {
        if (!(radius[i] == 0.0)) {
            PathError error = checkBlendingParameter(i, target_waypoints, radiuses);
            if (error.type == PathErrorType::LARGE_RADIUS) {
                PathErrorParameter path_error_parameter;
                path_error.type = PathErrorType::LARGE_RADIUS;
                path_error_parameter.max_radius = error.path_error_parameter[0].max_radius;
                path_error_parameter.waypoint_index.push_back(i);
                path_error.path_error_parameter.push_back(path_error_parameter);
            }
        }
    }
    if (path_error.type == PathErrorType::LARGE_RADIUS) {
        genNoPath(current_.position);
        return path_error;
    }

    // calculate circular path
    vector<vector<double>> vector_dc(arc_num);
    vector<vector<double>> circle_start_position(arc_num);
    vector<vector<double>> circle_end_position(arc_num);
    vector<vector<double>> circle_center_position(arc_num);
    vector<vector<double>> unit_vector_b0(arc_num);
    vector<vector<double>> unit_vector_b1(arc_num);
    for (int i = 0; i < arc_num; i++) {
        vector_dc[i].resize(3);
        circle_start_position[i].resize(3);
        circle_end_position[i].resize(3);
        circle_center_position[i].resize(3);

        vector<double> vector_b1(3);
        for (int j = 0; j < 3; j++) {
            vector_dc[i][j] = divideWithFloatingError(vector_d[i], 2 * pow(cos(theta_radian[i] / 2.0), 2.0)) * (divideWithFloatingError(-xyz_distance[i][j], xyz_distance_magnitude[i]) + divideWithFloatingError(xyz_distance[i + 1][j], xyz_distance_magnitude[i + 1]));
        }
        vector<double> vector_dc_normalize = calVectorNomalize(vector_dc[i]);

        for (int j = 0; j < 3; j++) {
            circle_start_position[i][j] = -vector_d[i] * divideWithFloatingError(xyz_distance[i][j], xyz_distance_magnitude[i]) + waypoints[i + 1][j];
            circle_end_position[i][j] = vector_d[i] * divideWithFloatingError(xyz_distance[i + 1][j], xyz_distance_magnitude[i + 1]) + waypoints[i + 1][j];
            circle_center_position[i][j] = vector_dc_normalize[j] * sqrt(pow(vector_d[i], 2.0) + pow(radius[i], 2.0)) + waypoints[i + 1][j];
            vector_b1[j] = circle_start_position[i][j] - circle_center_position[i][j];
        }
        unit_vector_b1[i] = calVectorNomalize(vector_b1);
        unit_vector_b0[i] = xyz_unit_distance[i];
    }

    vector<double> linear_distance(paths_num);
    for (int i = 0; i < 3; i++) {
        linear_distance[0] += pow(circle_start_position[0][i] - waypoints[0][i], 2.0);
        linear_distance[paths_num - 1] += pow(waypoints[waypoints_num - 1][i] - circle_end_position[arc_num - 1][i], 2.0);
    }
    for (int i = 1; i < paths_num - 1; i++) {
        for (int j = 0; j < 3; j++) {
            linear_distance[i] += pow(circle_start_position[i][j] - circle_end_position[i - 1][j], 2.0);
        }
    }
    for (int i = 0; i < paths_num; i++) {
        linear_distance[i] = sqrt(linear_distance[i]);
    }

    vector<double> circle_distance(arc_num);
    for (int i = 0; i < arc_num; i++) {
        circle_distance[i] = radius[i] * (M_PI - theta_radian[i]);
    }

    vector<double> linear_legnth_segment(paths_num);
    linear_legnth_segment[0] = linear_distance[0] + circle_distance[0] / 2.0;
    for (int i = 0; i < paths_num - 2; i++) {
        linear_legnth_segment[i + 1] = circle_distance[i] / 2.0 + linear_distance[i + 1] + circle_distance[i + 1] / 2.0;
    }
    linear_legnth_segment[paths_num - 1] = circle_distance[arc_num - 1] / 2.0 + linear_distance[paths_num - 1];

    vector<double> linear_legnth(waypoints_num);
    for (int i = 0; i < paths_num; i++) {
        linear_legnth[i + 1] = linear_legnth[i] + linear_legnth_segment[i];
    }

    vector<vector<double>> linear_waypoints(waypoints_num);
    for (int i = 0; i < waypoints_num; i++) {
        // for 7-dof case
        linear_waypoints[i].resize(dof_ - 2);
        linear_waypoints[i][0] = linear_legnth[i];
        for (int j = 0; j < dof_ - 3; j++) {
            linear_waypoints[i][j + 1] = waypoints[i][j + 3];
        }
    }

    vector<double> init_position(init_.position.begin(), init_.position.begin() + 3);
    vector<vector<double>> geometric_waypoints;
    geometric_waypoints.push_back(init_position);
    for (int i = 0; i < arc_num; i++) {
        geometric_waypoints.push_back(circle_end_position[i]);
    }

    vector<double> geometric_length_segment(paths_num + arc_num);
    for (int i = 0; i < arc_num; i++) {
        geometric_length_segment[2 * i] = linear_distance[i];
        geometric_length_segment[2 * i + 1] = circle_distance[i];
    }
    geometric_length_segment[2 * arc_num] = linear_distance[paths_num - 1];

    vector<double> geometric_length(paths_num + arc_num + 1);
    for (int i = 0; i < paths_num + arc_num; i++) {
        geometric_length[i + 1] = geometric_length[i] + geometric_length_segment[i];
    }

    blending_parameter_.geometric_segment.clear();
    BlendingParameter::Geometric geometirc_linear_start;
    geometirc_linear_start.length = geometric_length[0];
    geometirc_linear_start.type = true;
    blending_parameter_.geometric_segment.push_back(geometirc_linear_start);
    for (int i = 0; i < arc_num; i++) {
        BlendingParameter::Geometric geometirc_linear;
        geometirc_linear.length = geometric_length[2 * i + 1];
        geometirc_linear.type = true;
        blending_parameter_.geometric_segment.push_back(geometirc_linear);

        BlendingParameter::Geometric geometirc_circular;
        geometirc_circular.length = geometric_length[2 * i + 2];
        geometirc_circular.type = false;
        blending_parameter_.geometric_segment.push_back(geometirc_circular);
    }
    BlendingParameter::Geometric geometirc_linear_finish;
    geometirc_linear_finish.length = geometric_length[paths_num + arc_num];
    geometirc_linear_finish.type = true;
    blending_parameter_.geometric_segment.push_back(geometirc_linear_finish);

    vector<vector<double>> distances(paths_num);
    vector<vector<double>> distances_linear(paths_num);
    vector<vector<double>> distances_orientation(paths_num);
    for (int i = 0; i < paths_num; i++) {
        // for 7-dof case
        distances[i].resize(dof_ - 2);
        distances_linear[i].resize(1);

        distances_orientation[i].resize(dof_ - 3);

        distances_linear[i][0] = linear_waypoints[i + 1][0] - linear_waypoints[i][0];
        distances[i][0] = distances_linear[i][0];
        for (int j = 0; j < dof_ - 3; j++) {
            distances_orientation[i][j] = linear_waypoints[i + 1][j + 1] - linear_waypoints[i][j + 1];
            if (distances_orientation[i][j] > 180.0) {
                distances_orientation[i][j] = distances_orientation[i][j] - 360.0;
            }
            else if (distances_orientation[i][j] < -180.0) {
                distances_orientation[i][j] = distances_orientation[i][j] + 360.0;
            }
            distances[i][j + 1] = distances_orientation[i][j];
        }
    }

    vector<double> zero_velocity_xyz(1);
    // vector<double> zero_velocity_rpy(3);
    double max_rpy_velocity = 75.0;
    double max_rpy_acceleration = 120.0;
    double max_redundant_velocity = 45.0;
    double max_redundant_acceleration = 60.0;
    // vector<vector<double>> uniform_velocity_xyz(paths_num);
    // vector<vector<double>> uniform_velocity_rpy(paths_num);
    // vector<vector<double>> path_time_segment_xyz(paths_num);
    // vector<vector<double>> path_time_segment_rpy(paths_num);
    // vector<vector<double>> path_minimun_velocity(paths_num);

    vector<vector<double>> waypoints_velocity_temp(waypoints_num);
    for (int i = 0; i < waypoints_num - 1; i++) {
        // for 7-dof case
        waypoints_velocity_temp[i].resize(dof_ - 2);
        waypoints_velocity_temp[i][0] = waypoints_velocity[i];
    }

    waypoints_velocity_temp[waypoints_num - 1].resize(dof_ - 2);

    vector<vector<vector<double>>> path_time_segment_dof(paths_num);
    // vector<vector<double>> path_time(paths_num);
    vector<vector<vector<vector<double>>>> coefficients(paths_num);
    for (int i = 0; i < paths_num; i++) {
        // for 7-dof case
        vector<double> target_velocity(dof_ - 2);
        vector<double> target_acceleration(dof_ - 2);
        vector<double> max_velocity(dof_ - 2);
        vector<double> uniform_velocity(dof_ - 2);
        double time_sum = 0.0;

        target_velocity[0] = target_velocities[i];
        if (degree_ == 5) {
            target_acceleration[0] = target_accelerations[i] * 2.0 / 3.0;
        }
        else if (degree_ == 7) {
            target_acceleration[0] = target_accelerations[i] * 8.0 / 15.0;
        }

        for (int j = 0; j < dof_ - 3; j++) {
            target_velocity[j + 1] = max_rpy_velocity;
            if (degree_ == 5) {
                target_acceleration[j + 1] = max_rpy_acceleration * 2.0 / 3.0;
            }
            else if (degree_ == 7) {
                target_acceleration[j + 1] = max_rpy_acceleration * 8.0 / 15.0;
            }
        }

        if (dof_ == 7) {
            target_velocity[4] = max_redundant_velocity;
            if (degree_ == 5) {
                target_acceleration[4] = max_redundant_acceleration * 2.0 / 3.0;
            }
            else if (degree_ == 7) {
                target_acceleration[4] = max_redundant_acceleration * 8.0 / 15.0;
            }
        }

        // FIXME: 궤적 시간 동기화 방식 변경
        max_velocity = calMaxVelocity(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], distances[i], target_acceleration);
        uniform_velocity = compareVelocity(max_velocity, target_velocity);
        time_sum = compareTimeSum(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], uniform_velocity, distances[i], target_acceleration);
        // path_time[i] = compareTime(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], uniform_velocity, distances[i], target_acceleration);
        uniform_velocity = calUniformVelocity(time_sum, waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], distances[i], target_acceleration);
        // uniform_velocity = recalUniformVelocity(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], distances[i], path_time[i]);
        path_time_segment_dof[i] = calPathTime(waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], uniform_velocity, time_sum, target_acceleration);
        // coefficients[i] = calPathCoefficient(linear_waypoints[i], linear_waypoints[i + 1], waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], uniform_velocity, path_time[i]);
        coefficients[i] = calPathCoefficient(linear_waypoints[i], linear_waypoints[i + 1], waypoints_velocity_temp[i], waypoints_velocity_temp[i + 1], uniform_velocity, path_time_segment_dof[i]);
    }

    // 동기화된 시간을 저장
    vector<vector<vector<double>>> path_time_dof(paths_num);

    // for (int i = 0; i < paths_num; i++) {
    //     vector<double> path_time_sum(4);

    //     path_time_sum[0] = 0;

    //     for (int k = 0; k < 3; k++) {
    //         path_time_sum[k + 1] = path_time_sum[k] + path_time[i][k];
    //     }
    //     // for (int k = 0; k < 4; k++) {
    //     //     path_time_sum[k] = path_time_sum[k] + current_.time;
    //     // }

    //     vector<vector<double>> path_time_temp(dof_ - 2);

    //     for (int j = 0; j < dof_ - 2; j++) {
    //         path_time_temp[j] = path_time_sum;
    //     }

    //     path_time_dof[i] = path_time_temp;
    // }

    // for (int i = 1; i < paths_num; i++) {
    //     double time_fin_before = path_time_dof[i - 1][0][3];

    //     for (int j = 0; j < dof_ - 2; j++) {
    //         for (int k = 0; k < 4; k++) {
    //             path_time_dof[i][j][k] += time_fin_before;
    //         }
    //     }
    // }

    for (int i = 0; i < paths_num; i++) {
        // for 7-dof case
        path_time_dof[i].resize(dof_ - 2);
        for (int k = 0; k < dof_ - 2; k++) {
            path_time_dof[i][k].resize(4);
        }
    }

    for (int i = 0; i < paths_num; i++) {
        for (int k = 0; k < dof_ - 2; k++) {
            for (int j = 0; j < 3; j++) {
                path_time_dof[i][k][j + 1] = path_time_dof[i][k][j] + path_time_segment_dof[i][k][j];
            }
            if (i != paths_num - 1) {
                path_time_dof[i + 1][k][0] = path_time_dof[i][k][3];
            }
        }
    }

    blending_parameter_.radius = radius;
    blending_parameter_.vector_b0 = unit_vector_b0;
    blending_parameter_.vector_b1 = unit_vector_b1;
    blending_parameter_.circle_center_points = circle_center_position;
    blending_parameter_.coefficients = coefficients;
    blending_parameter_.path_time = path_time_dof;
    blending_parameter_.geometric_length = geometric_length;
    blending_parameter_.xyz_unit_distance = xyz_unit_distance;
    blending_parameter_.geometric_waypoints = geometric_waypoints;
    blending_parameter_.path_order = 0;
    blending_parameter_.geometric_order = 0;
    blending_parameter_.geometric_type = true;

    init_.time = current_.time;
    finish_.time = init_.time + path_time_dof[paths_num - 1][0][3];
    finish_.position = target_waypoints[target_waypoints.size() - 1];

    path_type_ = PathType::BLENDING;
    return path_error;
}

void PathPlanner::genSinePath(const double operation_time) {
    for (int i = 0; i < JS_DOF; i++) {
        init_.position[i] = current_.position[i];
    }
    sine_parameter_.operation_time = operation_time;
    sine_parameter_.amplitude = 90;
    init_.time = current_.time;
    path_type_ = PathType::SINE;
}

void PathPlanner::genBackwardPath(const double unit_time) {
    vector<vector<vector<double>>> coefficient;
    init_.position = current_.position;
    init_.velocity = current_.velocity;
    degree_ = 5;

    double unit_time_temp = 0.0;

    if (unit_time != 0.0) {
        unit_time_temp = unit_time;

    } else {
        for (unsigned int i = 0; i < dof_; i++) {
            double temp = divideWithFloatingError(fabs(init_.velocity[i]), 2.0 / 3.0 * target_.acceleration[i]);
            if (temp >= unit_time_temp) {
                unit_time_temp = temp;
            }
        }
    }
    coefficient = calBackwardPolynomialCoefficienit(init_.position, init_.velocity, unit_time_temp);

    vector<double> waypoint(dof_);
    for (unsigned int i = 0; i < dof_; i++) {
        waypoint[i] = init_.position[i] - init_.velocity[i]*unit_time_temp / 2.0;
                if (is_cartesian_space_ && i >= 3) {
            if (waypoint[i] > 180.0) {
                waypoint[i] = waypoint[i] - 360.0;
            }
            else if (waypoint[i] < -180.0) {
                waypoint[i] = waypoint[i] + 360.0;
            }
        }
    }

    backward_parameter_.coefficients = coefficient;
    backward_parameter_.unit_time = unit_time_temp;
    init_.time = current_.time;
    finish_.time = init_.time + unit_time_temp * 3;
    finish_.position = waypoint;

    path_type_ = PathType::BACKWARD;
}

void PathPlanner::cal(const double t) {
    switch (path_type_) {
        case PathType::NONE: {
            desired_.position = init_.position;
            desired_.velocity.resize(dof_);
            desired_.acceleration.resize(dof_);
            break;
        }
        case PathType::STOP: {
            calStopPath(t);
            break;
        }
        case PathType::TRAPEZOID: {
            calTrapezoidPath(t);
            break;
        }
        case PathType::POLYNOMIAL: {
            calPolynomialPath(t);
            break;
        }
        case PathType::LINEAR: {
            calLinearPath(t);
            break;
        }
        case PathType::CIRCULAR: {
            calCircularPath(t);
            break;
        }
        case PathType::B_SPLINE: {
            calBSplineConstVelocity(t);
            break;
        }
        case PathType::BLENDING: {
            calBlendingPath(t);
            break;
        }
        case PathType::SINE: {
            calSinePath(t);
            break;
        }
        case PathType::BACKWARD: {
            calBackwardPath(t);
            break;
        }
        default: {
            break;
        }
    }

    current_.position = desired_.position;
    current_.velocity = desired_.velocity;
    current_.acceleration = desired_.acceleration;
    current_.time = t;
}

bool PathPlanner::isPathOperating() {
    return (path_type_ != PathType::NONE);
}

bool PathPlanner::isCartesianSpace() {
    return is_cartesian_space_;
}

vector<double> PathPlanner::getDesPos() {
    return desired_.position;
}

void PathPlanner::getDesPos(double* position) {
    for (unsigned int i = 0; i < dof_; i++) {
        position[i] = desired_.position[i];
    }
}

void PathPlanner::getDesPos(double& position) {
    if (dof_ == 1) {
        position = desired_.position[0];
    }
}

vector<double> PathPlanner::getDesVel() {
    return desired_.velocity;
}

void PathPlanner::getDesVel(double* velocity) {
    for (unsigned int i = 0; i < dof_; i++) {
        velocity[i] = desired_.velocity[i];
    }
}

vector<double> PathPlanner::getDesAcc() {
    return desired_.acceleration;
}

void PathPlanner::getDesAcc(double* acceleration) {
    for (unsigned int i = 0; i < dof_; i++) {
        acceleration[i] = desired_.acceleration[i];
    }
}

vector<double> PathPlanner::getFinPos() {
    return finish_.position;
}

void PathPlanner::getFinPos(double* position) {
    for (unsigned int i = 0; i < dof_; i++) {
        position[i] = finish_.position[i];
    }
}

PathPlanner::PathError PathPlanner::genCurrentToStartPath(const vector<double> start_waypoint, vector<vector<vector<double>>>& coefficient, vector<vector<double>>& path_time) {
    vector<double> distance(dof_);

    init_.position = current_.position;
    init_.velocity = current_.velocity;

    PathError path_error;
    //  error_type -> PathErrorType::ZERO_TARGET_VEL
    if (isEqual(target_.velocity[0], 0.0) || isEqual(target_.velocity[1], 0.0) || isEqual(target_.velocity[2], 0.0) || isEqual(target_.velocity[3], 0.0) || isEqual(target_.velocity[4], 0.0) || isEqual(target_.velocity[5], 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_VEL;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        return path_error;
    }

    //  error_type -> PathErrorType::ZERO_TARGET_ACC
    if (isEqual(target_.acceleration[0], 0.0) || isEqual(target_.acceleration[1], 0.0) || isEqual(target_.acceleration[2], 0.0) || isEqual(target_.acceleration[3], 0.0) || isEqual(target_.acceleration[4], 0.0) || isEqual(target_.acceleration[5], 0.0)) {
        PathErrorParameter path_error_parameter;
        path_error.type = PathErrorType::ZERO_TARGET_ACC;
        path_error_parameter.waypoint_index.push_back(0);
        path_error.path_error_parameter.push_back(path_error_parameter);
        return path_error;
    }

    for (unsigned int i = 0; i < dof_; i++) {
        distance[i] = start_waypoint[i] - current_.position[i];

        if (is_cartesian_space_ && i >= 3) {
            if (distance[i] > 180.0) {
                distance[i] = distance[i] - 360.0;
            }
            else if (distance[i] < -180.0) {
                distance[i] = distance[i] + 360.0;
            }
        }
    }

    vector<double> max_velocity(dof_);
    vector<double> uniform_velocity(dof_);
    vector<double> polynominal_acceleration(dof_);
    vector<vector<double>> path_time_segment;
    double time_sum = 0.0;

    for (unsigned int i = 0; i < dof_; i++) {
        // �ִ밡�ӵ��� 2/3���� ����
        if (degree_ == 5) {
            polynominal_acceleration[i] = target_.acceleration[i] * 2.0 / 3.0;
        }
        else if (degree_ == 7) {
            polynominal_acceleration[i] = target_.acceleration[i] * 8.0 / 15.0;
        }
        else {
            genNoPath(current_.position);
        }
    }

    max_velocity = calMaxVelocity(init_.velocity, finish_.velocity, distance, polynominal_acceleration);
    uniform_velocity = compareVelocity(max_velocity, target_.velocity);
    time_sum = compareTimeSum(init_.velocity, finish_.velocity, uniform_velocity, distance, polynominal_acceleration);
    uniform_velocity = calUniformVelocity(time_sum, init_.velocity, finish_.velocity, distance, polynominal_acceleration);
    path_time_segment = calPathTime(init_.velocity, finish_.velocity, uniform_velocity, time_sum, polynominal_acceleration);
    coefficient = calPathCoefficient(init_.position, start_waypoint, init_.velocity, finish_.velocity, uniform_velocity, path_time_segment);

    path_time.resize(dof_);
    for (unsigned int k = 0; k < dof_; k++) {
        path_time[k].resize(4);
    }
    for (unsigned int k = 0; k < dof_; k++) {
        for (int j = 0; j < 3; j++) {
            path_time[k][j + 1] = path_time[k][j] + path_time_segment[k][j];
        }
    }

    return path_error;
}

void PathPlanner::genBSplinePolynominalPath(const double length, const double target_velocity, const double target_acceleration) {
    vector<double> bspline_length(1);
    vector<double> init_line_velocity(1);
    vector<double> finish_line_velocity(1);
    vector<double> polynominal_velocity(1);
    vector<double> polynominal_acceleration(1);
    double time_sum = 0.0;
    vector<vector<double>> path_time_segment;
    vector<vector<double>> path_time(1);
    vector<double> uniform_velocity;
    vector<double> init_position(1);
    vector<double> finish_position(1);

    init_line_velocity[0] = sqrt(pow(init_.velocity[0], 2.0) + pow(init_.velocity[1], 2.0) + pow(init_.velocity[2], 2.0));
    finish_line_velocity[0] = sqrt(pow(finish_.velocity[0], 2.0) + pow(finish_.velocity[1], 2.0) + pow(finish_.velocity[2], 2.0));
    polynominal_velocity[0] = target_velocity;
    bspline_length[0] = length;

    // �ִ밡�ӵ��� 2/3���� ����
    if (degree_ == 5) {
        polynominal_acceleration[0] = target_acceleration * 2.0 / 3.0;
    }
    else if (degree_ == 7) {
        polynominal_acceleration[0] = target_acceleration * 8.0 / 15.0;
    }
    else {
        genNoPath(current_.position);
    }

    time_sum = compareTimeSum(init_line_velocity, finish_line_velocity, polynominal_velocity, bspline_length, polynominal_acceleration);
    uniform_velocity = calUniformVelocity(time_sum, init_line_velocity, finish_line_velocity, bspline_length, polynominal_acceleration);
    path_time_segment = calPathTime(init_line_velocity, finish_line_velocity, uniform_velocity, time_sum, polynominal_acceleration);

    init_position[0] = 0.0;
    finish_position[0] = bspline_length[0];

    path_time[0].resize(4);
    for (int i = 0; i < 3; i++) {
        path_time[0][i + 1] = path_time[0][i] + path_time_segment[0][i];
    }

    bSpline_parameter_.coefficient = calPathCoefficient(init_position, finish_position, init_line_velocity, finish_line_velocity, polynominal_velocity, path_time_segment);
    bSpline_parameter_.path_time = path_time;
}

void PathPlanner::genBSplineInterpoleParameter(vector<double>& interpole_parameter, vector<double>& knot_vector) {
    vector<double> length_segment(bSpline_parameter_.waypoint_num - 1);
    double length = 0.0;

    // set interpole parameter according to path law
    if (bSpline_parameter_.path_law == 0) {
        // path_law = 0 : users law
        interpole_parameter[0] = 0.0;
        interpole_parameter[1] = 0.12;
        interpole_parameter[2] = 0.20;
        interpole_parameter[3] = 0.35;
        interpole_parameter[4] = 0.43;
        interpole_parameter[5] = 0.51;
        interpole_parameter[6] = 0.69;
        interpole_parameter[7] = 0.76;
        interpole_parameter[8] = 0.82;
        interpole_parameter[9] = 1.0;
    }
    else if (bSpline_parameter_.path_law == 1) {
        // path_law = 1: Equally spaced Parameter
        interpole_parameter[0] = 0;
        interpole_parameter[bSpline_parameter_.waypoint_num - 1] = bSpline_parameter_.operation_time;
        for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 2; i++) {
            interpole_parameter[i + 1] = (i + 1) * divideWithFloatingError(bSpline_parameter_.operation_time - 0, bSpline_parameter_.waypoint_num - 1);
        }
    }
    else if (bSpline_parameter_.path_law == 2) {
        // path_law = 2: Cord Length spaced Parameter
        for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
            for (int j = 0; j < 3; j++) {
                length_segment[i] += pow((bSpline_parameter_.waypoints[i + 1][j] - bSpline_parameter_.waypoints[i][j]), 2.0);
            }
            length += sqrt(length_segment[i]);
        }
        for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
            interpole_parameter[i + 1] = interpole_parameter[i] + sqrt(length_segment[i]) * divideWithFloatingError(bSpline_parameter_.operation_time, length);
        }
    }
    else {
        // path_law = 3: Centripetal Distribution Parameter
        for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
            for (int j = 0; j < 3; j++) {
                length_segment[i] += pow((bSpline_parameter_.waypoints[i + 1][j] - bSpline_parameter_.waypoints[i][j]), 2.0);
            }
            // Centripetal Distribution' definition
            length += pow(sqrt(length_segment[i]), 0.5);
        }
        for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num - 1; i++) {
            interpole_parameter[i + 1] = interpole_parameter[i] + sqrt(length_segment[i]) * divideWithFloatingError(bSpline_parameter_.operation_time, length);
        }
    }

    // set knot
    knot_vector.clear();
    for (int i = 0; i < bSpline_parameter_.p_degree; i++) {
        knot_vector.push_back(interpole_parameter[0]);
    }
    if (bSpline_parameter_.p_degree % 2 == 0) {
        // p_degree = odd number
        for (unsigned int i = 1; i < bSpline_parameter_.waypoint_num - 1; i++) {
            knot_vector.push_back(interpole_parameter[i]);
        }
    }
    else {
        // p_degree = even number
        for (unsigned int i = 1; i < bSpline_parameter_.waypoint_num; i++) {
            knot_vector.push_back((interpole_parameter[i - 1] + interpole_parameter[i]) / 2.0);
        }
    }
    for (int i = 0; i < bSpline_parameter_.p_degree; i++) {
        knot_vector.push_back(interpole_parameter[bSpline_parameter_.waypoint_num - 1]);
    }
    bSpline_parameter_.knot_vector = knot_vector;
}

void PathPlanner::genBSplineBasisFunction(vector<double> interpole_parameter, vector<double> knot_vector, vector<vector<double>>& basis, vector<vector<vector<double>>>& basis_total) {
    basis_total.resize(bSpline_parameter_.p_degree);
    for (int i = 0; i < bSpline_parameter_.p_degree; i++) {
        basis_total[i].resize(bSpline_parameter_.controlpoint_num);
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
            basis_total[i][j].resize(bSpline_parameter_.waypoint_num);
        }
    }

    basis.resize(bSpline_parameter_.controlpoint_num);
    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num; i++) {
        basis[i].resize(bSpline_parameter_.waypoint_num);
    }

    double DL = 0.0;
    double DR = 0.0;
    double dn1 = 0.0;
    double dd1 = 0.0;
    double dn2 = 0.0;
    double dd2 = 0.0;

    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num; i++) {
        // for every degree, for p_degree = 0;
        if (interpole_parameter[i] - bSpline_parameter_.operation_time > 0.000000001) {
            //interpole_parameter < operation_time
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
                if (knot_vector[j] <= interpole_parameter[i] && interpole_parameter[i] < knot_vector[j + 1]) {
                    basis_total[0][j][i] = 1.0;
                }
                else {
                    basis_total[0][j][i] = 0.0;
                }
            }
        }
        else {
            //interpole_parameter == operation_time
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
                if (j != bSpline_parameter_.controlpoint_num - 1) {
                    if (knot_vector[j] <= interpole_parameter[i] && interpole_parameter[i] < knot_vector[j + 1]) {
                        basis_total[0][j][i] = 1.0;
                    }
                    else {
                        basis_total[0][j][i] = 0.0;
                    }
                }
                else {
                    if (knot_vector[j] <= interpole_parameter[i]) {
                        basis_total[0][j][i] = 1.0;
                    }
                    else {
                        basis_total[0][j][i] = 0.0;
                    }
                }
            }
        }

        // for every degree, for p_degree > 0;
        for (int p = 1; p < bSpline_parameter_.p_degree; p++) {
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
                if (j != bSpline_parameter_.controlpoint_num - 1) {
                    dn1 = interpole_parameter[i] - knot_vector[j];
                    dd1 = knot_vector[j + p] - knot_vector[j];
                    DL = divideWithFloatingError(dn1, dd1);

                    dn2 = knot_vector[j + p + 1] - interpole_parameter[i];
                    dd2 = knot_vector[j + p + 1] - knot_vector[j + 1];
                    DR = divideWithFloatingError(dn2, dd2);

                    basis_total[p][j][i] = DL * basis_total[p - 1][j][i] + DR * basis_total[p - 1][j + 1][i];
                    basis[j][i] = basis_total[p][j][i];
                }
                else {
                    dn1 = interpole_parameter[i] - knot_vector[j];
                    dd1 = knot_vector[j + p] - knot_vector[j];
                    DL = divideWithFloatingError(dn1, dd1);

                    dn2 = knot_vector[j + p + 1] - interpole_parameter[i];
                    dd2 = knot_vector[j + p + 1] - knot_vector[j + 1];
                    DR = divideWithFloatingError(dn2, dd2);

                    basis_total[p][j][i] = DL * basis_total[p - 1][j][i];
                    basis[j][i] = basis_total[p][j][i];
                }
            }
        }
    }
}

void PathPlanner::genBSplineBasisFunctionDerivative(vector<double> knot_vector, vector<vector<vector<double>>>& basis_total, vector<vector<double>>& basis_first_derivative, vector<vector<double>>& basis_second_derivative, vector<vector<double>>& basis_third_derivative, vector<vector<double>>& basis_fourth_derivative) {
    // calculate bspline basis derivative
    basis_first_derivative.resize(bSpline_parameter_.controlpoint_num);
    basis_second_derivative.resize(bSpline_parameter_.controlpoint_num);
    basis_third_derivative.resize(bSpline_parameter_.controlpoint_num);
    basis_fourth_derivative.resize(bSpline_parameter_.controlpoint_num);
    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num; i++) {
        basis_first_derivative[i].resize(bSpline_parameter_.waypoint_num);
        basis_second_derivative[i].resize(bSpline_parameter_.waypoint_num);
        basis_third_derivative[i].resize(bSpline_parameter_.waypoint_num);
        basis_fourth_derivative[i].resize(bSpline_parameter_.waypoint_num);
    }

    vector<vector<vector<double>>> basis_all_derivative;
    vector<vector<vector<double>>> temp_a;

    basis_all_derivative.resize(bSpline_parameter_.p_degree);
    temp_a.resize(bSpline_parameter_.p_degree);
    for (int i = 0; i < bSpline_parameter_.p_degree; i++) {
        basis_all_derivative[i].resize(bSpline_parameter_.controlpoint_num);
        temp_a[i].resize(bSpline_parameter_.p_degree);
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
            basis_all_derivative[i][j].resize(bSpline_parameter_.waypoint_num);
        }
        for (int j = 0; j < bSpline_parameter_.p_degree; j++) {
            temp_a[i][j].resize(bSpline_parameter_.controlpoint_num);
        }
    }

    // generate 'a'
    for (int k = 0; k < bSpline_parameter_.p_degree; k++) {
        for (int i = 0; i < k + 1; i++) {
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
                if (k == 0 && i == 0) {
                    temp_a[k][i][j] = 1;
                }
                else if (i == 0) {
                    if ((knot_vector[j + bSpline_parameter_.p_degree - k] - knot_vector[j]) != 0.0) {
                        temp_a[k][i][j] = divideWithFloatingError(temp_a[k - 1][i][j], knot_vector[j + bSpline_parameter_.p_degree - k] - knot_vector[j]);
                    }
                    else {
                        temp_a[k][i][j] = 0.0;
                    }
                }
                else if (i == k) {
                    if ((knot_vector[j + bSpline_parameter_.p_degree] - knot_vector[j + k]) != 0.0) {
                        temp_a[k][i][j] = divideWithFloatingError(-temp_a[k - 1][k - 1][j], knot_vector[j + bSpline_parameter_.p_degree] - knot_vector[j + k]);
                    }
                    else {
                        temp_a[k][i][j] = 0.0;
                    }
                }
                else {
                    if ((knot_vector[j + bSpline_parameter_.p_degree + i - k] - knot_vector[j + i]) != 0) {
                        temp_a[k][i][j] = divideWithFloatingError((temp_a[k - 1][i][j] - temp_a[k - 1][i - 1][j]), (knot_vector[j + bSpline_parameter_.p_degree + i - k] - knot_vector[j + i]));
                    }
                    else {
                        temp_a[k][i][j] = 0.0;
                    }
                }
            }
        }
    }

    // generate basis_derivative
    for (unsigned int u = 0; u < bSpline_parameter_.waypoint_num; u++) {
        for (int k = 0; k < bSpline_parameter_.p_degree; k++) {
            for (int i = 0; i < k + 1; i++) {
                for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
                    double factorial = 1.0;
                    for (int f = bSpline_parameter_.p_degree - k; f < bSpline_parameter_.p_degree; f++) {
                        factorial *= f;
                    }
                    if (j + i >= bSpline_parameter_.controlpoint_num) {
                        basis_all_derivative[k][j][u] += factorial * temp_a[k][i][j] * basis_total[bSpline_parameter_.p_degree - k][j + i - bSpline_parameter_.controlpoint_num][u];
                    }
                    else {
                        basis_all_derivative[k][j][u] += factorial * temp_a[k][i][j] * basis_total[bSpline_parameter_.p_degree - 1 - k][j + i][u];
                    }
                }
            }
        }
    }

    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
            basis_first_derivative[j][i] = basis_all_derivative[1][j][i];
            basis_second_derivative[j][i] = basis_all_derivative[2][j][i];
            if (bSpline_parameter_.p_degree > 3) {
                basis_third_derivative[j][i] = basis_all_derivative[3][j][i];
            }
            else if (bSpline_parameter_.p_degree > 4) {
                basis_fourth_derivative[j][i] = basis_all_derivative[4][j][i];
            }
        }
    }
}

void PathPlanner::genBSplineControlPoint(vector<vector<double>> basis, vector<vector<double>> basis_first_derivative, vector<vector<double>> basis_second_derivative, vector<vector<double>> basis_third_derivative) {
    // generate control point
    ModelMatrix matrix_r;
    ModelMatrix matrix_b;

    vector<vector<double>> basis_transpose;
    vector<vector<double>> basis_first_derivative_transpose;
    vector<vector<double>> basis_second_derivative_transpose;
    vector<vector<double>> basis_third_derivative_transpose;

    vector<double> first_derivative_knot_vector;
    vector<double> second_derivative_knot_vector;

    basis_transpose.resize(bSpline_parameter_.waypoint_num);
    basis_first_derivative_transpose.resize(bSpline_parameter_.waypoint_num);
    basis_second_derivative_transpose.resize(bSpline_parameter_.waypoint_num);
    basis_third_derivative_transpose.resize(bSpline_parameter_.waypoint_num);

    for (unsigned int i = 0; i < bSpline_parameter_.waypoint_num; i++) {
        basis_transpose[i].resize(bSpline_parameter_.controlpoint_num);
        basis_first_derivative_transpose[i].resize(bSpline_parameter_.controlpoint_num);
        basis_second_derivative_transpose[i].resize(bSpline_parameter_.controlpoint_num);
        basis_third_derivative_transpose[i].resize(bSpline_parameter_.controlpoint_num);
    }

    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.waypoint_num; j++) {
            basis_transpose[j][i] = basis[i][j];
            basis_first_derivative_transpose[j][i] = basis_first_derivative[i][j];
            basis_second_derivative_transpose[j][i] = basis_second_derivative[i][j];
            basis_third_derivative_transpose[j][i] = basis_third_derivative[i][j];
        }
    }
    vector<double> curvature_vector(dof_);
    bSpline_parameter_.controlpoints.clear();
    bSpline_parameter_.controlpoints.resize(bSpline_parameter_.controlpoint_num);
    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num; i++) {
        bSpline_parameter_.controlpoints[i].resize(dof_);
    }

    // generate 'r'
    vector<double> matrix_r_element;
    matrix_r_element.insert(matrix_r_element.end(), bSpline_parameter_.waypoints[0].begin(), bSpline_parameter_.waypoints[0].end());
    matrix_r_element.insert(matrix_r_element.end(), bSpline_parameter_.tangent_vector[0].begin(), bSpline_parameter_.tangent_vector[0].end());
    if (bSpline_parameter_.p_degree > 4) {
        matrix_r_element.insert(matrix_r_element.end(), curvature_vector.begin(), curvature_vector.end());
    }
    for (unsigned int i = 1; i < bSpline_parameter_.waypoint_num - 1; i++) {
        matrix_r_element.insert(matrix_r_element.end(), bSpline_parameter_.waypoints[i].begin(), bSpline_parameter_.waypoints[i].end());
    }
    if (bSpline_parameter_.p_degree > 4) {
        matrix_r_element.insert(matrix_r_element.end(), curvature_vector.begin(), curvature_vector.end());
    }
    matrix_r_element.insert(matrix_r_element.end(), bSpline_parameter_.tangent_vector[1].begin(), bSpline_parameter_.tangent_vector[1].end());
    matrix_r_element.insert(matrix_r_element.end(), bSpline_parameter_.waypoints[bSpline_parameter_.waypoint_num - 1].begin(), bSpline_parameter_.waypoints[bSpline_parameter_.waypoint_num - 1].end());

    if (bSpline_parameter_.p_degree > 4) {
        matrix_r = ModelMatrix(bSpline_parameter_.waypoint_num + 4, 6, matrix_r_element);
    }
    else {
        matrix_r = ModelMatrix(bSpline_parameter_.waypoint_num + 2, 6, matrix_r_element);
    }

    // generate 'b'
    vector<double> matrix_b_element;
    matrix_b_element.insert(matrix_b_element.end(), basis_transpose[0].begin(), basis_transpose[0].end());
    matrix_b_element.insert(matrix_b_element.end(), basis_first_derivative_transpose[0].begin(), basis_first_derivative_transpose[0].end());
    if (bSpline_parameter_.p_degree > 4) {
        matrix_b_element.insert(matrix_b_element.end(), basis_second_derivative_transpose[0].begin(), basis_second_derivative_transpose[0].end());
    }
    for (unsigned int i = 1; i < bSpline_parameter_.waypoint_num - 1; i++) {
        matrix_b_element.insert(matrix_b_element.end(), basis_transpose[i].begin(), basis_transpose[i].end());
    }
    if (bSpline_parameter_.p_degree > 4) {
        matrix_b_element.insert(matrix_b_element.end(), basis_second_derivative_transpose[bSpline_parameter_.waypoint_num - 1].begin(), basis_second_derivative_transpose[bSpline_parameter_.waypoint_num - 1].end());
    }
    matrix_b_element.insert(matrix_b_element.end(), basis_first_derivative_transpose[bSpline_parameter_.waypoint_num - 1].begin(), basis_first_derivative_transpose[bSpline_parameter_.waypoint_num - 1].end());
    matrix_b_element.insert(matrix_b_element.end(), basis_transpose[bSpline_parameter_.waypoint_num - 1].begin(), basis_transpose[bSpline_parameter_.waypoint_num - 1].end());

    if (bSpline_parameter_.p_degree > 4) {
        matrix_b = ModelMatrix(bSpline_parameter_.waypoint_num + 4, bSpline_parameter_.controlpoint_num, matrix_b_element);
    }
    else {
        matrix_b = ModelMatrix(bSpline_parameter_.waypoint_num + 2, bSpline_parameter_.controlpoint_num, matrix_b_element);
    }

    // generate 'b_inverse'
    ModelMatrix matrix_b_inverse = matrix_b.inverse();

    // generate control point
    for (int k = 0; k < 6; k++) {
        for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num; i++) {
            for (unsigned int j = 0; j < matrix_r.row(); j++) {
                bSpline_parameter_.controlpoints[i][k] += matrix_b_inverse.get(i, j) * matrix_r.get(j, k);
            }
        }
    }

    // generate control point derivative
    first_derivative_knot_vector.resize(bSpline_parameter_.knot_number - 2);
    second_derivative_knot_vector.resize(bSpline_parameter_.knot_number - 4);
    bSpline_parameter_.controlpoints_first_derivative.resize(bSpline_parameter_.controlpoint_num - 1);
    bSpline_parameter_.controlpoints_second_derivative.resize(bSpline_parameter_.controlpoint_num - 2);
    bSpline_parameter_.controlpoints_third_derivative.resize(bSpline_parameter_.controlpoint_num - 3);

    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num - 1; i++) {
        bSpline_parameter_.controlpoints_first_derivative[i].resize(dof_);
    }

    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num - 2; i++) {
        bSpline_parameter_.controlpoints_second_derivative[i].resize(dof_);
    }

    for (unsigned int i = 0; i < bSpline_parameter_.controlpoint_num - 3; i++) {
        bSpline_parameter_.controlpoints_third_derivative[i].resize(dof_);
    }

    // derivative of the knotvector
    for (int i = 0; i < bSpline_parameter_.knot_number - 2; i++) {
        first_derivative_knot_vector[i] = bSpline_parameter_.knot_vector[i + 1];
    }

    for (int i = 0; i < bSpline_parameter_.knot_number - 4; i++) {
        second_derivative_knot_vector[i] = first_derivative_knot_vector[i + 1];
    }

    // 1 derivative of the control points
    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - 1; j++) {
            bSpline_parameter_.controlpoints_first_derivative[j][i] = divideWithFloatingError((bSpline_parameter_.p_degree - 1) * (bSpline_parameter_.controlpoints[j + 1][i] - bSpline_parameter_.controlpoints[j][i]), (bSpline_parameter_.knot_vector[j + bSpline_parameter_.p_degree] - bSpline_parameter_.knot_vector[j + 1]));
        }
    }

    // 2 derivative of the control points
    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - 2; j++) {
            bSpline_parameter_.controlpoints_second_derivative[j][i] = divideWithFloatingError((bSpline_parameter_.p_degree - 2) * (bSpline_parameter_.controlpoints_first_derivative[j + 1][i] - bSpline_parameter_.controlpoints_first_derivative[j][i]), (first_derivative_knot_vector[j + bSpline_parameter_.p_degree - 1] - first_derivative_knot_vector[j + 1]));
        }
    }

    // 3 derivative of the control points
    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - 3; j++) {
            bSpline_parameter_.controlpoints_third_derivative[j][i] = divideWithFloatingError((bSpline_parameter_.p_degree - 3) * (bSpline_parameter_.controlpoints_second_derivative[j + 1][i] - bSpline_parameter_.controlpoints_second_derivative[j][i]), (second_derivative_knot_vector[j + bSpline_parameter_.p_degree - 2] - second_derivative_knot_vector[j + 1]));
        }
    }
}

void PathPlanner::calStopPath(const double t) {
    for (unsigned int i = 0; i < dof_; i++) {
        desired_.position[i] = 0.0;
        desired_.velocity[i] = 0.0;
        desired_.acceleration[i] = 0.0;

        if (t < init_.time) {
            desired_.position[i] = init_.position[i];
            desired_.velocity[i] = init_.velocity[i];
            desired_.acceleration[i] = 0.0;
        }
        else if (t < finish_.time) {
            double t0 = init_.time;
            for (int k = 0; k < 6; k++) {
                desired_.position[i] += stop_parameter_.coefficient[i][k] * pow(t - t0, k);
            }
            for (int k = 1; k < 6; k++) {
                desired_.velocity[i] += k * stop_parameter_.coefficient[i][k] * pow(t - t0, k - 1);
            }
            for (int k = 2; k < 6; k++) {
                desired_.acceleration[i] += k * (k - 1) * stop_parameter_.coefficient[i][k] * pow(t - t0, k - 2);
            }
        }
        else {
            desired_.position[i] = finish_.position[i];
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
            genNoPath(desired_.position);
        }
    }
}

void PathPlanner::calTrapezoidPath(const double t) {
    for (unsigned int i = 0; i < dof_; i++) {
        if (t < init_.time) {
            //���� ����
            desired_.position[i] = init_.position[i];
            desired_.velocity[i] = init_.velocity[i];
            desired_.acceleration[i] = 0.0;
        }
        else if (init_.time <= t && t < finish_.time) {
            desired_.position[i] = 0.0;
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
            if (trapezoid_Parameter_.path_time[i][0] <= t && t < trapezoid_Parameter_.path_time[i][1]) {
                //���� ����
                double t0 = trapezoid_Parameter_.path_time[i][0];

                for (int k = 0; k < 3; k++) {
                    desired_.position[i] += trapezoid_Parameter_.coefficient[0][i][k] * pow(t - t0, k);
                }
                for (int k = 1; k < 3; k++) {
                    desired_.velocity[i] += k * trapezoid_Parameter_.coefficient[0][i][k] * pow(t - t0, k - 1);
                }
                for (int k = 2; k < 3; k++) {
                    desired_.acceleration[i] += k * (k - 1) * trapezoid_Parameter_.coefficient[0][i][k] * pow(t - t0, k - 2);
                }
            }
            else if (trapezoid_Parameter_.path_time[i][1] <= t && t < trapezoid_Parameter_.path_time[i][2]) {
                //��� ����
                double t1 = trapezoid_Parameter_.path_time[i][1];
                for (int k = 0; k < 3; k++) {
                    desired_.position[i] += trapezoid_Parameter_.coefficient[1][i][k] * pow(t - t1, k);
                }
                for (int k = 1; k < 3; k++) {
                    desired_.velocity[i] += k * trapezoid_Parameter_.coefficient[1][i][k] * pow(t - t1, k - 1);
                }
                for (int k = 2; k < 3; k++) {
                    desired_.acceleration[i] += k * (k - 1) * trapezoid_Parameter_.coefficient[1][i][k] * pow(t - t1, k - 2);
                }
            }
            else if (trapezoid_Parameter_.path_time[i][2] <= t && t < trapezoid_Parameter_.path_time[i][3]) {
                //���� ����
                double t2 = trapezoid_Parameter_.path_time[i][2];
                for (int k = 0; k < 3; k++) {
                    desired_.position[i] += trapezoid_Parameter_.coefficient[2][i][k] * pow(t - t2, k);
                }
                for (int k = 1; k < 3; k++) {
                    desired_.velocity[i] += k * trapezoid_Parameter_.coefficient[2][i][k] * pow(t - t2, k - 1);
                }
                for (int k = 2; k < 3; k++) {
                    desired_.acceleration[i] += k * (k - 1) * trapezoid_Parameter_.coefficient[2][i][k] * pow(t - t2, k - 2);
                }
            }
        }
        else {
            //���� ����
            desired_.position[i] = finish_.position[i];
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
            genNoPath(desired_.position);
        }
    }
}

void PathPlanner::calPolynomialPath(const double t) {
    int degree = degree_ + 1;
    for (unsigned int i = 0; i < dof_; i++) {
        if (t < init_.time) {
            //���� ����
            desired_.position[i] = init_.position[i];
            desired_.velocity[i] = init_.velocity[i];
            desired_.acceleration[i] = 0.0;
        }
        else if (init_.time <= t && finish_.time > t) {
            desired_.position[i] = 0.0;
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
            if (polynomial_parameter_.path_time[polynomial_parameter_.count_num][0][3] < t) {
                //waypoint ��ȭ
                polynomial_parameter_.count_num++;
            }
            if (polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][0] <= t && t < polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][1]) {
                //���� ����
                double t0 = polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][0];
                for (int k = 0; k < degree; k++) {
                    desired_.position[i] += polynomial_parameter_.coefficients[polynomial_parameter_.count_num][0][i][k] * pow(t - t0, k);
                }
                for (int k = 1; k < degree; k++) {
                    desired_.velocity[i] += k * polynomial_parameter_.coefficients[polynomial_parameter_.count_num][0][i][k] * pow(t - t0, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desired_.acceleration[i] += k * (k - 1) * polynomial_parameter_.coefficients[polynomial_parameter_.count_num][0][i][k] * pow(t - t0, k - 2);
                }
            }
            else if (polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][1] <= t && t < polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][2]) {
                //��� ����
                double t1 = polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][1];
                for (int k = 0; k < degree; k++) {
                    desired_.position[i] += polynomial_parameter_.coefficients[polynomial_parameter_.count_num][1][i][k] * pow(t - t1, k);
                }
                for (int k = 1; k < degree; k++) {
                    desired_.velocity[i] += k * polynomial_parameter_.coefficients[polynomial_parameter_.count_num][1][i][k] * pow(t - t1, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desired_.acceleration[i] += k * (k - 1) * polynomial_parameter_.coefficients[polynomial_parameter_.count_num][1][i][k] * pow(t - t1, k - 2);
                }
            }
            else if (polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][2] <= t && finish_.time > t) {
             //���� ����
                double t2 = polynomial_parameter_.path_time[polynomial_parameter_.count_num][i][2];
                for (int k = 0; k < degree; k++) {
                    desired_.position[i] += polynomial_parameter_.coefficients[polynomial_parameter_.count_num][2][i][k] * pow(t - t2, k);
                }
                for (int k = 1; k < degree; k++) {
                    desired_.velocity[i] += k * polynomial_parameter_.coefficients[polynomial_parameter_.count_num][2][i][k] * pow(t - t2, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desired_.acceleration[i] += k * (k - 1) * polynomial_parameter_.coefficients[polynomial_parameter_.count_num][2][i][k] * pow(t - t2, k - 2);
                }
            }
        }
        else {
            //���� ����
            desired_.position[i] = finish_.position[i];
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
            genNoPath(desired_.position);
        }
    }
}

void PathPlanner::calLinearPath(const double t) {
    int degree = degree_ + 1;
    double desired_linear_position = 0.0;
    double desired_linear_velocity = 0.0;
    double desired_linear_acceleration = 0.0;

    if (t < init_.time) {
        //���� ����
        desired_.position = init_.position;
        desired_.velocity = init_.velocity;
        for (unsigned int i = 0; i < dof_; i++) {
            desired_.acceleration[i] = 0.0;
        }
    }
    else if (init_.time <= t && finish_.time > t) {
        for (unsigned int i = 0; i < dof_; i++) {
            desired_.position[i] = 0.0;
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
        }

        if (linear_parameter_.path_time[0][0] <= t && t < linear_parameter_.path_time[0][1]) {
            //���� ����
            double t0 = linear_parameter_.path_time[0][0];
            for (int k = 0; k < degree; k++) {
                desired_linear_position += linear_parameter_.coefficients[0][0][k] * pow(t - t0, k);
                for (int i = 0; i < 3; i++) {
                    desired_.position[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_position + init_.position[i];
                }
            }
            for (int k = 1; k < degree; k++) {
                desired_linear_velocity += k * linear_parameter_.coefficients[0][0][k] * pow(t - t0, k - 1);
                for (int i = 0; i < 3; i++) {
                    desired_.velocity[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_velocity;
                }
            }
            for (int k = 2; k < degree; k++) {
                desired_linear_acceleration += k * (k - 1) * linear_parameter_.coefficients[0][0][k] * pow(t - t0, k - 2);
                for (int i = 0; i < 3; i++) {
                    desired_.acceleration[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_acceleration;
                }
            }
        }
        else if (linear_parameter_.path_time[0][1] <= t && t < linear_parameter_.path_time[0][2]) {
            //��� ����
            double t1 = linear_parameter_.path_time[0][1];
            for (int k = 0; k < degree; k++) {
                desired_linear_position += linear_parameter_.coefficients[1][0][k] * pow(t - t1, k);
                for (int i = 0; i < 3; i++) {
                    desired_.position[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_position + init_.position[i];
                }
            }
            for (int k = 1; k < degree; k++) {
                desired_linear_velocity += k * linear_parameter_.coefficients[1][0][k] * pow(t - t1, k - 1);
                for (int i = 0; i < 3; i++) {
                    desired_.velocity[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_velocity;
                }
            }
            for (int k = 2; k < degree; k++) {
                desired_linear_acceleration += k * (k - 1) * linear_parameter_.coefficients[1][0][k] * pow(t - t1, k - 2);
                for (int i = 0; i < 3; i++) {
                    desired_.acceleration[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_acceleration;
                }
            }
        }
        else if (linear_parameter_.path_time[0][2] <= t && t < linear_parameter_.path_time[0][3]) {
            //���� ����
            double t2 = linear_parameter_.path_time[0][2];
            for (int k = 0; k < degree; k++) {
                desired_linear_position += linear_parameter_.coefficients[2][0][k] * pow(t - t2, k);
                for (int i = 0; i < 3; i++) {
                    desired_.position[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_position + init_.position[i];
                }
            }
            for (int k = 1; k < degree; k++) {
                desired_linear_velocity += k * linear_parameter_.coefficients[2][0][k] * pow(t - t2, k - 1);
                for (int i = 0; i < 3; i++) {
                    desired_.velocity[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_velocity;
                }
            }
            for (int k = 2; k < degree; k++) {
                desired_linear_acceleration += k * (k - 1) * linear_parameter_.coefficients[2][0][k] * pow(t - t2, k - 2);
                for (int i = 0; i < 3; i++) {
                    desired_.acceleration[i] = linear_parameter_.distance_unit_vector[i] * desired_linear_acceleration;
                }
            }
        }

        for (int i = 0; i < 3; i++) {
            if (linear_parameter_.path_time[i + 1][0] <= t && t < linear_parameter_.path_time[i + 1][1]) {
                //���� ����
                double t0 = linear_parameter_.path_time[i + 1][0];
                for (int k = 0; k < degree; k++) {
                    desired_.position[i + 3] += linear_parameter_.coefficients[0][i + 1][k] * pow(t - t0, k);
                }
                for (int k = 1; k < degree; k++) {

                    desired_.velocity[i + 3] += k * linear_parameter_.coefficients[0][i + 1][k] * pow(t - t0, k - 1);
                }
                for (int k = 2; k < degree; k++) {

                    desired_.acceleration[i + 3] += k * (k - 1) * linear_parameter_.coefficients[0][i + 1][k] * pow(t - t0, k - 2);
                }
            }
            else if (linear_parameter_.path_time[i + 1][1] <= t && t < linear_parameter_.path_time[i + 1][2]) {
                //��� ����
                double t1 = linear_parameter_.path_time[i + 1][1];
                for (int k = 0; k < degree; k++) {
                    desired_.position[i + 3] += linear_parameter_.coefficients[1][i + 1][k] * pow(t - t1, k);
                }
                for (int k = 1; k < degree; k++) {
                    desired_.velocity[i + 3] += k * linear_parameter_.coefficients[1][i + 1][k] * pow(t - t1, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desired_.acceleration[i + 3] += k * (k - 1) * linear_parameter_.coefficients[1][i + 1][k] * pow(t - t1, k - 2);
                }
            }
            else if (linear_parameter_.path_time[i + 1][2] <= t && t < linear_parameter_.path_time[i + 1][3]) {
                //���� ����
                double t2 = linear_parameter_.path_time[i + 1][2];
                for (int k = 0; k < degree; k++) {
                    desired_.position[i + 3] += linear_parameter_.coefficients[2][i + 1][k] * pow(t - t2, k);
                }
                for (int k = 1; k < degree; k++) {
                    desired_.velocity[i + 3] += k * linear_parameter_.coefficients[2][i + 1][k] * pow(t - t2, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desired_.acceleration[i + 3] += k * (k - 1) * linear_parameter_.coefficients[2][i + 1][k] * pow(t - t2, k - 2);
                }
            }
        }
    }
    else {
        //���� ����
        desired_.position = finish_.position;
        for (unsigned int i = 0; i < dof_; i++) {
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
        }
        genNoPath(desired_.position);
    }
}


void PathPlanner::calCircularPath(const double t) {
    int degree = degree_ + 1;
    // �ǽð����� ��ȣ������ �����ϴ� �Լ�
    // �ǽð� �ð��� t�� �Է����� �޾Ƽ� �� �����ֱ⸶�� �κ������� desired ��ġ, �ӵ�, ���ӵ��� ������
    // ���� ����
    vector<double> desire_angle_position;
    vector<double> desire_angle_velocity;
    vector<double> desire_angle_acceleration;

    vector<double> desire_position(dof_);
    vector<double> desire_velocity(dof_);
    vector<double> desire_acceleration(dof_);

    // ��ȣ������ �ð��� ����/���/���� �������� ������ �ش�ð��� �ش��Լ��� �����ϴ� ������ �Ǿ�����
    // �� �Լ��� ���� ���� �� ���� ������ �Լ� ���ο� �ּ����� ������
    if (t < init_.time) { // ��ȣ���� ���� ������ ������ġ ����
        for (unsigned int i = 0; i < dof_; i++) {
            desire_position[i] = init_.position[i];
            desire_velocity[i] = init_.velocity[i];
            desire_acceleration[i] = 0.0;
        }
    }
    else if (t >= init_.time && t < init_.time + circular_parameter_.start_path_time[0][3]) {
        for (unsigned int i = 0; i < dof_; i++) {
            if (init_.time + circular_parameter_.start_path_time[i][0] <= t && t < init_.time + circular_parameter_.start_path_time[i][1]) {
                double t0 = init_.time + circular_parameter_.start_path_time[i][0];
                desire_position[i] = 0.0;
                desire_velocity[i] = 0.0;
                desire_acceleration[i] = 0.0;

                for (int k = 0; k < degree; k++) {
                    desire_position[i] += circular_parameter_.start_coefficient[0][i][k] * pow(t - t0, k);
                }
                for (int k = 1; k < degree; k++) {
                    desire_velocity[i] += k * circular_parameter_.start_coefficient[0][i][k] * pow(t - t0, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desire_acceleration[i] += k * (k - 1) * circular_parameter_.start_coefficient[0][i][k] * pow(t - t0, k - 2);
                }
            }
            else if (init_.time + circular_parameter_.start_path_time[i][1] <= t && t < init_.time + circular_parameter_.start_path_time[i][2]) {
                double t1 = init_.time + circular_parameter_.start_path_time[i][1];
                desire_position[i] = 0.0;
                desire_velocity[i] = 0.0;
                desire_acceleration[i] = 0.0;

                for (int k = 0; k < degree; k++) {
                    desire_position[i] += circular_parameter_.start_coefficient[1][i][k] * pow(t - t1, k);
                }
                for (int k = 1; k < degree; k++) {
                    desire_velocity[i] += k * circular_parameter_.start_coefficient[1][i][k] * pow(t - t1, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desire_acceleration[i] += k * (k - 1) * circular_parameter_.start_coefficient[1][i][k] * pow(t - t1, k - 2);
                }
            }
            else if (init_.time + circular_parameter_.start_path_time[i][2] <= t && t < init_.time + circular_parameter_.start_path_time[i][3]) {
                double t2 = init_.time + circular_parameter_.start_path_time[i][2];
                desire_position[i] = 0.0;
                desire_velocity[i] = 0.0;
                desire_acceleration[i] = 0.0;

                for (int k = 0; k < degree; k++) {
                    desire_position[i] += circular_parameter_.start_coefficient[2][i][k] * pow(t - t2, k);
                }
                for (int k = 1; k < degree; k++) {
                    desire_velocity[i] += k * circular_parameter_.start_coefficient[2][i][k] * pow(t - t2, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    desire_acceleration[i] += k * (k - 1) * circular_parameter_.start_coefficient[2][i][k] * pow(t - t2, k - 2);
                }
            }
        }
    }
    else if (init_.time + circular_parameter_.start_path_time[0][3] <= t && t < init_.time + circular_parameter_.start_path_time[0][3] + circular_parameter_.path_time[0][3]) {
        double circular_start_time = init_.time + circular_parameter_.start_path_time[0][3];

        if (t >= circular_start_time && t < circular_start_time + circular_parameter_.path_time[0][1]) { // ���۽������� ���ӱ��� ����������� �Ʒ��� �Լ� ����
            // ���ӱ��� �Ǵ� ���ӱ����� 5�� ���׽� ����(desired ������, ���ӵ� �� �����ӵ�) ���� �Լ� (�ð��� ���� 1���� �Լ��� ���������� �����ϴ� �Լ�)
            calAccelerationPath(t - circular_start_time, circular_parameter_.coefficient[0], desire_angle_position, desire_angle_velocity, desire_angle_acceleration);
            // �ð��� ���� 1���� �Լ��� ������, ���ӵ�, �����ӵ� ������ ��ȣ����� �������� ������ �ռ��Ͽ� �ð��� ���� 6���� �Լ��� ��ġ, �ӵ�, ���ӵ� �������� ��ȯ�ϴ� �Լ�
            calThetatoPose(desire_angle_position[0], desire_angle_velocity[0], desire_angle_acceleration[0], desire_position, desire_velocity, desire_acceleration);
        }
        else if (t >= circular_start_time + circular_parameter_.path_time[0][1] && t < circular_start_time + circular_parameter_.path_time[0][2]) { // ���ӱ��� ����������� ��ӱ��� ����������� �Ʒ��� �Լ� ����
         // ��ӱ����� desired ������, ���ӵ� �� �����ӵ��� �����ϴ� �Լ� (�ð��� ���� 1���� �Լ��� ���������� �����ϴ� �Լ�)
            calCruisePath(t - circular_start_time - circular_parameter_.path_time[0][1], circular_parameter_.coefficient[1], desire_angle_position, desire_angle_velocity, desire_angle_acceleration);
            // �ð��� ���� 1���� �Լ��� ������, ���ӵ�, �����ӵ� ������ ��ȣ����� �������� ������ �ռ��Ͽ� �ð��� ���� 6���� �Լ��� ��ġ, �ӵ�, ���ӵ� �������� ��ȯ�ϴ� �Լ�
            calThetatoPose(desire_angle_position[0], desire_angle_velocity[0], desire_angle_acceleration[0], desire_position, desire_velocity, desire_acceleration);
        }
        else if (t >= circular_start_time + circular_parameter_.path_time[0][2] && t < circular_start_time + circular_parameter_.path_time[0][3]) { // ��ӱ��� ����������� ���ӱ��� ����������� �Ʒ��� �Լ� ����
         // ���ӱ��� �Ǵ� ���ӱ����� 5�� ���׽� ����(desired ������, ���ӵ� �� �����ӵ�) ���� �Լ� (�ð��� ���� 1���� �Լ��� ���������� �����ϴ� �Լ�)
            calAccelerationPath(t - circular_start_time - circular_parameter_.path_time[0][2], circular_parameter_.coefficient[2], desire_angle_position, desire_angle_velocity, desire_angle_acceleration);
            // �ð��� ���� 1���� �Լ��� ������, ���ӵ�, �����ӵ� ������ ��ȣ����� �������� ������ �ռ��Ͽ� �ð��� ���� 6���� �Լ��� ��ġ, �ӵ�, ���ӵ� �������� ��ȯ�ϴ� �Լ�
            calThetatoPose(desire_angle_position[0], desire_angle_velocity[0], desire_angle_acceleration[0], desire_position, desire_velocity, desire_acceleration);
        }

        if (circular_parameter_.circular_mode == CircularType::UNCONSTRAINT_RPY) {
            for (int i = 0; i < 3; i++) {
                if (circular_start_time <= t && t < circular_start_time + circular_parameter_.path_time[i + 1][1]) {
                    double t0 = circular_start_time;
                    desire_position[i + 3] = 0.0;
                    desire_velocity[i + 3] = 0.0;
                    desire_acceleration[i + 3] = 0.0;

                    for (int k = 0; k < degree; k++) {
                        desire_position[i + 3] += circular_parameter_.coefficient[0][i + 1][k] * pow(t - t0, k);
                    }
                    for (int k = 1; k < degree; k++) {
                        desire_velocity[i + 3] += k * circular_parameter_.coefficient[0][i + 1][k] * pow(t - t0, k - 1);
                    }
                    for (int k = 2; k < degree; k++) {
                        desire_acceleration[i + 3] += k * (k - 1) * circular_parameter_.coefficient[0][i + 1][k] * pow(t - t0, k - 2);
                    }
                }
                else if (circular_start_time + circular_parameter_.path_time[i + 1][1] <= t && t < circular_start_time + circular_parameter_.path_time[i + 1][2]) {
                    double t1 = circular_start_time + circular_parameter_.path_time[i + 1][1];
                    desire_position[i + 3] = 0.0;
                    desire_velocity[i + 3] = 0.0;
                    desire_acceleration[i + 3] = 0.0;

                    for (int k = 0; k < degree; k++) {
                        desire_position[i + 3] += circular_parameter_.coefficient[1][i + 1][k] * pow(t - t1, k);
                    }
                    for (int k = 1; k < degree; k++) {
                        desire_velocity[i + 3] += k * circular_parameter_.coefficient[1][i + 1][k] * pow(t - t1, k - 1);
                    }
                    for (int k = 2; k < degree; k++) {
                        desire_acceleration[i + 3] += k * (k - 1) * circular_parameter_.coefficient[1][i + 1][k] * pow(t - t1, k - 2);
                    }
                }
                else if (circular_start_time + circular_parameter_.path_time[i + 1][2] <= t && t < circular_start_time + circular_parameter_.path_time[i + 1][3]) {
                    double t2 = circular_start_time + circular_parameter_.path_time[i + 1][2];
                    desire_position[i + 3] = 0.0;
                    desire_velocity[i + 3] = 0.0;
                    desire_acceleration[i + 3] = 0.0;

                    for (int k = 0; k < degree; k++) {
                        desire_position[i + 3] += circular_parameter_.coefficient[2][i + 1][k] * pow(t - t2, k);
                    }
                    for (int k = 1; k < degree; k++) {
                        desire_velocity[i + 3] += k * circular_parameter_.coefficient[2][i + 1][k] * pow(t - t2, k - 1);
                    }
                    for (int k = 2; k < degree; k++) {
                        desire_acceleration[i + 3] += k * (k - 1) * circular_parameter_.coefficient[2][i + 1][k] * pow(t - t2, k - 2);
                    }
                }
            }
        }
    }
    else if (t >= finish_.time) {
        desire_position = finish_.position;
        for (unsigned int i = 0; i < dof_; i++) {
            desire_velocity[i] = 0.0;
            desire_acceleration[i] = 0.0;
        }
        genNoPath(desire_position);
    }

    desired_.position = desire_position;
    desired_.velocity = desire_velocity;
    desired_.acceleration = desire_acceleration;
}

void PathPlanner::calBSplineConstVelocity(const double t) {
    double time = t - init_.time;

    vector<vector<double>> basis_total;
    vector<double> basis_p_degree;
    vector<double> basis_p_1_degree;
    vector<double> basis_p_2_degree;
    vector<double> basis_p_3_degree;
    vector<double> path;
    vector<double> path_first_derivative;
    vector<double> path_second_derivative;
    vector<double> path_third_derivative;

    double time_law_first_derivative = 0.0;
    double time_law_second_derivative = 0.0;

    calBSplineBasisFunction(bSpline_parameter_.time, basis_p_degree, basis_p_1_degree, basis_p_2_degree, basis_p_3_degree);
    calBSplineInterpolepath(basis_p_degree, basis_p_1_degree, basis_p_2_degree, basis_p_3_degree, path, path_first_derivative, path_second_derivative, path_third_derivative);
    calConstVelocityTimeLaw(time, path_first_derivative, path_second_derivative, path_third_derivative, time_law_first_derivative, time_law_second_derivative);
    calBSplineInterpoleTrajectory(time, time_law_first_derivative, time_law_second_derivative, path, path_first_derivative, path_second_derivative);
}

void PathPlanner::calBlendingPath(const double t) {
    int degree = degree_ + 1;
    double linear_position = 0.0;
    double linear_velocity = 0.0;
    double linear_acceleration = 0.0;
    vector<double> rpy_position(dof_ - 3);
    vector<double> rpy_velocity(dof_ - 3);
    vector<double> rpy_acceleration(dof_ - 3);

    if (t < init_.time) {
        for (unsigned int i = 0; i < dof_; i++) {
            desired_.position[i] = init_.position[i];
            desired_.velocity[i] = init_.velocity[i];
            desired_.acceleration[i] = 0.0;
        }
    }
    else if (init_.time <= t && t < finish_.time) {
        for (unsigned int i = 0; i < blending_parameter_.path_time.size(); i++) {
            if (init_.time + blending_parameter_.path_time[i][0][0] <= t && t < init_.time + blending_parameter_.path_time[i][0][3]) {
                blending_parameter_.path_order = i;
            }
        }

        if (init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][0] <= t && t < init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][1]) {
            double t0 = init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][0];
            for (int k = 0; k < degree; k++) {
                linear_position += blending_parameter_.coefficients[blending_parameter_.path_order][0][0][k] * pow(t - t0, k);
            }
            for (int k = 1; k < degree; k++) {
                linear_velocity += k * blending_parameter_.coefficients[blending_parameter_.path_order][0][0][k] * pow(t - t0, k - 1);
            }
            for (int k = 2; k < degree; k++) {
                linear_acceleration += k * (k - 1) * blending_parameter_.coefficients[blending_parameter_.path_order][0][0][k] * pow(t - t0, k - 2);
            }
        }
        else if (init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][1] <= t && t < init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][2]) {
            double t1 = init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][1];

            for (int k = 0; k < degree; k++) {
                linear_position += blending_parameter_.coefficients[blending_parameter_.path_order][1][0][k] * pow(t - t1, k);
            }
            for (int k = 1; k < degree; k++) {
                linear_velocity += k * blending_parameter_.coefficients[blending_parameter_.path_order][1][0][k] * pow(t - t1, k - 1);
            }
            for (int k = 2; k < degree; k++) {
                linear_acceleration += k * (k - 1) * blending_parameter_.coefficients[blending_parameter_.path_order][1][0][k] * pow(t - t1, k - 2);
            }
        }
        else if (init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][2] <= t && t < init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][3]) {
            double t2 = init_.time + blending_parameter_.path_time[blending_parameter_.path_order][0][2];

            for (int k = 0; k < degree; k++) {
                linear_position += blending_parameter_.coefficients[blending_parameter_.path_order][2][0][k] * pow(t - t2, k);
            }
            for (int k = 1; k < degree; k++) {
                linear_velocity += k * blending_parameter_.coefficients[blending_parameter_.path_order][2][0][k] * pow(t - t2, k - 1);
            }
            for (int k = 2; k < degree; k++) {
                linear_acceleration += k * (k - 1) * blending_parameter_.coefficients[blending_parameter_.path_order][2][0][k] * pow(t - t2, k - 2);
            }
        }

        for (int i = 0; i < dof_ - 3; i++) {
            if (init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][0] <= t && t < init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][1]) {
                double t0 = init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][0];

                for (int k = 0; k < degree; k++) {
                    rpy_position[i] += blending_parameter_.coefficients[blending_parameter_.path_order][0][i + 1][k] * pow(t - t0, k);
                }
                for (int k = 1; k < degree; k++) {
                    rpy_velocity[i] += k * blending_parameter_.coefficients[blending_parameter_.path_order][0][i + 1][k] * pow(t - t0, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    rpy_acceleration[i] += k * (k - 1) * blending_parameter_.coefficients[blending_parameter_.path_order][0][i + 1][k] * pow(t - t0, k - 2);
                }
            }
            else if (init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][1] <= t && t < init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][2]) {
                double t1 = init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][1];

                for (int k = 0; k < degree; k++) {
                    rpy_position[i] += blending_parameter_.coefficients[blending_parameter_.path_order][1][i + 1][k] * pow(t - t1, k);
                }
                for (int k = 1; k < degree; k++) {
                    rpy_velocity[i] += k * blending_parameter_.coefficients[blending_parameter_.path_order][1][i + 1][k] * pow(t - t1, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    rpy_acceleration[i] += k * (k - 1) * blending_parameter_.coefficients[blending_parameter_.path_order][1][i + 1][k] * pow(t - t1, k - 2);
                }
            }
            else if (init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][2] <= t && t < init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][3]) {
                double t2 = init_.time + blending_parameter_.path_time[blending_parameter_.path_order][i + 1][2];

                for (int k = 0; k < degree; k++) {
                    rpy_position[i] += blending_parameter_.coefficients[blending_parameter_.path_order][2][i + 1][k] * pow(t - t2, k);
                }
                for (int k = 1; k < degree; k++) {
                    rpy_velocity[i] += k * blending_parameter_.coefficients[blending_parameter_.path_order][2][i + 1][k] * pow(t - t2, k - 1);
                }
                for (int k = 2; k < degree; k++) {
                    rpy_acceleration[i] += k * (k - 1) * blending_parameter_.coefficients[blending_parameter_.path_order][2][i + 1][k] * pow(t - t2, k - 2);
                }
            }
        }

        for (unsigned int i = 0; i < blending_parameter_.geometric_segment.size() - 1; i++) {
            if (blending_parameter_.geometric_segment[i].length <= linear_position && linear_position < blending_parameter_.geometric_segment[i + 1].length) {
                blending_parameter_.geometric_order = i;
            }
        }
        double linear_start = blending_parameter_.geometric_segment[blending_parameter_.geometric_order].length;
        if (blending_parameter_.geometric_segment[blending_parameter_.geometric_order + 1].type) { // ���� ����
            for (int i = 0; i < 3; i++) {
                desired_.position[i] = blending_parameter_.xyz_unit_distance[blending_parameter_.geometric_order / 2][i] * (linear_position - linear_start) + blending_parameter_.geometric_waypoints[blending_parameter_.geometric_order / 2][i];
                desired_.velocity[i] = blending_parameter_.xyz_unit_distance[blending_parameter_.geometric_order / 2][i] * linear_velocity;
                desired_.acceleration[i] = blending_parameter_.xyz_unit_distance[blending_parameter_.geometric_order / 2][i] * linear_acceleration;
            }
        }
        else { // ��ȣ ����
            vector<double> circle_desired_position(3);
            vector<double> circle_desired_velocity(3);
            vector<double> circle_desired_acceleration(3);

            double angle_position = divideWithFloatingError(linear_position - linear_start, blending_parameter_.radius[blending_parameter_.geometric_order / 2]);
            double angle_velocity = divideWithFloatingError(linear_velocity, blending_parameter_.radius[blending_parameter_.geometric_order / 2]);
            double angle_acceleration = divideWithFloatingError(linear_acceleration, blending_parameter_.radius[blending_parameter_.geometric_order / 2]);

            calThetatoPosition(angle_position, angle_velocity, angle_acceleration, circle_desired_position, circle_desired_velocity, circle_desired_acceleration);
            for (int i = 0; i < 3; i++) {
                desired_.position[i] = circle_desired_position[i];
                desired_.velocity[i] = circle_desired_velocity[i];
                desired_.acceleration[i] = circle_desired_acceleration[i];
            }
        }

        for (int i = 0; i < dof_ - 3; i++) {
            desired_.position[i + 3] = rpy_position[i];
            desired_.velocity[i + 3] = rpy_velocity[i];
            desired_.acceleration[i + 3] = rpy_acceleration[i];
        }
    }
    else {
        desired_.position = finish_.position;
        for (unsigned int i = 0; i < dof_; i++) {
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
        }
        genNoPath(current_.position);
    }
}

void PathPlanner::calSinePath(const double t) {
    double time = t - init_.time;
    double freq = sqrt(45 / sine_parameter_.amplitude);
    if (sine_parameter_.operation_time > time && time > 0) {
        for (int i = 0; i < 1; i++) {
            desired_.position[i] = sine_parameter_.amplitude * cos(target_.velocity[i] / sine_parameter_.amplitude * time) - sine_parameter_.amplitude + init_.position[i];
            desired_.velocity[i] = -target_.velocity[i] * sin(target_.velocity[i] / sine_parameter_.amplitude * time);
            desired_.acceleration[i] = -target_.velocity[i] * target_.velocity[i] / sine_parameter_.amplitude * cos(target_.velocity[i] / sine_parameter_.amplitude * time);
        }
    }
    else {
        genStopPath(0.2);
    }
    // if (sine_parameter_.operation_time > time && time > 0) {
    //     for (int i = 0; i < JS_DOF; i++) {
    //         desired_.acceleration[i] = sine_parameter_.amplitude*cos(freq*time);
    //         desired_.velocity[i] = sine_parameter_.amplitude/freq*sin(freq*time);
    //         desired_.position[i] = -sine_parameter_.amplitude/freq/freq*cos(freq*time) + init_.position[i] + sine_parameter_.amplitude/freq/freq;
    //     }
    // } else {
    //     genStopPath(0.2);
    // }
}

void PathPlanner::calBackwardPath(const double t) {
    double time_temp = t - init_.time;
    for (unsigned int i = 0; i < dof_; i++) {
        desired_.position[i] = 0.0;
        desired_.velocity[i] = 0.0;
        desired_.acceleration[i] = 0.0;

        if (t < init_.time) {
             desired_.position[i] = init_.position[i];
             desired_.velocity[i] = init_.velocity[i];
             desired_.acceleration[i] = 0.0;
        } else if (init_.time <= t && t < finish_.time) {
            // ongoing direction deceleration
            if (time_temp < backward_parameter_.unit_time) {
                for (int k = 0; k < 6; k++) {
                    desired_.position[i] += backward_parameter_.coefficients[0][i][k] * pow(time_temp, k);
                }
                for (int k = 1; k < 6; k++) {
                    desired_.velocity[i] += k * backward_parameter_.coefficients[0][i][k] * pow(time_temp, k - 1);
                }
                for (int k = 2; k < 6; k++) {
                    desired_.acceleration[i] += k * (k - 1) * backward_parameter_.coefficients[0][i][k] * pow(time_temp, k - 2);
                }
            }
            // return
            else if (backward_parameter_.unit_time <= time_temp && time_temp < backward_parameter_.unit_time * 2) {
                for (int k = 0; k < 6; k++) {
                    desired_.position[i] += backward_parameter_.coefficients[1][i][k] * pow(time_temp, k);
                }
                for (int k = 1; k < 6; k++) {
                    desired_.velocity[i] += k * backward_parameter_.coefficients[1][i][k] * pow(time_temp, k - 1);
                }
                for (int k = 2; k < 6; k++) {
                    desired_.acceleration[i] += k * (k - 1) * backward_parameter_.coefficients[1][i][k] * pow(time_temp, k - 2);
                }
            }
            // opposite direction deceleration
            else if (backward_parameter_.unit_time * 2 <= time_temp && time_temp < backward_parameter_.unit_time * 3) {
                for (int k = 0; k < 6; k++) {
                    desired_.position[i] += backward_parameter_.coefficients[2][i][k] * pow(time_temp, k);
                }
                for (int k = 1; k < 6; k++) {
                    desired_.velocity[i] += k * backward_parameter_.coefficients[2][i][k] * pow(time_temp, k - 1);
                }
                for (int k = 2; k < 6; k++) {
                    desired_.acceleration[i] += k * (k - 1) * backward_parameter_.coefficients[2][i][k] * pow(time_temp, k - 2);
                }
            }
        } else {
            desired_.position[i] = finish_.position[i];
            desired_.velocity[i] = 0.0;
            desired_.acceleration[i] = 0.0;
            genNoPath(desired_.position);
        }
    }
}

void PathPlanner::calAccelerationPath(const double delta_t, const vector<vector<double>> coeff, vector<double>& desire_position, vector<double>& desire_velocity, vector<double>& desire_accleration) {
    int vector_size = coeff.size();
    int degree = degree_ + 1;

    desire_position.resize(vector_size);
    desire_velocity.resize(vector_size);
    desire_accleration.resize(vector_size);

    for (int i = 0; i < vector_size; i++) {
        for (int k = 0; k < degree; k++) {
            desire_position[i] += coeff[i][k] * pow(delta_t, k);
        }
        for (int k = 1; k < degree; k++) {
            desire_velocity[i] += k * coeff[i][k] * pow(delta_t, k - 1);
        }
        for (int k = 2; k < degree; k++) {
            desire_accleration[i] += k * (k - 1) * coeff[i][k] * pow(delta_t, k - 2);
        }
    }
}

void PathPlanner::calCruisePath(const double delta_t, const vector<vector<double>> coeff, vector<double>& desire_position, vector<double>& desire_velocity, vector<double>& desire_accleration) {
    int vector_size = coeff.size();
    desire_position.resize(vector_size);
    desire_velocity.resize(vector_size);
    desire_accleration.resize(vector_size);

    for (int i = 0; i < vector_size; i++) {
        desire_position[i] = coeff[i][0] + delta_t * coeff[i][1];
        desire_velocity[i] = coeff[i][1];
        desire_accleration[i] = coeff[i][2];
    }
}

void PathPlanner::calThetatoPose(const double desire_angle_position, const double desire_angle_velocity, const double desire_angle_accleration, vector<double>& desire_position, vector<double>& desire_velocity, vector<double>& desire_accleration) {
    vector<double> desire_centripetal_acceleration(dof_);
    vector<double> desire_tangential_acceleration(dof_);
    double desire_radian_position = desire_angle_position * kDeg2Rad;
    double desire_radian_velocity = desire_angle_velocity * kDeg2Rad;
    double desire_radian_accleration = desire_angle_accleration * kDeg2Rad;

    for (int i = 0; i < 3; i++) {
        desire_position[i] = circular_parameter_.radius * (cos(desire_radian_position) * circular_parameter_.vector_b1[i] + sin(desire_radian_position) * circular_parameter_.vector_b0[i]) + circular_parameter_.center_point[i]; // desired ��ġ - �� (4.2.11) ����
        desire_velocity[i] = circular_parameter_.radius * desire_radian_velocity * (-1 * sin(desire_radian_position) * circular_parameter_.vector_b1[i] + cos(desire_radian_position) * circular_parameter_.vector_b0[i]); // desired �ӵ� - �� (4.2.12) ����
        desire_centripetal_acceleration[i] = circular_parameter_.radius * pow(desire_radian_velocity, 2) * (-1 * cos(desire_radian_position) * circular_parameter_.vector_b1[i] - sin(desire_radian_position) * circular_parameter_.vector_b0[i]); // desired ���ɰ��ӵ� - �� (4.2.13) ����
        desire_tangential_acceleration[i] = circular_parameter_.radius * desire_radian_accleration * (-1 * sin(desire_radian_position) * circular_parameter_.vector_b1[i] + cos(desire_radian_position) * circular_parameter_.vector_b0[i]); // desired �������ӵ� - �� (4.2.13)����
        desire_accleration[i] = desire_centripetal_acceleration[i] + desire_tangential_acceleration[i]; // desired ���ӵ� - �� (4.2.13) ����
    }

    if (circular_parameter_.circular_mode == CircularType::TANGENT_RPY) {
        ModelMatrix rotation_matrix(3, 3);
        ModelMatrix target_rotation_matrix(3, 3);

        rotation_matrix = convertNormalVectortoRotationMatrix(desire_angle_position, circular_parameter_.vector_n);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                target_rotation_matrix.set(i, j, rotation_matrix.get(i, 0) * circular_parameter_.init_rotation_matrix.get(0, j) + rotation_matrix.get(i, 1) * circular_parameter_.init_rotation_matrix.get(1, j) + rotation_matrix.get(i, 2) * circular_parameter_.init_rotation_matrix.get(2, j));
            }
        }
        desire_position[5] = atan2(target_rotation_matrix.get(1, 0), target_rotation_matrix.get(0, 0)) * kRad2Deg;
        double temp = sqrt(target_rotation_matrix.get(2, 1) * target_rotation_matrix.get(2, 1) + target_rotation_matrix.get(2, 2) * target_rotation_matrix.get(2, 2));
        desire_position[4] = atan2(-target_rotation_matrix.get(2, 0), temp) * kRad2Deg;
        desire_position[3] = atan2(target_rotation_matrix.get(2, 1), target_rotation_matrix.get(2, 2)) * kRad2Deg;

        vector<double> desire_position_previous(dof_);
        vector<double> desire_velocity_previous(dof_);
        desire_position_previous = desired_.position;
        desire_velocity_previous = desired_.velocity;
        vector<double> desire_velocity_temp(3);
        for (int i = 0; i < 3; i++) {
            desire_velocity_temp[i] = desire_position[i + 3] - desire_position_previous[i + 3];
            if (desire_velocity_temp[i] > 180.0) {
                desire_velocity_temp[i] = (desire_velocity_temp[i] - 360.0);
            }
            else if (desire_velocity_temp[i] < -180.0) {
                desire_velocity_temp[i] = (desire_velocity_temp[i] + 360.0);
            }
            desire_velocity[i + 3] = desire_velocity_temp[i] * 1000.0;
            desire_accleration[i + 3] = (desire_velocity[i + 3] - desire_velocity_previous[i + 3]) * 1000.0;
        }
    }
}

void PathPlanner::calBSplineInterpole(double t) {
    double time = t;

    vector<vector<double>> basis_total;
    vector<double> basis_p_degree;
    vector<double> basis_p_1_degree;
    vector<double> basis_p_2_degree;
    vector<double> basis_p_3_degree;
    vector<double> path;
    vector<double> path_first_derivative;
    vector<double> path_second_derivative;
    vector<double> path_third_derivative;

    double time_law = 0.0;
    double time_law_first_derivative = 0.0;
    double time_law_second_derivative = 0.0;

    calBSplineBasisFunction(time, basis_p_degree, basis_p_1_degree, basis_p_2_degree, basis_p_3_degree);
    calBSplineInterpolepath(basis_p_degree, basis_p_1_degree, basis_p_2_degree, basis_p_3_degree, path, path_first_derivative, path_second_derivative, path_third_derivative);
    calTimeLaw(time, time_law, time_law_first_derivative, time_law_second_derivative);
    calBSplineInterpoleTrajectory(time_law, time_law_first_derivative, time_law_second_derivative, path, path_first_derivative, path_second_derivative);
}

void PathPlanner::calBSplineBasisFunction(const double time, vector<double>& basis_p_degree, vector<double>& basis_p_1_degree, vector<double>& basis_p_2_degree, vector<double>& basis_p_3_degree) {
    basis_p_degree.resize(bSpline_parameter_.controlpoint_num);
    basis_p_1_degree.resize(bSpline_parameter_.controlpoint_num - 1);
    basis_p_2_degree.resize(bSpline_parameter_.controlpoint_num - 2);
    basis_p_3_degree.resize(bSpline_parameter_.controlpoint_num - 3);

    for (int i = 0; i < 4; i++) {
        vector<vector<double>> basis_total;
        basis_total.resize(bSpline_parameter_.p_degree - i);
        for (int j = 0; j < bSpline_parameter_.p_degree - i; j++) {
            basis_total[j].resize(bSpline_parameter_.controlpoint_num - i);
        }

        double DL = 0.0;
        double DR = 0.0;
        double dn1 = 0.0;
        double dd1 = 0.0;
        double dn2 = 0.0;
        double dd2 = 0.0;

        // for every degree, for p = 0;
        if (time < bSpline_parameter_.operation_time) {
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - i; j++) {
                if (bSpline_parameter_.knot_vector[j + i] <= time && time < bSpline_parameter_.knot_vector[j + i + 1]) {
                    basis_total[0][j] = 1.0;
                }
                else {
                    basis_total[0][j] = 0.0;
                }
            }
        }
        else { //time == operation time)
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - i; j++) {
                if (j != bSpline_parameter_.controlpoint_num - i - 1) {
                    if (bSpline_parameter_.knot_vector[j + i] <= time && time < bSpline_parameter_.knot_vector[j + i + 1]) {
                        basis_total[0][j] = 1.0;
                    }
                    else {
                        basis_total[0][j] = 0.0;
                    }
                }
                else {
                    if (bSpline_parameter_.knot_vector[j] <= time) {
                        basis_total[0][j] = 1.0;
                    }
                    else {
                        basis_total[0][j] = 0.0;
                    }
                }
            }
        }

        // for every degree, for p > 0;
        for (int p = 1; p < bSpline_parameter_.p_degree - i; p++) {
            for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - i; j++) {
                if (j != bSpline_parameter_.controlpoint_num - i - 1) {
                    dn1 = time - bSpline_parameter_.knot_vector[j + i];
                    dd1 = bSpline_parameter_.knot_vector[j + i + p] - bSpline_parameter_.knot_vector[j + i];
                    DL = divideWithFloatingError(dn1, dd1);

                    dn2 = bSpline_parameter_.knot_vector[j + i + p + 1] - time;
                    dd2 = bSpline_parameter_.knot_vector[j + i + p + 1] - bSpline_parameter_.knot_vector[j + i + 1];
                    DR = divideWithFloatingError(dn2, dd2);

                    basis_total[p][j] = DL * basis_total[p - 1][j] + DR * basis_total[p - 1][j + 1];
                    if (i == 0) {
                        basis_p_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                    else if (i == 1) {
                        basis_p_1_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                    else if (i == 2) {
                        basis_p_2_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                    else {
                        basis_p_3_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                }
                else {
                    dn1 = time - bSpline_parameter_.knot_vector[j + i];
                    dd1 = bSpline_parameter_.knot_vector[j + i + p] - bSpline_parameter_.knot_vector[j + i];
                    DL = divideWithFloatingError(dn1, dd1);

                    dn2 = bSpline_parameter_.knot_vector[j + i + p + 1] - time;
                    dd2 = bSpline_parameter_.knot_vector[j + i + p + 1] - bSpline_parameter_.knot_vector[j + i + 1];
                    DR = divideWithFloatingError(dn2, dd2);

                    basis_total[p][j] = DL * basis_total[p - 1][j];
                    if (i == 0) {
                        basis_p_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                    else if (i == 1) {
                        basis_p_1_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                    else if (i == 2) {
                        basis_p_2_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                    else {
                        basis_p_3_degree[j] = basis_total[bSpline_parameter_.p_degree - i - 1][j];
                    }
                }
            }
        }
    }
}

void PathPlanner::calTimeLaw(const double time, double& time_law, double& time_law_first_derivative, double& time_law_second_derivative) {
    time_law = time;
    time_law_first_derivative = 1.0;
    time_law_second_derivative = 0.0;
}

void PathPlanner::calConstVelocityTimeLaw(const double time, const vector<double> first_derivative_path, const vector<double> second_derivative_path, const vector<double> third_derivative_path, double& time_first_derivative, double& time_second_derivative) {
    double acceleration_end_time = bSpline_parameter_.path_time[0][1];
    double decceleration_start_time = bSpline_parameter_.path_time[0][2];
    double line_velocity = 0.0;
    double line_acceleration = 0.0;
    double line_jerk = 0.0;
    double period = bSpline_parameter_.time_period;
    int degree = 7;
    double dpdu_norm = sqrt(pow(first_derivative_path[0], 2.0) + pow(first_derivative_path[1], 2.0) + pow(first_derivative_path[2], 2.0));
    double dpdu_dot_d2pdu2 = first_derivative_path[0] * second_derivative_path[0] + first_derivative_path[1] * second_derivative_path[1] + first_derivative_path[2] * second_derivative_path[2];
    double dpdu_dot_d3pdu3 = first_derivative_path[0] * third_derivative_path[0] + first_derivative_path[1] * third_derivative_path[1] + first_derivative_path[2] * third_derivative_path[2];

    if (time < 0.0000000000001) {
        double t0 = bSpline_parameter_.path_time[0][0];
        for (int k = 1; k < degree + 1; k++) {
            line_velocity += k * bSpline_parameter_.coefficient[0][0][k] * pow(time - t0, k - 1);
        }
        for (int k = 2; k < degree + 1; k++) {
            line_acceleration += k * (k - 1) * bSpline_parameter_.coefficient[0][0][k] * pow(time - t0, k - 2);
        }
        for (int k = 3; k < degree + 1; k++) {
            line_jerk += k * (k - 1) * (k - 2) * bSpline_parameter_.coefficient[0][0][k] * pow(time - t0, k - 3);
        }

        double dudt = divideWithFloatingError(line_velocity, dpdu_norm);
        double d2udt2 = divideWithFloatingError(line_acceleration, dpdu_norm) - divideWithFloatingError(line_velocity * line_velocity * dpdu_dot_d2pdu2, pow(dpdu_norm, 4.0));
        double d3udt3 = divideWithFloatingError(line_jerk, dpdu_norm) - divideWithFloatingError(dpdu_dot_d3pdu3 * pow(line_velocity, 3.0), dpdu_norm) - divideWithFloatingError(3 * dpdu_dot_d2pdu2 * d2udt2 * dudt, dpdu_norm);                 //divideWithFloatingError(line_jerk, dpdu_norm) - divideWithFloatingError(dpdu_dot_d3pdu3 * pow(line_velocity, 3.0), pow(dpdu_norm, 5.0)) - divideWithFloatingError(3 * dpdu_dot_d2pdu2 * line_velocity * d2udt2, pow(dpdu_norm, 5.0));
        bSpline_parameter_.time_first_derivative = dudt + period / 2 * d2udt2 + pow(period, 2.0) / 6 * d3udt3;
        bSpline_parameter_.time_second_derivative = 0.0;

        bSpline_parameter_.time += bSpline_parameter_.time_first_derivative * period;
    }
    else if (time < bSpline_parameter_.operation_time) {
        if (time <= acceleration_end_time) {
            double t0 = bSpline_parameter_.path_time[0][0];
            for (int k = 1; k < degree + 1; k++) {
                line_velocity += k * bSpline_parameter_.coefficient[0][0][k] * pow(time - t0, k - 1);
            }
            for (int k = 2; k < degree + 1; k++) {
                line_acceleration += k * (k - 1) * bSpline_parameter_.coefficient[0][0][k] * pow(time - t0, k - 2);
            }
            for (int k = 3; k < degree + 1; k++) {
                line_jerk += k * (k - 1) * (k - 2) * bSpline_parameter_.coefficient[0][0][k] * pow(time - t0, k - 3);
            }
        }
        else if (time > acceleration_end_time && time <= decceleration_start_time) {
            double t1 = bSpline_parameter_.path_time[0][1];

            for (int k = 1; k < degree + 1; k++) {
                line_velocity += k * bSpline_parameter_.coefficient[1][0][k] * pow(time - t1, k - 1);
            }
            for (int k = 2; k < degree + 1; k++) {
                line_acceleration += k * (k - 1) * bSpline_parameter_.coefficient[1][0][k] * pow(time - t1, k - 2);
            }
            for (int k = 3; k < degree + 1; k++) {
                line_jerk += k * (k - 1) * (k - 2) * bSpline_parameter_.coefficient[1][0][k] * pow(time - t1, k - 3);
            }
        }
        else {
            double t2 = bSpline_parameter_.path_time[0][2];

            for (int k = 1; k < degree + 1; k++) {
                line_velocity += k * bSpline_parameter_.coefficient[2][0][k] * pow(time - t2, k - 1);
            }
            for (int k = 2; k < degree + 1; k++) {
                line_acceleration += k * (k - 1) * bSpline_parameter_.coefficient[2][0][k] * pow(time - t2, k - 2);
            }
            for (int k = 3; k < degree + 1; k++) {
                line_jerk += k * (k - 1) * (k - 2) * bSpline_parameter_.coefficient[2][0][k] * pow(time - t2, k - 3);
            }
        }
        double dudt = divideWithFloatingError(line_velocity, dpdu_norm);
        double d2udt2 = divideWithFloatingError(line_acceleration, dpdu_norm) - divideWithFloatingError(line_velocity * line_velocity * dpdu_dot_d2pdu2, pow(dpdu_norm, 4.0));
        double d3udt3 = divideWithFloatingError(line_jerk, dpdu_norm) - divideWithFloatingError(dpdu_dot_d3pdu3 * pow(line_velocity, 3.0), dpdu_norm) - divideWithFloatingError(3 * dpdu_dot_d2pdu2 * d2udt2 * dudt, dpdu_norm);

        bSpline_parameter_.time_first_derivative_previous = bSpline_parameter_.time_first_derivative;
        bSpline_parameter_.time_first_derivative = dudt + period / 2 * d2udt2 + pow(period, 2.0) / 6 * d3udt3;

        bSpline_parameter_.time_second_derivative = divideWithFloatingError(bSpline_parameter_.time_first_derivative - bSpline_parameter_.time_first_derivative_previous, period);
        bSpline_parameter_.time += bSpline_parameter_.time_first_derivative * period;
    }
    else {
        double t2 = bSpline_parameter_.path_time[0][2];

        for (int k = 1; k < degree + 1; k++) {
            line_velocity += k * bSpline_parameter_.coefficient[2][0][k] * pow(time - t2, k - 1);
        }
        for (int k = 2; k < degree + 1; k++) {
            line_acceleration += k * (k - 1) * bSpline_parameter_.coefficient[2][0][k] * pow(time - t2, k - 2);
        }
        for (int k = 3; k < degree + 1; k++) {
            line_jerk += k * (k - 1) * (k - 2) * bSpline_parameter_.coefficient[2][0][k] * pow(time - t2, k - 3);
        }

        double dudt = divideWithFloatingError(line_velocity, dpdu_norm);
        double d2udt2 = divideWithFloatingError(line_acceleration, dpdu_norm) - divideWithFloatingError(line_velocity * line_velocity * dpdu_dot_d2pdu2, pow(dpdu_norm, 4.0));
        double d3udt3 = divideWithFloatingError(line_jerk, dpdu_norm) - divideWithFloatingError(dpdu_dot_d3pdu3 * pow(line_velocity, 3.0), dpdu_norm) - divideWithFloatingError(3 * dpdu_dot_d2pdu2 * d2udt2 * dudt, dpdu_norm);
        bSpline_parameter_.time_first_derivative_previous = bSpline_parameter_.time_first_derivative;
        bSpline_parameter_.time_first_derivative = dudt + period / 2 * d2udt2 + pow(period, 2.0) / 6 * d3udt3;

        bSpline_parameter_.time_second_derivative = divideWithFloatingError(bSpline_parameter_.time_first_derivative - bSpline_parameter_.time_first_derivative_previous, period);
        bSpline_parameter_.time += bSpline_parameter_.time_first_derivative * period;
    }

    time_first_derivative = bSpline_parameter_.time_first_derivative;
    time_second_derivative = bSpline_parameter_.time_second_derivative;
}

void PathPlanner::calBSplineInterpolepath(const vector<double> basis, const vector<double> basis_first_derivative, const vector<double> basis_second_derivative, const vector<double> basis_third_derivative, vector<double>& path, vector<double>& first_derivative_path, vector<double>& second_derivative_path, vector<double>& third_derivative_path) {
    path.resize(dof_);
    first_derivative_path.resize(dof_);
    second_derivative_path.resize(dof_);
    third_derivative_path.resize(dof_);

    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num; j++) {
            path[i] += bSpline_parameter_.controlpoints[j][i] * basis[j];
        }
    }

    // dS/du
    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - 1; j++) {
            first_derivative_path[i] += bSpline_parameter_.controlpoints_first_derivative[j][i] * basis_first_derivative[j];
        }
    }

    // d^2S/du^2
    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - 2; j++) {
            second_derivative_path[i] += bSpline_parameter_.controlpoints_second_derivative[j][i] * basis_second_derivative[j];
        }
    }

    // d^3S/du^3
    for (unsigned int i = 0; i < dof_; i++) {
        for (unsigned int j = 0; j < bSpline_parameter_.controlpoint_num - 3; j++) {
            third_derivative_path[i] += bSpline_parameter_.controlpoints_third_derivative[j][i] * basis_third_derivative[j];
        }
    }
}

void PathPlanner::calBSplineInterpoleTrajectory(const double time, const double time_law_first_derivative, const double time_law_second_derivative, const vector<double> path, const vector<double> first_derivative_path, const vector<double> second_derivative_path) {
    // �ʱ�ȭ
    vector<double> temp_position(dof_);
    vector<double> temp_velocity(dof_);
    vector<double> temp_acceleration(dof_);

    if (time < 0.0000000001) {
        for (unsigned int i = 0; i < dof_; i++) {
            // desired position
            temp_position[i] = init_.position[i];
            // desired velocity
            temp_velocity[i] = first_derivative_path[i] * time_law_first_derivative;
            // desired acceleration
            temp_acceleration[i] = first_derivative_path[i] * time_law_second_derivative + second_derivative_path[i] * time_law_first_derivative;
        }
    }
    else if (time < bSpline_parameter_.operation_time) {
        for (unsigned int i = 0; i < dof_; i++) {
            // desired position
            temp_position[i] = path[i];
            // desired velocity
            temp_velocity[i] = first_derivative_path[i] * time_law_first_derivative;
            // Desired acc
            temp_acceleration[i] = first_derivative_path[i] * time_law_second_derivative + second_derivative_path[i] * time_law_first_derivative;
        }
    }
    else {
        for (unsigned int i = 0; i < dof_; i++) {
            // desired position
            temp_position[i] = finish_.position[i];
            // desired velocity
            temp_velocity[i] = 0.0;
            // desired acceleration
            temp_acceleration[i] = 0.0;
        }
        genNoPath(current_.position);
    }

    desired_.position = temp_position;
    desired_.velocity = temp_velocity;
    desired_.acceleration = temp_acceleration;

    bSpline_parameter_.length += sqrt(pow(temp_velocity[0] * bSpline_parameter_.time_period, 2.0) + pow(temp_velocity[1] * bSpline_parameter_.time_period, 2.0) + pow(temp_velocity[2] * bSpline_parameter_.time_period, 2.0));
}

void PathPlanner::calThetatoPosition(const double desire_angle_position, const double desire_angle_velocity, const double desire_angle_accleration, vector<double>& desire_position, vector<double>& desire_velocity, vector<double>& desire_accleration) {
    vector<double> desire_centripetal_acceleration(dof_);
    vector<double> desire_tangential_acceleration(dof_);
    double desire_radian_position = desire_angle_position;
    double desire_radian_velocity = desire_angle_velocity;
    double desire_radian_accleration = desire_angle_accleration;
    int circle_order = blending_parameter_.geometric_order / 2;

    for (int i = 0; i < 3; i++) {
        desire_position[i] = blending_parameter_.radius[circle_order] * (cos(desire_radian_position) * blending_parameter_.vector_b1[circle_order][i] + sin(desire_radian_position) * blending_parameter_.vector_b0[circle_order][i]) + blending_parameter_.circle_center_points[circle_order][i]; // desired ��ġ - �� (4.2.11) ����
        desire_velocity[i] = blending_parameter_.radius[circle_order] * desire_radian_velocity * (-1 * sin(desire_radian_position) * blending_parameter_.vector_b1[circle_order][i] + cos(desire_radian_position) * blending_parameter_.vector_b0[circle_order][i]); // desired �ӵ� - �� (4.2.12) ����
        desire_centripetal_acceleration[i] = blending_parameter_.radius[circle_order] * pow(desire_radian_velocity, 2) * (-1 * cos(desire_radian_position) * blending_parameter_.vector_b1[circle_order][i] - sin(desire_radian_position) * blending_parameter_.vector_b0[circle_order][i]); // desired ���ɰ��ӵ� - �� (4.2.13) ����
        desire_tangential_acceleration[i] = blending_parameter_.radius[circle_order] * desire_radian_accleration * (-1 * sin(desire_radian_position) * blending_parameter_.vector_b1[circle_order][i] + cos(desire_radian_position) * blending_parameter_.vector_b0[circle_order][i]); // desired �������ӵ� - �� (4.2.13)����
        desire_accleration[i] = desire_centripetal_acceleration[i] + desire_tangential_acceleration[i]; // desired ���ӵ� - �� (4.2.13) ����
    }
}

vector<double> PathPlanner::calMaxVelocity(const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> distance, const vector<double> acceleration) {
    int vector_size = 0;
    vector_size = init_velocity.size();
    vector<double> displacement(vector_size);
    vector<double> max_velocity(vector_size);
    for (int i = 0; i < vector_size; i++) {
        if (init_velocity[i] >= 0.0 && finish_velocity[i] >= 0.0) {
            displacement[i] = (init_velocity[i] + finish_velocity[i]) * fabs(init_velocity[i] - finish_velocity[i]) / (2 * acceleration[i]);

            if (displacement[i] <= distance[i]) {
                max_velocity[i] = sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 + distance[i] * acceleration[i]);
            }
            else {
                max_velocity[i] = -sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 - distance[i] * acceleration[i]);
            }
        }
        else if (init_velocity[i] > 0.0 && finish_velocity[i] < 0.0) {
            displacement[i] = pow(init_velocity[i], 2.0) - pow(finish_velocity[i], 2.0) / (2.0 * acceleration[i]);

            if (displacement[i] <= distance[i]) {
                max_velocity[i] = sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 + distance[i] * acceleration[i]);
            }
            else {
                max_velocity[i] = -sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 - distance[i] * acceleration[i]);
            }
        }
        else if (init_velocity[i] < 0.0 && finish_velocity[i] > 0.0) {
            displacement[i] = pow(finish_velocity[i], 2.0) - pow(init_velocity[i], 2.0) / (2.0 * acceleration[i]);

            if (displacement[i] <= distance[i]) {
                max_velocity[i] = sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 + distance[i] * acceleration[i]);
            }
            else {
                max_velocity[i] = -sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 - distance[i] * acceleration[i]);
            }
        }
        else {
            displacement[i] = (init_velocity[i] + finish_velocity[i]) * fabs(init_velocity[i] - finish_velocity[i]) / (2.0 * acceleration[i]);

            if (displacement[i] <= distance[i]) {
                max_velocity[i] = sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 + distance[i] * acceleration[i]);
            }
            else {
                max_velocity[i] = -sqrt((pow(init_velocity[i], 2.0) + pow(finish_velocity[i], 2.0)) / 2.0 - distance[i] * acceleration[i]);
            }
        }
    }
    return max_velocity;
}

double PathPlanner::compareTimeSum(const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> uniform_velocity,
                                   const vector<double> distance, const vector<double> acceleration) {
    int vector_size = init_velocity.size();
    double path_time = 0.0;
    double time_temp = 0.0;
    for (int i = 0; i < vector_size; i++) {
        if (!isEqual(acceleration[i], 0.0) && !isEqual(uniform_velocity[i], 0.0)) {
            time_temp = fabs(uniform_velocity[i] - init_velocity[i]) / acceleration[i]
                + (distance[i] - (uniform_velocity[i] + init_velocity[i]) * fabs(uniform_velocity[i] - init_velocity[i]) / (2.0 * acceleration[i])
                    - (uniform_velocity[i] + finish_velocity[i]) * fabs(uniform_velocity[i] - finish_velocity[i]) / (2.0 * acceleration[i])) / uniform_velocity[i]
                + fabs(uniform_velocity[i] - finish_velocity[i]) / acceleration[i];
        }
        else {
            time_temp = 0.0;
        }

        if (time_temp > path_time) {
            path_time = time_temp;
        }
    }
    return path_time;
}

vector<double> PathPlanner::compareTime(const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> uniform_velocity,
                                             const vector<double> distance, const vector<double> acceleration) {
    int vector_size = 0;
    vector_size = init_velocity.size();
    vector<double> path_time(3);
    for (int i = 0; i < 3; i++) {
        path_time[i] = 0.0;
    }
    double time_temp = 0.0;
    for (int i = 0; i < vector_size; i++) {
        //가속 시간
        if (!isEqual(acceleration[i], 0.0)) {
            time_temp = fabs(uniform_velocity[i] - init_velocity[i]) / acceleration[i];
        } else {
            time_temp = 0.0;
        }
        if (time_temp > path_time[0]) {
            path_time[0] = time_temp;
        }

        //감속 시간
        if (!isEqual(acceleration[i], 0.0)) {
            time_temp = fabs(uniform_velocity[i] - finish_velocity[i]) / acceleration[i];
        } else {
            time_temp = 0.0;
        }
        if (time_temp > path_time[2]) {
            path_time[2] = time_temp;
        }

        //등속 시간
        if (!isEqual(acceleration[i], 0.0) && !isEqual(uniform_velocity[i], 0.0)) {
            time_temp = (distance[i] -  (uniform_velocity[i] + init_velocity[i]) * fabs(uniform_velocity[i] - init_velocity[i]) / (2.0 * acceleration[i]) - (uniform_velocity[i] + finish_velocity[i]) * fabs(uniform_velocity[i] - finish_velocity[i]) / (2.0 * acceleration[i])) / uniform_velocity[i];
        } else {
            time_temp = 0.0;
        }
        if (time_temp > path_time[1]) {
            path_time[1] = time_temp;
        }
    }
    return path_time;
}

vector<double> PathPlanner::recalUniformVelocity(const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> distance, const vector<double> path_time) {
    int vector_size = 0;
    vector_size = init_velocity.size();
    vector<double> uniform_velocity(vector_size);
    for (int i = 0; i < vector_size; i++) {
        uniform_velocity[i] = divideWithFloatingError((distance[i] - (init_velocity[i] * path_time[0] + finish_velocity[i] * path_time[2]) / 2.0), path_time[1] + (path_time[0] + path_time[2]) / 2.0);
    }
    return uniform_velocity;
}

vector<vector<vector<double>>> PathPlanner::calPathCoefficient(const vector<double> init_position, const vector<double> finish_position, const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> uniform_velocity, const vector<double> path_time) {
    vector<vector<vector<double>>> coefficient(3);
    int vector_size = init_position.size();
    int degree = degree_ + 1;

    coefficient[0].resize(vector_size);
    for (int i = 0 ; i < vector_size; i++) {
        coefficient[0][i].resize(degree);
        coefficient[0][i][0] = init_position[i];
        coefficient[0][i][1] = init_velocity[i];
        coefficient[0][i][2] = 0.0;
        if (degree_ == 5) {
            if (!isEqual(path_time[0], 0.0)) {
                coefficient[0][i][3] = (uniform_velocity[i] - init_velocity[i]) / pow(path_time[0], 2.0);
                coefficient[0][i][4] = (init_velocity[i] - uniform_velocity[i]) / (2.0 * pow(path_time[0], 3.0));
                coefficient[0][i][5] = 0.0;
            } else {
                coefficient[0][i][3] = 0.0;
                coefficient[0][i][4] = 0.0;
                coefficient[0][i][5] = 0.0;
            }
        } else if (degree_ == 7) {
            if (!isEqual(path_time[0], 0.0)) {
                coefficient[0][i][3] = 0.0;
                coefficient[0][i][4] = 5 * (uniform_velocity[i] - init_velocity[i]) / 2 / pow(path_time[0], 3.0);
                coefficient[0][i][5] = - 3 * (uniform_velocity[i] - init_velocity[i]) / pow(path_time[0], 4.0);
                coefficient[0][i][6] = (uniform_velocity[i] - init_velocity[i]) / pow(path_time[0], 5.0);
                coefficient[0][i][7] = 0.0;
            } else {
                coefficient[0][i][3] = 0.0;
                coefficient[0][i][4] = 0.0;
                coefficient[0][i][5] = 0.0;
                coefficient[0][i][6] = 0.0;
                coefficient[0][i][7] = 0.0;
            }
        }
    }

    coefficient[1].resize(vector_size);
    for (int i = 0 ; i < vector_size; i++) {
        coefficient[1][i].resize(degree);
        coefficient[1][i][0] = init_position[i] + path_time[0] * (init_velocity[i] + uniform_velocity[i]) / 2.0;
        coefficient[1][i][1] = uniform_velocity[i];
        coefficient[1][i][2] = 0.0;
        coefficient[1][i][3] = 0.0;
        coefficient[1][i][4] = 0.0;
        coefficient[1][i][5] = 0.0;
        if (degree_ == 7) {
            coefficient[1][i][6] = 0.0;
            coefficient[1][i][7] = 0.0;
        }
    }

    coefficient[2].resize(vector_size);
    for (int i = 0 ; i < vector_size; i++) {
        coefficient[2][i].resize(degree);
        coefficient[2][i][0] = finish_position[i] - path_time[2] * (finish_velocity[i] + uniform_velocity[i]) / 2.0;
        coefficient[2][i][1] = uniform_velocity[i];
        coefficient[2][i][2] = 0.0;
        if (degree_ == 5) {
            if (!isEqual(path_time[2], 0.0)) {
                coefficient[2][i][3] = - (uniform_velocity[i] - finish_velocity[i]) / pow(path_time[2], 2.0);
                coefficient[2][i][4] = (uniform_velocity[i] - finish_velocity[i]) / (2.0 * pow(path_time[2], 3.0));
                coefficient[2][i][5] = 0.0;
            } else {
                coefficient[2][i][3] = 0.0;
                coefficient[2][i][4] = 0.0;
                coefficient[2][i][5] = 0.0;
            }
        } else if (degree_ == 7) {
            if (!isEqual(path_time[2], 0.0)) {
                coefficient[2][i][3] = 0.0;
                coefficient[2][i][4] = 5 * (finish_velocity[i] - uniform_velocity[i]) / 2 / pow(path_time[2], 3.0);
                coefficient[2][i][5] = - 3 * (finish_velocity[i] - uniform_velocity[i]) / pow(path_time[2], 4.0);
                coefficient[2][i][6] = (finish_velocity[i] - uniform_velocity[i]) / pow(path_time[2], 5.0);
                coefficient[2][i][7] = 0.0;
            } else {
                coefficient[2][i][3] = 0.0;
                coefficient[2][i][4] = 0.0;
                coefficient[2][i][5] = 0.0;
                coefficient[2][i][6] = 0.0;
                coefficient[2][i][7] = 0.0;
            }
        }
    }

    return coefficient;
}

vector<double> PathPlanner::calUniformVelocity(const double max_time, const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> distance, const vector<double> acceleration) {
    int vector_size = init_velocity.size();
    vector<double> uniform_velocity(vector_size);
    for (int i = 0; i < vector_size; i++) {
        double large_velocity = 0.0;
        double small_velocity = 0.0;
        if (init_velocity[i] <= finish_velocity[i]) {
            large_velocity = finish_velocity[i];
            small_velocity = init_velocity[i];
        }
        else {
            large_velocity = init_velocity[i];
            small_velocity = finish_velocity[i];
        }

        double distance_up = divideWithFloatingError(fabs(large_velocity - small_velocity) * (small_velocity + large_velocity), 2.0 * acceleration[i]) + large_velocity * (max_time - divideWithFloatingError(large_velocity - small_velocity, acceleration[i]));
        double distance_down = divideWithFloatingError(fabs(large_velocity - small_velocity) * (small_velocity + large_velocity), 2.0 * acceleration[i]) + small_velocity * (max_time - divideWithFloatingError(large_velocity - small_velocity, acceleration[i]));

        if (distance_up <= distance[i]) {
            double temp = pow(acceleration[i] * max_time + small_velocity + large_velocity, 2.0) - 2.0 * (2.0 * acceleration[i] * distance[i] + pow(small_velocity, 2.0) + pow(large_velocity, 2.0));
            uniform_velocity[i] = (acceleration[i] * max_time + small_velocity + large_velocity - sqrt(fabs(temp))) / 2.0;
        }
        else if (distance_down < distance[i] && distance[i] < distance_up) {
            uniform_velocity[i] = divideWithFloatingError(distance[i] + divideWithFloatingError(pow(small_velocity, 2.0) - pow(large_velocity, 2.0), 2.0 * acceleration[i]), max_time - divideWithFloatingError(large_velocity - small_velocity, acceleration[i]));
        }
        else {
            double temp = pow(acceleration[i] * max_time - small_velocity - large_velocity, 2.0) - 2.0 * (-2.0 * acceleration[i] * distance[i] + pow(small_velocity, 2.0) + pow(large_velocity, 2.0));
            uniform_velocity[i] = (-acceleration[i] * max_time + small_velocity + large_velocity + sqrt(fabs(temp))) / 2.0;
        }

    }
    return uniform_velocity;
}

vector<vector<double>> PathPlanner::calPathTime(const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> uniform_velocity, const double max_time, const vector<double> acceleration) {
    int vector_size = init_velocity.size();
    vector<vector<double>> path_time(vector_size);
    for (int i = 0; i < vector_size; i++) {
        path_time[i].resize(3);
        //���� �ð�
        if (!isEqual(acceleration[i], 0.0)) {
            path_time[i][0] = fabs(uniform_velocity[i] - init_velocity[i]) / acceleration[i];
        }
        else {
            path_time[i][0] = 0.0;
        }

        //���� �ð�
        if (!isEqual(acceleration[i], 0.0)) {
            path_time[i][2] = fabs(uniform_velocity[i] - finish_velocity[i]) / acceleration[i];
        }
        else {
            path_time[i][2] = 0.0;
        }

        //��� �ð�
        path_time[i][1] = max_time - path_time[i][0] - path_time[i][2];
    }
    return path_time;
}

vector<double> PathPlanner::compareVelocity(const vector<double> max_velocity, const vector<double> target_velocity) {
    int vector_size = max_velocity.size();
    vector<double> velocity_temp(vector_size);
    for (int i = 0; i < vector_size; i++) {
        double target_velocity_temp = fabs(target_velocity[i]);
        if (max_velocity[i] >= target_velocity_temp) {
            velocity_temp[i] = target_velocity_temp;
        }
        else if (max_velocity[i] <= (-target_velocity_temp)) {
            velocity_temp[i] = -target_velocity_temp;
        }
        else {
            velocity_temp[i] = max_velocity[i];
        }
    }
    return velocity_temp;
}

vector<vector<vector<double>>> PathPlanner::calTrapezoidCoefficient(const vector<double> init_position, const vector<double> finish_position, const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> uniform_velocity, const vector<vector<double>> path_time) {
    vector<vector<vector<double>>> coefficient(3);
    int vector_size = init_position.size();

    coefficient[0].resize(vector_size);
    for (int i = 0; i < vector_size; i++) {
        coefficient[0][i].resize(3);
        coefficient[0][i][0] = init_position[i];
        coefficient[0][i][1] = init_velocity[i];
        coefficient[0][i][2] = divideWithFloatingError(uniform_velocity[i] - init_velocity[i], 2.0 * path_time[i][0]);
    }

    coefficient[1].resize(vector_size);
    for (int i = 0; i < vector_size; i++) {
        coefficient[1][i].resize(3);
        coefficient[1][i][0] = init_position[i] + path_time[i][0] * (init_velocity[i] + uniform_velocity[i]) / 2.0;
        coefficient[1][i][1] = uniform_velocity[i];
        coefficient[1][i][2] = 0.0;
    }

    coefficient[2].resize(vector_size);
    for (int i = 0; i < vector_size; i++) {
        coefficient[2][i].resize(3);
        coefficient[2][i][0] = finish_position[i] - path_time[i][2] * (finish_velocity[i] + uniform_velocity[i]) / 2.0;
        coefficient[2][i][1] = uniform_velocity[i];
        coefficient[2][i][2] = divideWithFloatingError(finish_velocity[i] - uniform_velocity[i], 2.0 * path_time[i][2]);
    }

    return coefficient;
}

vector<vector<vector<double>>> PathPlanner::calPathCoefficient(const vector<double> init_position, const vector<double> finish_position, const vector<double> init_velocity, const vector<double> finish_velocity, const vector<double> uniform_velocity, const vector<vector<double>> path_time) {
    vector<vector<vector<double>>> coefficient(3);
    int vector_size = init_position.size();
    int degree = degree_ + 1;

    coefficient[0].resize(vector_size);
    for (int i = 0; i < vector_size; i++) {
        coefficient[0][i].resize(degree);
        coefficient[0][i][0] = init_position[i];
        coefficient[0][i][1] = init_velocity[i];
        coefficient[0][i][2] = 0.0;
        if (degree_ == 5) {
            if (!isEqual(path_time[i][0], 0.0)) {
                coefficient[0][i][3] = (uniform_velocity[i] - init_velocity[i]) / pow(path_time[i][0], 2.0);
                coefficient[0][i][4] = (init_velocity[i] - uniform_velocity[i]) / (2.0 * pow(path_time[i][0], 3.0));
                coefficient[0][i][5] = 0.0;
            }
            else {
                coefficient[0][i][3] = 0.0;
                coefficient[0][i][4] = 0.0;
                coefficient[0][i][5] = 0.0;
            }
        }
        else if (degree_ == 7) {
            if (!isEqual(path_time[i][0], 0.0)) {
                coefficient[0][i][3] = 0.0;
                coefficient[0][i][4] = 5 * (uniform_velocity[i] - init_velocity[i]) / 2 / pow(path_time[i][0], 3.0);
                coefficient[0][i][5] = -3 * (uniform_velocity[i] - init_velocity[i]) / pow(path_time[i][0], 4.0);
                coefficient[0][i][6] = (uniform_velocity[i] - init_velocity[i]) / pow(path_time[i][0], 5.0);
                coefficient[0][i][7] = 0.0;
            }
            else {
                coefficient[0][i][3] = 0.0;
                coefficient[0][i][4] = 0.0;
                coefficient[0][i][5] = 0.0;
                coefficient[0][i][6] = 0.0;
                coefficient[0][i][7] = 0.0;
            }
        }
    }

    coefficient[1].resize(vector_size);
    for (int i = 0; i < vector_size; i++) {
        coefficient[1][i].resize(degree);
        coefficient[1][i][0] = init_position[i] + path_time[i][0] * (init_velocity[i] + uniform_velocity[i]) / 2.0;
        coefficient[1][i][1] = uniform_velocity[i];
        coefficient[1][i][2] = 0.0;
        coefficient[1][i][3] = 0.0;
        coefficient[1][i][4] = 0.0;
        coefficient[1][i][5] = 0.0;
        if (degree_ == 7) {
            coefficient[1][i][6] = 0.0;
            coefficient[1][i][7] = 0.0;
        }
    }

    coefficient[2].resize(vector_size);
    for (int i = 0; i < vector_size; i++) {
        coefficient[2][i].resize(degree);
        coefficient[2][i][0] = finish_position[i] - path_time[i][2] * (finish_velocity[i] + uniform_velocity[i]) / 2.0;
        coefficient[2][i][1] = uniform_velocity[i];
        coefficient[2][i][2] = 0.0;
        if (degree_ == 5) {
            if (!isEqual(path_time[i][2], 0.0)) {
                coefficient[2][i][3] = -(uniform_velocity[i] - finish_velocity[i]) / pow(path_time[i][2], 2.0);
                coefficient[2][i][4] = (uniform_velocity[i] - finish_velocity[i]) / (2.0 * pow(path_time[i][2], 3.0));
                coefficient[2][i][5] = 0.0;
            }
            else {
                coefficient[2][i][3] = 0.0;
                coefficient[2][i][4] = 0.0;
                coefficient[2][i][5] = 0.0;
            }
        }
        else if (degree_ == 7) {
            if (!isEqual(path_time[i][2], 0.0)) {
                coefficient[2][i][3] = 0.0;
                coefficient[2][i][4] = 5 * (finish_velocity[i] - uniform_velocity[i]) / 2 / pow(path_time[i][2], 3.0);
                coefficient[2][i][5] = -3 * (finish_velocity[i] - uniform_velocity[i]) / pow(path_time[i][2], 4.0);
                coefficient[2][i][6] = (finish_velocity[i] - uniform_velocity[i]) / pow(path_time[i][2], 5.0);
                coefficient[2][i][7] = 0.0;
            }
            else {
                coefficient[2][i][3] = 0.0;
                coefficient[2][i][4] = 0.0;
                coefficient[2][i][5] = 0.0;
                coefficient[2][i][6] = 0.0;
                coefficient[2][i][7] = 0.0;
            }
        }
    }

    return coefficient;
}

vector<vector<double>> PathPlanner::calPolynominalCoefficient(const vector<double> init_position, const vector<double> init_velocity, const vector<double> finish_velocity, const double path_time) {
    int vector_size = init_position.size();
    vector<vector<double>> coefficient(vector_size);
    int degree = degree_ + 1;

    for (int i = 0; i < vector_size; i++) {
        coefficient[i].resize(degree);
        coefficient[i][0] = init_position[i];
        coefficient[i][1] = init_velocity[i];
        coefficient[i][2] = 0.0;
        if (degree_ == 5) {
            if (!isEqual(path_time, 0.0)) {
                coefficient[i][3] = (finish_velocity[i] - init_velocity[i]) / pow(path_time, 2.0);
                coefficient[i][4] = (init_velocity[i] - finish_velocity[i]) / (2.0 * pow(path_time, 3.0));
                coefficient[i][5] = 0.0;
            }
            else {
                coefficient[i][3] = 0.0;
                coefficient[i][4] = 0.0;
                coefficient[i][5] = 0.0;
            }
        }
        else if (degree_ == 7) {
            if (!isEqual(path_time, 0.0)) {
                coefficient[i][4] = 5 * (finish_velocity[i] - init_velocity[i]) / 2.0 / pow(path_time, 3.0);
                coefficient[i][5] = -3 * (finish_velocity[i] - init_velocity[i]) / pow(path_time, 4.0);
                coefficient[i][6] = (finish_velocity[i] - init_velocity[i]) / pow(path_time, 5.0);
                coefficient[i][7] = 0.0;
            }
            else {
                coefficient[i][4] = 0.0;
                coefficient[i][5] = 0.0;
                coefficient[i][6] = 0.0;
                coefficient[i][7] = 0.0;
            }
        }
    }
    return coefficient;
}

vector<vector<vector<double>>> PathPlanner::calBackwardPolynomialCoefficienit(const vector<double> init_position, const vector<double> init_velocity, const double unit_time) {
    int vector_size = init_position.size();
    vector<vector<vector<double>>> coefficient(vector_size);
    int degree = 5;

    coefficient.resize(3);
    for (int i = 0; i < 3; i++) {
        coefficient[i].resize(vector_size);
    }
    for (int i = 0; i < vector_size; i++) {
        coefficient[0][i].resize(degree+1);
        coefficient[1][i].resize(degree+1);
        coefficient[2][i].resize(degree+1);

        if (!isEqual(unit_time, 0.0)) {
            coefficient[0][i][0] = init_position[i];
            coefficient[0][i][1] = init_velocity[i];
            coefficient[0][i][2] = 0.0;
            coefficient[0][i][3] = -init_velocity[i] / pow(unit_time, 2.0);
            coefficient[0][i][4] = init_velocity[i] / (2.0 * pow(unit_time, 3.0));
            coefficient[0][i][5] = 0.0;

            coefficient[1][i][0] = init_position[i] + 2*init_velocity[i]*unit_time;
            coefficient[1][i][1] = -5*init_velocity[i];
            coefficient[1][i][2] = 6*init_velocity[i] / unit_time;
            coefficient[1][i][3] = -3*init_velocity[i] / pow(unit_time, 2.0);
            coefficient[1][i][4] = init_velocity[i] / (2.0  * pow(unit_time, 3.0));
            coefficient[1][i][5] = 0.0;

            coefficient[2][i][0] = init_position[i] - 14*init_velocity[i]*unit_time;
            coefficient[2][i][1] = 27*init_velocity[i];
            coefficient[2][i][2] = -18*init_velocity[i]/unit_time;
            coefficient[2][i][3] = 5*init_velocity[i]/pow(unit_time, 2.0);
            coefficient[2][i][4] = -init_velocity[i] / (2.0 * pow(unit_time, 3.0));
            coefficient[2][i][5] = 0.0;
        } else {
            coefficient[0][i][0] = 0.0;
            coefficient[0][i][1] = 0.0;
            coefficient[0][i][2] = 0.0;
            coefficient[0][i][3] = 0.0;
            coefficient[0][i][4] = 0.0;
            coefficient[0][i][5] = 0.0;

            coefficient[1][i][0] = 0.0;
            coefficient[1][i][1] = 0.0;
            coefficient[1][i][2] = 0.0;
            coefficient[1][i][3] = 0.0;
            coefficient[1][i][4] = 0.0;
            coefficient[1][i][5] = 0.0;

            coefficient[2][i][0] = 0.0;
            coefficient[2][i][1] = 0.0;
            coefficient[2][i][2] = 0.0;
            coefficient[2][i][3] = 0.0;
            coefficient[2][i][4] = 0.0;
            coefficient[2][i][5] = 0.0;
        }
    }

    return coefficient;
}

vector<double> PathPlanner::calCrossProduct(const vector<double> vec_1, const vector<double> vec_2) {
    unsigned int first_vector_size = vec_1.size();
    unsigned int second_vector_size = vec_2.size();
    vector<double> product_vector(first_vector_size);

    if (first_vector_size == second_vector_size) {
        if (first_vector_size == 3) {
            product_vector[0] = vec_1[1] * vec_2[2] - vec_1[2] * vec_2[1];
            product_vector[1] = vec_1[2] * vec_2[0] - vec_1[0] * vec_2[2];
            product_vector[2] = vec_1[0] * vec_2[1] - vec_1[1] * vec_2[0];
        }
        else {
            product_vector = vec_1;
        }
    }
    else {
        product_vector = vec_1;
    }
    return product_vector;
}

double PathPlanner::calVectorMagnitude(vector<double> vec) {
    double temp = 0.0;
    unsigned int first_vector_size = vec.size();

    for (unsigned int i = 0; i < first_vector_size; i++) {
        temp += vec[i] * vec[i];
    }
    return sqrt(temp);
}

vector<double> PathPlanner::calVectorNomalize(const vector<double> vec) {
    unsigned int first_vector_size = vec.size();
    vector<double> unit_vector(first_vector_size);

    double vector_magnitude = calVectorMagnitude(vec);
    for (unsigned int i = 0; i < first_vector_size; i++) {
        unit_vector[i] = divideWithFloatingError(vec[i], vector_magnitude);
    }
    return unit_vector;
}

ModelMatrix PathPlanner::convertRPYtoRotationMatrix(const vector<double> RPY) {
    ModelMatrix rotation_matrix(3, 3);

    rotation_matrix.set(0, 0, cos(RPY[2] * kDeg2Rad) * cos(RPY[1] * kDeg2Rad));
    rotation_matrix.set(0, 1, cos(RPY[2] * kDeg2Rad) * sin(RPY[1] * kDeg2Rad) * sin(RPY[0] * kDeg2Rad) - sin(RPY[2] * kDeg2Rad) * cos(RPY[0] * kDeg2Rad));
    rotation_matrix.set(0, 2, cos(RPY[2] * kDeg2Rad) * sin(RPY[1] * kDeg2Rad) * cos(RPY[0] * kDeg2Rad) + sin(RPY[2] * kDeg2Rad) * sin(RPY[0] * kDeg2Rad));

    rotation_matrix.set(1, 0, sin(RPY[2] * kDeg2Rad) * cos(RPY[1] * kDeg2Rad));
    rotation_matrix.set(1, 1, sin(RPY[2] * kDeg2Rad) * sin(RPY[1] * kDeg2Rad) * sin(RPY[0] * kDeg2Rad) + cos(RPY[2] * kDeg2Rad) * cos(RPY[0] * kDeg2Rad));
    rotation_matrix.set(1, 2, sin(RPY[2] * kDeg2Rad) * sin(RPY[1] * kDeg2Rad) * cos(RPY[0] * kDeg2Rad) - cos(RPY[2] * kDeg2Rad) * sin(RPY[0] * kDeg2Rad));

    rotation_matrix.set(2, 0, -sin(RPY[1] * kDeg2Rad));
    rotation_matrix.set(2, 1, cos(RPY[1] * kDeg2Rad) * sin(RPY[0] * kDeg2Rad));
    rotation_matrix.set(2, 2, cos(RPY[1] * kDeg2Rad) * cos(RPY[0] * kDeg2Rad));

    return rotation_matrix;
}

ModelMatrix PathPlanner::convertNormalVectortoRotationMatrix(const double theta, const vector<double> normal_vector) {
    ModelMatrix rotation_matrix(3, 3);

    rotation_matrix.set(0, 0, normal_vector[0] * normal_vector[0] * (1 - cos(theta * kDeg2Rad)) + cos(theta * kDeg2Rad));
    rotation_matrix.set(0, 1, normal_vector[0] * normal_vector[1] * (1 - cos(theta * kDeg2Rad)) - normal_vector[2] * sin(theta * kDeg2Rad));
    rotation_matrix.set(0, 2, normal_vector[0] * normal_vector[2] * (1 - cos(theta * kDeg2Rad)) + normal_vector[1] * sin(theta * kDeg2Rad));
    rotation_matrix.set(1, 0, normal_vector[1] * normal_vector[0] * (1 - cos(theta * kDeg2Rad)) + normal_vector[2] * sin(theta * kDeg2Rad));
    rotation_matrix.set(1, 1, normal_vector[1] * normal_vector[1] * (1 - cos(theta * kDeg2Rad)) + cos(theta * kDeg2Rad));
    rotation_matrix.set(1, 2, normal_vector[1] * normal_vector[2] * (1 - cos(theta * kDeg2Rad)) - normal_vector[0] * sin(theta * kDeg2Rad));
    rotation_matrix.set(2, 0, normal_vector[2] * normal_vector[0] * (1 - cos(theta * kDeg2Rad)) - normal_vector[1] * sin(theta * kDeg2Rad));
    rotation_matrix.set(2, 1, normal_vector[2] * normal_vector[1] * (1 - cos(theta * kDeg2Rad)) + normal_vector[0] * sin(theta * kDeg2Rad));
    rotation_matrix.set(2, 2, normal_vector[2] * normal_vector[2] * (1 - cos(theta * kDeg2Rad)) + cos(theta * kDeg2Rad));

    return rotation_matrix;
}
