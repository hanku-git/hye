/**
 * @file path_planner.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Class for trajectory generation
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include <vector>

#include "robot_parameter.hpp"
#include "model_matrix.hpp"

const int kBSplineWaypointNumMin = 10; /**< Minimum waypoint number of B-spline */

using namespace std;

/**
 * @brief Class for trajectory generation
 *
 * @details Creates trapezoidal trajectories, circular trajectories, blending trajectories,
 * sine trajectories, and b-spline trajectories, including the most used polynomial trajectories,
 * and outputs position, velocity, and acceleration in real time.
 */
class PathPlanner {
public:
    /**
     * @brief Describes the path type.
     */
    enum class PathType {
        NONE,       /**< No path            */
        STOP,       /**< Stop path          */
        TRAPEZOID,  /**< Trapezoid path     */
        POLYNOMIAL, /**< Polynomial path    */
        LINEAR,     /**< Linear paths       */
        CIRCULAR,   /**< Circular path      */
        B_SPLINE,   /**< B-spline path      */
        BLENDING,   /**< Blending path      */
        SINE,       /**< Sine path          */
        BACKWARD,   /**< Backward path      */
    };

    /**
     * @brief Describes the circular type.
     */
    enum class CircularType {
        UNCONSTRAINT_RPY,   /**< Unconstraint rpy   */
        TANGENT_RPY,        /**< Tangent rpy        */
    };

    /**
     * @brief Describes the blending error type.
     */
    enum class PathErrorType {
        NONE,              /**< No error in path                        */
        COLINEAR,          /**< At least 3 waypoints are colinear       */
        SAME_WAYPOINTS,    /**< At least 3 waypoints are colinear       */
        LARGE_RADIUS,      /**< Radius is bigger than half of distances */
        INIT_VEL,          /**< Init velocity is non-zero velocity      */
        MIN_WAYPOINT,      /**< Minimum waypoint error */
        MAX_WAYPOINT,      /**< Maximum waypoint error */
        ZERO_TARGET_VEL,   /**< Target velocity is too low or negative value */
        ZERO_TARGET_ACC,   /**< Target acceleration is too low or negative value */
        CLOSE_WAYPOINTS,   /**< Adjacent waypoints are too close */
    };

    /**
     * @brief Parameter indicating path error for each waypoint
     */
    typedef struct _PathErrorParameter {
        vector<int> waypoint_index; /**< Index of waypoint */
        double max_radius;               /**< Maximum radius of corresponding waypoint */

        _PathErrorParameter() { /**< Struct constructor for setting initial values */
            waypoint_index.clear();
        }
    } PathErrorParameter;

    /**
     * @brief Parameter indicating path error
     */
    typedef struct _PathError {
        PathErrorType type; /**< Path error */
        vector<PathErrorParameter> path_error_parameter; /**< Path error parameter */

        _PathError() { /**< Struct constructor for setting initial values */
            type = PathErrorType::NONE;
            path_error_parameter.clear();
        }
    } PathError;

public:
    /**
     * @brief Construct a new Path Planner object
     */
    PathPlanner();

    /**
     * @brief Construct a new Path Planner object
     *
     * @details In the case of an object for a cartesian space trajectory, calculations for
     * 3 position components and 3 orientation components are performed separately.
     * The cartesian space degree of freedom only works normally with 6 degrees of freedom.
     *
     * @param[in] is_cartesian_space Path planning space (true: CS, false: JS)
     * @param[in] dof Degree of freedom
     */
    PathPlanner(bool is_cartesian_space, const unsigned int dof = 6);

    /**
     * @brief Destroy the Path Planner object
     */
    ~PathPlanner();

public:
    /**
     * @brief Set init position
     *
     * @param[in] init_time Init time of trajectory
     * @param[in] position Init position of trajectory
     */
    void setInitPosition(const double init_time, const vector<double> position);

    /**
     * @brief Set init position for 1 dof planning.
     *
     * @param[in] init_time Init time of trajectory
     * @param[in] position Init position of trajectory
     */
    void setInitPosition(const double init_time, const double position);

    /**
     * @brief Set target velocity
     *
     * @param[in] target_velocity Target velocity of trajectory
     */
    void setTargetVelocity(const vector<double> target_velocity);

    /**
     * @brief Set target acceleration.
     *
     * @param[in] target_acceleration Target acceleration of trajectory
     */
    void setTargetAcceleration(const vector<double> target_acceleration);

    /**
     * @brief Set polynominal degree (5 or 7)
     *
     * @details Only 5 or 7 is possible, and when other values are entered,
     * a trajectory is created with a 5th order polynomial.
     *
     * @param[in] degree Polynominal degree of polynomial trajectory
     */
    void setPolynominalDegree(const int degree);

    /**
     * @brief A function that stops by setting the current position as the initial position
     *  (different from stop path)
     *
     * @details After setting the current position vector as the initial position, set
     * the trajectory mode to NOPATH.
     *
     * @param[in] position Init position of trajectory
     */
    void genNoPath(const vector<double> position);

    /**
     * @brief Function to create stop motion (different from no path)
     *
     * @details After setting the current speed as the initial speed, the moving direction
     * is calculated. The maximum stop time is calculated using the moving direction and
     * maximum acceleration, and the back motion time is calculated. After that, set
     * the trajectory mode to STOP.
     *
     * @param[in] stop_time Time to stop
     */
    void genStopPath(const double stop_time = 0.0);

    /**
     * @brief Generate trapezoidal path
     *
     * @details Includes code to solve the problem of divergence in the process of changing
     * the angle (orientation). After checking the moving direction, the maximum transit time
     * of the trajectory is calculated using the predetermined maximum speed and maximum
     * acceleration conditions. Recalculate the maximum speed and acceleration time of the
     * trajectory using the maximum transit time. Set the trajectory mode to TRAPEZOID after
     * designating the location at the start and end of the trajectory
     *
     * @param[in] waypoint Pose of each waypoint
     */
    void genTrapezoidPath(const vector<double> waypoint);

    /**
     * @brief Generate polynomial path
     *
     * @details Includes code to solve the problem of divergence in the process of changing
     * the angle (orientation). The current velocity is obtained from a member variable with
     * the current velocity value and the operation is performed. Set the maximum acceleration
     * to 2/3 of input acceleration, the member variable with the maximum acceleration value.
     * When the velocity profile is generated in the form of a triangular velocity profile,
     * the maximum velocity is calculated to synchronize the trajectory. The constant speed is
     * calculated by comparing the maximum speed  of the predetermined trajectory with the
     * previously calculated maximum speed. Calculate the maximum operating time by calculating
     * the acceleration time, deceleration time, and constant speed time using a constant
     * speed. After calculating the coefficient of the fifth-order polynomial trajectory used
     * in the acceleration and deceleration sections, set the trajectory mode to POLYNOMIAL.
     *
     * @param[in] waypoint Pose of each waypoint
     */
    PathError genPolynomialPath(const vector<double> waypoint);

    /**
     * @brief Generate polynomial path with target veloricty and acceleration
     *
     * @details Calculate the total number of routes using the number of waypoints.
     * The waypoints of the entire route are calculated using the displacement information
     * between the current location information and waypoints. After calculating the parameter
     * information of the S-curve path corresponding to the acceleration/deceleration section
     * and constant speed section using the starting point, speed at the starting point, and
     * displacement information, set the trajectory mode to POLYNOMIAL.
     *
     * @param[in] waypoints Pose of each waypoint
     * @param[in] target_velocities Target velocities
     * @param[in] target_accelerations Target accelerations
     */
    PathError genPolynomialPath(const vector<vector<double>> waypoints,
                                const vector<vector<double>> target_velocities,
                                const vector<vector<double>> target_accelerations,
                                const vector<vector<double>> finish_velocities);

    /**
     * @brief Generate linear path
     *
     * @param[in] waypoint Pose of each waypoint
     */
    PathError genLinearPath(const vector<double> waypoint);

    /**
     * @brief Generate circular path
     *
     * @details After calculating the unit direction vector, two basic unit direction vectors
     * of the circular path are calculated using this. After that, the displacement vector
     * is calculated and an arc trajectory is generated in the form of an S-curve trajectory
     * using the generatePolynomialPath function.
     *
     * @param[in] waypoints Pose of each waypoint
     * @param[in] circular_mode Circular type
     */
    PathError genCircularPath(const vector<vector<double>> waypoints,
                              const CircularType circular_mode);

    /**
     * @brief Generate B-Spline trajectory
     *
     * @param[in] p_degree P-degree
     * @param[in] path_law Path law
     * @param[in] target_velocity Target velocities of each waypoint
     * @param[in] waypoints Pose of each waypoint
     * @param[in] velocities Velocities of each waypoint
     */
    PathError genBSplineTrajectory(const int p_degree,
                                   const int path_law,
                                   const vector<vector<double>> waypoints,
                                   const vector<vector<double>> velocities,
                                   const double target_velocity,
                                   const double target_acceleration);

    /**
     * @brief Calculate B-spline trajectory length
     *
     * @return double Length
     */
    double calBSplineTrajectoryLength();

    /**
     * @brief Generate B-Spline const velocity parameter
     *
     * @param[in] p_degree P-degree
     * @param[in] path_law Path law
     * @param[in] waypoints Pose of each waypoint
     * @param[in] velocities Velocities of each waypoint
     * @param[in] length Length
     * @param[in] target_velocity Target velocities of each waypoint
     * @param[in] target_acceleration Target accelerations of each waypoint
     */
    PathError genBSplineConstVelocityPath(const int p_degree,
                                          const int path_law,
                                          const vector<vector<double>> waypoints,
                                          const vector<vector<double>> velocities,
                                          const double length,
                                          const double target_velocity,
                                          const double target_acceleration);

    /**
     * @brief Generate blending path
     *
     * @return Velocities of each waypoint
     * @param[in] target_waypoints Pose of each waypoint
     * @param[in] target_velocities Velocities of each waypoint
     * @param[in] target_accelerations Accelerations of each waypoint
     * @param[in] radiuses Blending radiuses
     * @param[in] radius_option Radius option
     */
    vector<double> calBlendingWaypointsVelocity(const vector<vector<double>> target_waypoints,
                                                const vector<double> target_velocities,
                                                const vector<double> target_accelerations,
                                                const vector<double> radiuses,
                                                const bool radius_option);

    /**
     * @brief Generate blending path
     *
     * @return Blending path error
     * @param[in] target_waypoints Pose of each waypoint
     * @param[in] target_velocities Velocities of each waypoint
     * @param[in] target_accelerations Accelerations of each waypoint
     * @param[in] radiuses Blending radiuses
     * @param[in] radius_option Radius option
     * @param[in] waypoints_velocity Waypoints velocity
     */
    PathError genBlendingPath(const vector<vector<double>> target_waypoints,
                              const vector<double> target_velocities,
                              const vector<double> target_accelerations,
                              const vector<double> radiuses,
                              const bool radius_option,
                              const vector<double> waypoints_velocity);

    /**
    * @brief Generate sine path
    *
    * @param[in] operation_time Operation time
    */
    void genSinePath(const double operation_time);

    /**
     * @brief generate backward path.
     * @param[in] unit_time unit time.
     */
    void genBackwardPath(const double backward_time);

    /**
     * @brief Function to calculate trajectory in time
     *
     * @details Separate each trajectory mode with a statement and execute the calculation
     * function for the corresponding trajectory mode. Ex) If the trajectory mode is
     * POLYNOMIAL, calculate the S-curve trajectory through calculatePolynomialPath(t)
     *
     * @param[in] t Time
     */
    void cal(const double t);

    /**
     * @brief Get path operation status
     *
     * @returns bool Path operation status (true, if path is operating)
     */
    bool isPathOperating();

    /**
     * @brief Get path space
     *
     * @returns bool Path space (true, if cartesian space)
     */
    bool isCartesianSpace();

    /**
     * @brief Get desired position
     *
     * @returns vector<double> Desired position
     */
    vector<double> getDesPos();

    /**
     * @brief Get desired position
     *
     * @param[out] position Desired position array
     */
    void getDesPos(double* position);

    /**
     * @brief Get desired position
     *
     * @param position Desired position array
     */
    void getDesPos(double& position);

    /**
     * @brief Get desired velocity
     *
     * @returns vector<double> Desired velocity
     */
    vector<double> getDesVel();

    /**
     * @brief Get desired velocity
     *
     * @param[out] velocity Desired velocity array
     */
    void getDesVel(double* velocity);

    /**
     * @brief Get desired acceleration
     *
     * @returns vector<double> Desired acceleration
     */
    vector<double> getDesAcc();

    /**
     * @brief Get desired acceleration
     *
     * @param[out] acceleration Desired acceleration array
     */
    void getDesAcc(double* acceleration);

    /**
     * @brief Get finish position
     *
     * @returns vector<double> Finish position
     */
    vector<double> getFinPos();

    /**
     * @brief Get finish position
     *
     * @param[out] position Finish position array
     */
    void getFinPos(double* position);

    /**
     * @brief Check the error of blending parameter
     *
     * @param[in] number Number of waypoints
     * @param[in] target_waypoints Target pose of waypoints
     * @param[in] radiuses Radius of waypoints
     * @return PathError Blending parameter error
     */
    PathError checkBlendingParameter(const int number,
                                     const vector<vector<double>> target_waypoints,
                                     const vector<double> radiuses);

private:
    // Circular-generate
    PathError genCurrentToStartPath(const vector<double> start_waypoint,
                                    vector<vector<vector<double>>>& coefficient,
                                    vector<vector<double>>& path_time);

    // B-Spline-generate
    void genBSplinePolynominalPath(const double length,
                                   const double target_velocity,
                                   const double target_accelerations);

    void genBSplineInterpoleParameter(vector<double>& interpole_parameter,
                                      vector<double>& knot_vector);

    void genBSplineBasisFunction(vector<double> interpole_parameter,
                                 vector<double> knot_vector,
                                 vector<vector<double>>& basis,
                                 vector<vector<vector<double>>>& basis_total);

    void genBSplineBasisFunctionDerivative(vector<double> knot_vector,
                                           vector<vector<vector<double>>>& basis_total,
                                           vector<vector<double>>& basis_first_derivative,
                                           vector<vector<double>>& basis_second_derivative,
                                           vector<vector<double>>& basis_third_derivative,
                                           vector<vector<double>>& basis_fourth_derivative);

    void genBSplineControlPoint(vector<vector<double>> basis,
                                vector<vector<double>> basis_first_derivative,
                                vector<vector<double>> basis_second_derivative,
                                vector<vector<double>> basis_third_derivative);

    /**
     * @brief A function that calculates stop motion with a trapezoidal velocity
     * profile trajectory
     *
     * @details The current position is maintained before the trajectory start time.
     * The position command, velocity command, and acceleration command are calculated
     * by calculating the stop motion with the trajectory of the trapezoidal velocity
     * profile using the current time information.
     *
     * @param[in] t Current time
     */
    void calStopPath(const double t);

    /**
     * @brief A function that calculates the trapezoidal trajectory in real time
     *
     * @details The current position is maintained before the trajectory start time.
     * Calculate the position command in the acceleration section, constant speed section
     * and deceleration section considering the moving direction of the trajectory.
     * After the trajectory end time, the current position is maintained.
     *
     * @param[in] t Current time
     */
    void calTrapezoidPath(const double t);

    /**
     * @brief A function that calculates the S-curve trajectory in real time
     *
     * @details The current position is maintained before the trajectory start time.
     * Calculate the S-curve trajectory before the trajectory end time to calculate
     * the position, velocity, and acceleration. After the trajectory end time, the
     * current position is maintained.
     *
     * @param[in] t Current time
     */
    void calPolynomialPath(const double t);

    /**
     * @brief A function that calculates the linear trajectory in real time
     *
     * @param[in] t Current time
     */
    void calLinearPath(const double t);

    /**
     * @brief A function that calculates the arc trajectory in real time based on
     * the current time t
     *
     * @details The time vector calculated in generateCircularPath is used to
     * classify the sections of the arc trajectory, and the necessary functions
     * are executed when the current time corresponds to each section. It is
     * divided into five sections, that is, before the start of the trajectory,
     * an acceleration section, a constant speed section, a deceleration section,
     * and after the end of the trajectory. Store the output (goal position,
     * velocity, acceleration) of the function executed in the corresponding
     * section in each global variable
     *
     * @param[in] t Current time
     */
    void calCircularPath(const double t);

    void calBSplineConstVelocity(const double t);
    void calBlendingPath(const double t);
    void calSinePath(const double t);

    /**
     * @brief A function that calculate the to move backward in real time
     *
     * @details This functions is excuted to move backward when the joint limit
     * is detected. Total trajectory is divided into three sectons. The robot
     * decelerates and stop at first section, and then move backward along the
     * fore trajectory. At the last section, robot declerate the opposite direction
     * velocity and stop.
     *
     * @param[in] t Current time
     */
    void calBackwardPath(const double t);

    // Circular-calculate
    /**
     * @brief A function that calculates the target angle, angular velocity,
     * and angular acceleration of the acceleration or deceleration section
     *
     * @details Calculate the desired angle, angular velocity, and angular acceleration
     * at the current time using the current time, the start time of the acceleration
     * section or deceleration section, and polynomial coefficients
     *
     * @param[in] delta_t Start time of acceleration section or deceleration section
     * @param[in] coeff Polynomial coefficient of acceleration section or deceleration section
     * @param[out] desire_position Desired position
     * @param[out] desire_velocity Desired velocity
     * @param[out] desire_accleration Desired acceleration
     */
    void calAccelerationPath(const double delta_t,
                             const vector<vector<double>> coeff,
                             vector<double>& desire_position,
                             vector<double>& desire_velocity,
                             vector<double>& desire_accleration);

    /**
     * @brief A function that calculates the target angle, angular velocity,
     * and angular acceleration of the constant velocity section
     *
     * @details Calculate the desired angle, angular velocity and angular
     * acceleration at the current time using the current time and the
     * start time of the constant velocity section
     *
     * @param[in] delta_t Start time of uniform velocity section
     * @param[in] coeff Polynomial coefficient of acceleration section or
     * deceleration section
     * @param[out] desire_position Desired position
     * @param[out] desire_velocity Desired velocity
     * @param[out] desire_accleration Desired acceleration
     */
    void calCruisePath(const double delta_t,
                       const vector<vector<double>> coeff,
                       vector<double>& desire_position,
                       vector<double>& desire_velocity,
                       vector<double>& desire_accleration);

    /**
     * @brief Function to convert angle information to position information
     *
     * @details Angular information (target angle, angular velocity, and angular
     * acceleration) is converted into positional information (target position,
     * velocity, and acceleration) using geometric variables calculated in
     * generateCircularPath function. The target acceleration is calculated by
     * obtaining the centripetal acceleration and the tangential acceleration
     * of the arc trajectory, respectively, and adding them linearly.
     *
     * @param[in] desire_angle_position Desired angle
     * @param[in] desire_angle_velocity Desired angular velocity
     * @param[in] desire_angle_accleration Desired angular acceleration
     * @param[out] desire_position Desired position
     * @param[out] desire_velocity Desired velocity
     * @param[out] desire_accleration Desired acceleration
     */
    void calThetatoPose(const double desire_angle_position,
                        const double desire_angle_velocity,
                        const double desire_angle_accleration,
                        vector<double>& desire_position,
                        vector<double>& desire_velocity,
                        vector<double>& desire_accleration);

    // B-Spline-calculate
    void calBSplineInterpole(const double t);

    void calBSplineBasisFunction(const double time,
                                 vector<double>& basis_p_degree,
                                 vector<double>& basis_p_1_degree,
                                 vector<double>& basis_p_2_degree,
                                 vector<double>& basis_p_3_degree);

    void calTimeLaw(const double time,
                    double& time_law,
                    double& time_law_first_derivative,
                    double& time_law_second_derivative);

    void calConstVelocityTimeLaw(const double time,
                                 const vector<double> first_derivative_path,
                                 const vector<double> second_derivative_path,
                                 const vector<double> third_derivative_path,
                                 double& time_first_derivative,
                                 double& time_second_derivative);

    void calBSplineInterpolepath(const vector<double> basis,
                                 const vector<double> basis_first_derivative,
                                 const vector<double> basis_second_derivative,
                                 const vector<double> basis_third_derivative,
                                 vector<double>& path,
                                 vector<double>& first_derivative_path,
                                 vector<double>& second_derivative_path,
                                 vector<double>& third_derivative_path);

    void calBSplineInterpoleTrajectory(const double time,
                                       const double time_law_first_derivative,
                                       const double time_law_second_derivative,
                                       const vector<double> path,
                                       const vector<double> first_derivative_path,
                                       const vector<double> second_derivative_path);

    // Blending-calculate
    void calThetatoPosition(const double desire_angle_position,
                            const double desire_angle_velocity,
                            const double desire_angle_accleration,
                            vector<double>& desire_position,
                            vector<double>& desire_velocity,
                            vector<double>& desire_accleration);

    // Path-generate
    vector<double> calMaxVelocity(const vector<double> init_velocity,
                                  const vector<double> finish_velocity,
                                  const vector<double> distance,
                                  const vector<double> acceleration);

    vector<double> compareVelocity(const vector<double> max_velocity,
                                   const vector<double> target_velocity);

    /**
     * @brief Functions that compute coefficients of trapezoidal trajectory
     *
     * @details The first term in the coefficients of the polynomial corresponds
     * to the 0th degree term.
     *
     * @param[in] init_position Initial position
     * @param[in] finish_position Finish velocity
     * @param[in] init_velocity Initial velocity
     * @param[in] finish_velocity Finish velocity
     * @param[in] uniform_velocity Uniform velocity
     * @param[in] path_time Path time
     * @return vector<vector<vector<double>>> Coefficients
     */
    vector<vector<vector<double>>> calTrapezoidCoefficient(const vector<double> init_position,
                                                           const vector<double> finish_position,
                                                           const vector<double> init_velocity,
                                                           const vector<double> finish_velocity,
                                                           const vector<double> uniform_velocity,
                                                           const vector<vector<double>> path_time);

    /**
     * @brief Functions that compute coefficients of polynomials (one section)
     *
     * @details The first term in the coefficients of the polynomial corresponds
     * to the 0th degree term, and the last term corresponds to the 5th or 7th
     * degree term.
     *
     * @param[in] init_position Initial position
     * @param[in] init_velocity Initial velocity
     * @param[in] finish_velocity Finish velocity
     * @param[in] path_time Path time
     * @return vector<vector<double>> Coefficients of polynomials
     */
    vector<vector<double>> calPolynominalCoefficient(const vector<double> init_position,
                                                     const vector<double> init_velocity,
                                                     const vector<double> finish_velocity,
                                                     const double path_time);

    /**
     * @brief Functions that compute coefficients to move backward
     *
     * @details Total trajectory is consist of three sections: deceleration,
     * move opposite direction, deceleration for opposite direction.
     *
     * @param[in] init_position Position where joint limit detected
     * @param[in] init_velocity Velocity when joint limit detected
     * @param[in] unit_time Duration time for each sections
     * @return vector<vector<vector<double>>>
     */
    vector<vector<vector<double>>> calBackwardPolynomialCoefficienit(
                                       const vector<double> init_position,
                                       const vector<double> init_velocity,
                                       const double unit_time);

    /**
     * @brief A function that computes the cross product of vectors
     *
     * @param[in] vec_1 Input vector of size 3
     * @param[in] vec_2 Input vector of size 3
     * @return vector<double> Output vector of size 3
     */
    vector<double> calCrossProduct(const vector<double> vec_1, const vector<double> vec_2);

    /**
     * @brief A function that calculates the magnitude of a vector
     *
     * @param[in] vec Input vector
     * @return double Vector size
     */
    double calVectorMagnitude(vector<double> vec);

    /**
     * @brief A function that calculates a unit direction vector
     *
     * @param[in] vec Input vector
     * @return vector<double> Unit direction vector
     */
    vector<double> calVectorNomalize(const vector<double> vec);

    ModelMatrix convertRPYtoRotationMatrix(const vector<double> RPY);
    ModelMatrix convertNormalVectortoRotationMatrix(const double theta,
                                                    const vector<double> normal_vector);

    ///@{
    /** Functions used when creating trajectories according to
     * acceleration/deceleration and constant velocity time of each axis */
    double compareTimeSum(const vector<double> init_velocity,
                          const vector<double> finish_velocity,
                          const vector<double> uniform_velocity,
                          const vector<double> distance,
                          const vector<double> acceleration);

    vector<double> calUniformVelocity(const double max_time,
                                      const vector<double> init_velocity,
                                      const vector<double> finish_velocity,
                                      const vector<double> distance,
                                      const vector<double> acceleration);

    vector<vector<double>> calPathTime(const vector<double> init_velocity,
                                       const vector<double> finish_velocity,
                                       const vector<double> uniform_velocity,
                                       const double max_time,
                                       const vector<double> acceleration);

    /**
     * @brief Functions that compute coefficients of polynomials
     * (acceleration, uniform, deceleration section)
     *
     * @details The first term in the coefficients of the polynomial corresponds
     * to the 0th degree term, and the last term corresponds to the 5th or 7th
     * degree term.
     *
     * @param init_position Initial position
     * @param finish_position Finish position
     * @param init_velocity Initial velocity
     * @param finish_velocity Finish velocity
     * @param uniform_velocity Uniform velocity
     * @param path_time Path time
     * @return vector<vector<vector<double>>> Coefficients of polynomials
     * (acceleration, uniform, deceleration section)
     */
    vector<vector<vector<double>>> calPathCoefficient(const vector<double> init_position,
                                                      const vector<double> finish_position,
                                                      const vector<double> init_velocity,
                                                      const vector<double> finish_velocity,
                                                      const vector<double> uniform_velocity,
                                                      const vector<vector<double>> path_time);
    ///@}

    ///@{
    /** Functions used when creating a trajectory with synchronization of
     * acceleration/deceleration and constant speed times of all axes */
    vector<double> compareTime(const vector<double> init_velocity,
                               const vector<double> finish_velocity,
                               const vector<double> uniform_velocity,
                               const vector<double> distance,
                               const vector<double> acceleration);

    vector<double> recalUniformVelocity(const vector<double> init_velocity,
                                        const vector<double> finish_velocity,
                                        const vector<double> distance,
                                        const vector<double> path_time);

    vector<vector<vector<double>>> calPathCoefficient(const vector<double> init_position,
                                                      const vector<double> finish_position,
                                                      const vector<double> init_velocity,
                                                      const vector<double> finish_velocity,
                                                      const vector<double> uniform_velocity,
                                                      const vector<double> path_time);
    ///@}
private:
    typedef struct _PathParameter {
        double time;
        vector<double> position;
        vector<double> velocity;
        vector<double> acceleration;
        void reset(unsigned int dof) {
            time = 0.0;
            position.resize(dof);
            velocity.resize(dof);
            acceleration.resize(dof);
        }
    } PathParameter;

    typedef struct _StopParameter {
        vector<vector<double>> coefficient;
    } StopParameter;

    typedef struct _TrapezoidParameter {
        vector<vector<double>> path_time;
        vector<vector<vector<double>>> coefficient;
    } TrapezoidParameter;

    typedef struct _PolynomialParameter {
        unsigned int count_num;
        vector<vector<vector<double>>> path_time;
        vector<vector<vector<vector<double>>>> coefficients;
    } PolynomialParameter;

    typedef struct _SineParameter {
        double operation_time;
        double init_angle;
        double operation_angle;
        double amplitude;
    } SineParameter;

    typedef struct _LinearParameter {
        vector<double> distance_unit_vector;
        vector<vector<double>> path_time;
        vector<vector<vector<double>>> coefficients;
    } LinearParameter;

    typedef struct _CircularParameter {
        vector<vector<vector<double>>> start_coefficient;
        vector<vector<double>> start_path_time;

        // AngChange(62), AngTangent(63)
        CircularType circular_mode;
        // [3][x, y, z, R, P, Y]
        vector<vector<double>> waypoints;
        // [4]
        vector<vector<double>> path_time;
        // [3][4]
        vector<vector<double>> path_distance;
        // [3]
        ModelMatrix init_rotation_matrix;
        vector<double> vector_b0;
        vector<double> vector_b1;
        vector<double> vector_n;

        vector<double> operation_time;
        vector<vector<double>> operation_distance;
        vector<double> center_point;

        vector<double> init_velocity;
        vector<double> finish_velocity;
        vector<double> uniform_velocity;
        vector<double> acceleration;
        double radius;
        vector<vector<vector<double>>> coefficient;
    } CircularParameter;

    typedef struct _BSplineParameter {
        int p_degree;
        int path_law;
        double operation_time;

        int knot_number;
        vector<double> knot_vector;
        unsigned int waypoint_num;
        vector<vector<double>> waypoints;
        unsigned int controlpoint_num;
        vector<vector<double>> controlpoints;
        vector<vector<double>> tangent_vector;

        vector<vector<double>> controlpoints_first_derivative;
        vector<vector<double>> controlpoints_second_derivative;
        vector<vector<double>> controlpoints_third_derivative;

        vector<vector<double>> path_time;
        vector<vector<vector<double>>> coefficient;

        double time;
        double time_first_derivative;
        double time_first_derivative_previous;
        double time_second_derivative;
        double length;
        double time_period;
    } BSplineParameter;

    typedef struct _BlendingParameter {
        vector<double> radius;
        vector<double> radius_percent;

        vector<vector<vector<double>>> path_time;
        vector<double> geometric_length;

        vector<vector<double>> vector_b1;
        vector<vector<double>> vector_b0;
        vector<vector<double>> circle_center_points;

        vector<vector<vector<vector<double>>>> coefficients;

        vector<vector<double>> xyz_unit_distance;
        vector<vector<double>> geometric_waypoints;
        vector<vector<double>> xyz_waypoints;

        int path_order;
        bool geometric_type;
        int geometric_order;

        typedef struct _Geometric {
            double length;
            bool type;
        } Geometric;

        vector<Geometric> geometric_segment;
    } BlendingParameter;

    typedef struct _BackwardParameter {
        double unit_time;
        vector<vector<double>> path_time;
        vector<vector<vector<double>>> coefficients;
    } BackwardParameter;

private:
    unsigned int dof_;
    int degree_;
    bool is_cartesian_space_;
    PathType path_type_;

    PathParameter init_;
    PathParameter current_;
    PathParameter finish_;

    PathParameter target_;
    PathParameter desired_;
    PathParameter limit_;

    StopParameter stop_parameter_;
    TrapezoidParameter trapezoid_Parameter_;
    PolynomialParameter polynomial_parameter_;
    SineParameter sine_parameter_;
    LinearParameter linear_parameter_;
    CircularParameter circular_parameter_;
    BSplineParameter bSpline_parameter_;
    BlendingParameter blending_parameter_;
    BackwardParameter backward_parameter_;

public:
    vector<vector<vector<vector<double>>>> getCoeff() {
        return polynomial_parameter_.coefficients;
    }
};

#endif // PATH_PLANNER_HPP
