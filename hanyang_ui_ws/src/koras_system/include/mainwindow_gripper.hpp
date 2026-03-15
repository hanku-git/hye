#ifndef MAINWINDOW_GRIPPER_H
#define MAINWINDOW_GRIPPER_H

#include <QMainWindow>
#include <QLabel>
#include <QSlider>
#include <QWidget>
#include <QGridLayout>
#include <QProcess>
#include <QDoubleSpinBox>
#include <QFileDialog>

#include <thread>
#include "qnode.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <filesystem>

// Rviz2 imports
#include "rclcpp/clock.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/display.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/window_manager_interface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/ros_integration/ros_client_abstraction.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include <rviz_common/tool.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/config.hpp>
#include "rviz_common/visualization_manager.hpp"
#include <rviz_common/view_controller.hpp>

#include "rviz_common/visualization_frame.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow_gripper; }
QT_END_NAMESPACE

const double pi = 3.14159265358979;


class MainWindow_gripper : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow_gripper(rviz_common::VisualizationFrame* frame,
                       rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abs,
                       QWidget *parent = 0);
    ~MainWindow_gripper();

    std::filesystem::path current_path = std::filesystem::current_path();
    visualization_msgs::msg::Marker base = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker point = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker arrow = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker joint_L = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker joint_R = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker object = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker frame = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker rot_b = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker rot_l = visualization_msgs::msg::Marker();
    visualization_msgs::msg::Marker rot_r = visualization_msgs::msg::Marker();
    // visualization_msgs::msg::MarkerArray marker_array = visualization_msgs::msg::MarkerArray();
    // geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
    // geometry_msgs::msg::PoseArray pose_array = geometry_msgs::msg::PoseArray();
    void DisplayGrid();
    int gripper_type = 0;
    float corr_x = 0;
    float corr_y = 0;
    float corr_z = 0;
    std::string file_path;
    std::string target_object_name_;

private:
    Ui::MainWindow_gripper *ui;

private Q_SLOTS:
    bool init();
    void move_gripper();
    void OpenFolderCallback();
    void set_gripper_type(const std::string& type);
    void set_gripper(const std::string& grp_stl);
    void set_joint(const std::string& grp_stl);
    void visualize_rotation();
    void create_rotation(Eigen::Quaterniond q, int id);
    void Object_pub();
    void Frame_pub();
    void saveGraspingPose();

  private:
    // Visualization vars

    QWidget* central_widget;
    QVBoxLayout* main_layout;

    shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr rotation_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frame_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr object_pub_;

    // Rviz vars
    rviz_common::VisualizationFrame* frame_;
    rviz_common::VisualizationManager* manager_;
    rviz_common::RenderPanel* render_panel_;
    rviz_common::Display * Gripper_, *Grid_, *Object_, *Rotation_;
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abs_;

    // QList
    QList <QDoubleSpinBox *> qlist_grasping_pose_;

};
#endif // MAINWINDOW_GRIPPER_H
