#include "mainwindow_gripper.hpp"
#include "ui_mainwindow_gripper.h"

MainWindow_gripper::MainWindow_gripper(rviz_common::VisualizationFrame* frame,
         rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr ros_node_abs,
         QWidget* parent)
    : QMainWindow(parent)
    , frame_(frame)
    , ros_node_abs_(ros_node_abs)
    , ui(new Ui::MainWindow_gripper)
{
    ui->setupUi(this);

    /*-------------- gripper pose --------------*/
    connect(ui->doubleSpinBox_x ,qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_y , qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_z , qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_rx ,qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_ry , qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_rz , qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);

    /*------------- pivot point ---------------*/
	connect(ui->doubleSpinBox_px ,qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_py , qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_pz , qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);

    connect(ui->comboBox_subGraspingPoseAxis, &QComboBox::currentTextChanged, this,
            &MainWindow_gripper::move_gripper);

	connect(ui->doubleSpinBox_gripperOpenLength,qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);

    connect(ui->pushButton_open_obj_cad, &QPushButton::clicked, this,
            &MainWindow_gripper::OpenFolderCallback);

    connect(ui->radioButton_gripper_1 , &QRadioButton::clicked, this,
            std::bind(&MainWindow_gripper::set_gripper_type, this, "KU2F"));
    connect(ui->radioButton_gripper_2 , &QRadioButton::clicked, this,
            std::bind(&MainWindow_gripper::set_gripper_type, this, "KUVG"));
    connect(ui->radioButton_gripper_3 , &QRadioButton::clicked, this,
            std::bind(&MainWindow_gripper::set_gripper_type, this, "KUPG"));

    connect(ui->pushButton_get_obj, &QPushButton::clicked, this,
            &MainWindow_gripper::Object_pub);

    connect(ui->spinBox_subGraspingPoseNum ,qOverload<int>(&QSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);
    connect(ui->doubleSpinBox_subGraspingPoseAngle ,qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            &MainWindow_gripper::move_gripper);

    //// JSON save
    connect(ui->pushButton_save_pose         , &QPushButton::clicked, this,
            &MainWindow_gripper::saveGraspingPose);

    qlist_grasping_pose_    << ui->doubleSpinBox_x
                            << ui->doubleSpinBox_y
                            << ui->doubleSpinBox_z
                            << ui->doubleSpinBox_rx
                            << ui->doubleSpinBox_ry
                            << ui->doubleSpinBox_rz;

    /*-----------------------------
        INIT MAIN RVIZ CLASSES
    -----------------------------*/
    manager_ = frame_->getManager();
    manager_-> removeAllDisplays ();

    /*-----------------------------
        ADD VISUALIZATION
    -----------------------------*/
    main_layout = ui->verticalLayout;
    main_layout->addWidget(QWidget::createWindowContainer(frame_->getRenderWindow()));

    /*-----------------------------
          DISPLAYS INIT
    -----------------------------*/
    DisplayGrid();

    init();

}

MainWindow_gripper::~MainWindow_gripper()
{
    delete ui;
    delete frame_;
    rclcpp::shutdown();
}

// Eigen::Quaterniond axisAngleToQuaternion(double rx, double ry, double rz) {
//     // Convert degrees to radians
//     rx = rx * M_PI / 180.0;
//     ry = ry * M_PI / 180.0;
//     rz = rz * M_PI / 180.0;

//     // Calculate half angles
//     double sx = sin(rx / 2.0);
//     double cx = cos(rx / 2.0);
//     double sy = sin(ry / 2.0);
//     double cy = cos(ry / 2.0);
//     double sz = sin(rz / 2.0);
//     double cz = cos(rz / 2.0);

//     // Create quaternion
//     Eigen::Quaterniond q;
//     q.w() = cx * cy * cz + sx * sy * sz;
//     q.x() = sx * cy * cz - cx * sy * sz;
//     q.y() = cx * sy * cz + sx * cy * sz;
//     q.z() = cx * cy * sz - sx * sy * cz;

//     return q;
// }

Eigen::Quaterniond axisAngleToQuaternion(double rx, double ry, double rz) {
    // Convert degrees to radians
    rx = rx * M_PI / 180.0;
    ry = ry * M_PI / 180.0;
    rz = rz * M_PI / 180.0;

    // Create quaternion
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());

    return q;
}

// Eigen::Quaterniond axisAngleToQuaternion(double rx, double ry, double rz) {

//     rx = rx * M_PI / 180.0;
//     ry = ry * M_PI / 180.0;
//     rz = rz * M_PI / 180.0;
//     double rollOver2 = rz * 0.5;
//     double sinRollOver2 = sin(rollOver2);
//     double cosRollOver2 = cos (rollOver2);
//     double pitchOver2 = ry * 0.5;
//     double sinPitchOver2 = sin (pitchOver2);
//     double cosPitchOver2 = cos (pitchOver2);
//     double yawOver2 = rx * 0.5;
//     double sinYawOver2 = sin (yawOver2);
//     double cosYawOver2 = cos (yawOver2);
//     Eigen::Quaterniond q;
//     q.w() = cosYawOver2 * cosPitchOver2 * cosRollOver2 + sinYawOver2 * sinPitchOver2 * sinRollOver2;
//     q.x() = cosYawOver2 * sinPitchOver2 * cosRollOver2 + sinYawOver2 * cosPitchOver2 * sinRollOver2;
//     q.y() = sinYawOver2 * cosPitchOver2 * cosRollOver2 - cosYawOver2 * sinPitchOver2 * sinRollOver2;
//     q.z() = cosYawOver2 * cosPitchOver2 * sinRollOver2 - sinYawOver2 * sinPitchOver2 * cosRollOver2;

//     return q;
// }

bool MainWindow_gripper::init() {
    node_ = rclcpp::Node::make_shared("Gripper_setting_node");
	marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/marker_array", 10);
    rotation_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/pose_array", 10);
    object_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/object", 10);
    frame_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/object", 10);

    return true;
}

void MainWindow_gripper::OpenFolderCallback() {
    QString file_name = QFileDialog::getOpenFileName(this, tr("Open folder"),"*.stl");

    // object_name
    file_path = file_name.toStdString();
    size_t lastSlashPos = file_path.find_last_of('/');
    target_object_name_ = file_path.substr(lastSlashPos + 1);
    target_object_name_ = target_object_name_.substr(0, target_object_name_.length() - 4);
    cout<<"taraget obj name : "<< target_object_name_<< endl;
    ui->textEdit_obj_cad_path->setText(QString::fromStdString(target_object_name_));
}

void MainWindow_gripper::set_gripper_type(const std::string& type){
    visualization_msgs::msg::MarkerArray marker_array;
	if (type == "KUVG"){
		gripper_type = 0;
		set_gripper("KUVG/KUVG_1cup.stl");
		joint_L.action = visualization_msgs::msg::Marker::DELETE;
		joint_R.action = visualization_msgs::msg::Marker::DELETE;
		marker_array.markers.push_back(base);
		marker_array.markers.push_back(joint_L);
		marker_array.markers.push_back(joint_R);
		marker_pub_->publish(marker_array);
	}
	else if (type == "KU2F"){
		gripper_type = 1;
		set_gripper("KU2F/KU2F_BASE.stl");
		set_joint("KU2F/KU2F_TIP");
		marker_array.markers.push_back(base);
		marker_array.markers.push_back(joint_L);
		marker_array.markers.push_back(joint_R);
		marker_pub_->publish(marker_array);
	}
	else if (type == "KUPG"){
		gripper_type = 2;
		set_gripper("KUPG/KUPG_BASE.stl");
		set_joint("KUPG/KUPG_TIP");
		marker_array.markers.push_back(base);
		marker_array.markers.push_back(joint_L);
		marker_array.markers.push_back(joint_R);
		marker_pub_->publish(marker_array);
	}
}

void MainWindow_gripper::set_gripper(const std::string& grp_stl) {

    base.header.frame_id = "map";
    base.header.stamp = node_->now();
    base.ns = "Gripper_Base";
    base.id = 0;
    base.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    // base.mesh_resource = "file://" + current_path.string() +  "/src/hanyang_eng_koras_system/resources/gripper_CAD/" + grp_stl;
    std::string path_str = std::string("file://") + "/home/" + USER_NAME + "/[scanDataBinPickingEyeInHand]/CAD/simulation_CAD/gripper_CAD/" + grp_stl;
    base.mesh_resource = path_str;
    base.action = visualization_msgs::msg::Marker::ADD;
    base.scale.x = 0.001;
    base.scale.y = 0.001;
    base.scale.z = 0.001;
    base.color.a = 1.0;
    base.color.r = 0.0;
    base.color.g = 1.0;
    base.color.b = 0.0;

	point.header.frame_id = "map";
    point.header.stamp = node_->now();
    point.ns = "Point";
    point.id = 0;
    point.type = visualization_msgs::msg::Marker::SPHERE;
    point.action = visualization_msgs::msg::Marker::ADD;
	point.scale.x = 0.01;
    point.scale.y = 0.01;
    point.scale.z = 0.01;
    point.color.a = 1.0;
    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;

    move_gripper();
}

void MainWindow_gripper::set_joint(const std::string& grp_stl) {

	joint_L.header.frame_id = "map";
	joint_L.header.stamp = node_->now();
	joint_L.ns = "joint_L";
	joint_L.id = 0;
	joint_L.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	// joint_L.mesh_resource = "file://" + current_path.string() +  "/src/hanyang_eng_koras_system/resources/gripper_CAD/" + grp_stl + "_L.stl";
    std::string path_str = std::string("file://") + "/home/" + USER_NAME + "/[scanDataBinPickingEyeInHand]/CAD/simulation_CAD/gripper_CAD/" + grp_stl + "_L.stl";
	joint_L.mesh_resource = path_str;
	joint_L.action = visualization_msgs::msg::Marker::ADD;
	joint_L.scale.x = 0.001;
	joint_L.scale.y = 0.001;
	joint_L.scale.z = 0.001;
	joint_L.color.a = 1.0;
	joint_L.color.r = 0.0;
	joint_L.color.g = 1.0;
	joint_L.color.b = 0.0;

	joint_R.header.frame_id = "map";
	joint_R.header.stamp = node_->now();
	joint_R.ns = "joint_R";
	joint_R.id = 0;
	joint_R.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
	// joint_R.mesh_resource = "file://" + current_path.string() +  "/src/hanyang_eng_koras_system/resources/gripper_CAD/" + grp_stl + "_R.stl";
    path_str = std::string("file://") + "/home/" + USER_NAME + "/[scanDataBinPickingEyeInHand]/CAD/simulation_CAD/gripper_CAD/" + grp_stl + "_R.stl";
	joint_R.mesh_resource = path_str;
	joint_R.action = visualization_msgs::msg::Marker::ADD;
	joint_R.scale.x = 0.001;
	joint_R.scale.y = 0.001;
	joint_R.scale.z = 0.001;
	joint_R.color.a = 1.0;
	joint_R.color.r = 0.0;
	joint_R.color.g = 1.0;
	joint_R.color.b = 0.0;

    move_gripper();
}

void MainWindow_gripper::create_rotation(Eigen::Quaterniond q, int id) {

    rot_b.id = id;
    rot_r.id = id;
    rot_l.id = id;

    Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();

    Eigen::MatrixXd dist_mat(3,1);
    dist_mat << 0, -0.035*(1950 -1 * (ui->doubleSpinBox_gripperOpenLength-> value())), 0;
    Eigen::MatrixXd tip_mat = 0.001 * rotation_matrix * dist_mat;

    Eigen::MatrixXd base_trans(3,1);
    base_trans << (ui->doubleSpinBox_x -> value()), (ui->doubleSpinBox_y -> value()), (ui->doubleSpinBox_z -> value());
    Eigen::MatrixXd base_pos = 0.001 * rotation_matrix * base_trans;

    Eigen::MatrixXd point_trans(3,1);
    point_trans << (ui->doubleSpinBox_px -> value()), (ui->doubleSpinBox_py -> value()), (ui->doubleSpinBox_pz -> value());
    Eigen::MatrixXd point_pos = 0.001 * rotation_matrix * point_trans;

    rot_b.pose.position.x = 0.001 * base_trans(0) - point_pos(0);
    rot_b.pose.position.y = 0.001 * base_trans(1) - point_pos(1);
    rot_b.pose.position.z = 0.001 * base_trans(2) - point_pos(2);
    rot_b.pose.orientation.x = q.x();
    rot_b.pose.orientation.y = q.y();
    rot_b.pose.orientation.z = q.z();
    rot_b.pose.orientation.w = q.w();
    rot_l.pose.position.x = 0.001 * base_trans(0) - point_pos(0) + tip_mat(0);
    rot_l.pose.position.y = 0.001 * base_trans(1) - point_pos(1) + tip_mat(1);
    rot_l.pose.position.z = 0.001 * base_trans(2) - point_pos(2) + tip_mat(2);
    rot_l.pose.orientation.x = q.x();
    rot_l.pose.orientation.y = q.y();
    rot_l.pose.orientation.z = q.z();
    rot_l.pose.orientation.w = q.w();
    rot_r.pose.position.x = 0.001 * base_trans(0) - point_pos(0) - tip_mat(0);
    rot_r.pose.position.y = 0.001 * base_trans(1) - point_pos(1) - tip_mat(1);
    rot_r.pose.position.z = 0.001 * base_trans(2) - point_pos(2) - tip_mat(2);
    rot_r.pose.orientation.x = q.x();
    rot_r.pose.orientation.y = q.y();
    rot_r.pose.orientation.z = q.z();
    rot_r.pose.orientation.w = q.w();

    if (gripper_type == 0){
        rot_b.action = visualization_msgs::msg::Marker::ADD;
        rot_r.action = visualization_msgs::msg::Marker::DELETE;
        rot_l.action = visualization_msgs::msg::Marker::DELETE;
    }
    else if (gripper_type == 1){

        rot_b.action = visualization_msgs::msg::Marker::ADD;
        rot_r.action = visualization_msgs::msg::Marker::ADD;
        rot_l.action = visualization_msgs::msg::Marker::ADD;
    }
}

void MainWindow_gripper::visualize_rotation() {

    visualization_msgs::msg::MarkerArray rotation_array;
    Eigen::Quaterniond q1, q2, q;
    double rx, ry, rz, rx_angle, ry_angle, rz_angle;

    rot_b = base;
    rot_l = joint_L;
    rot_r = joint_R;

    rot_b.ns = "Rot_base";
    rot_b.color.r = 0.2;
    rot_b.color.g = 0.7;
    rot_b.color.b = 0.2;
    rot_b.color.a = 0.1;

    rot_l.ns = "Rot_joint_L";
    rot_l.color.r = 0.2;
    rot_l.color.g = 0.7;
    rot_l.color.b = 0.2;
    rot_l.color.a = 0.1;

    rot_r.ns = "Rot_joint_R";
    rot_r.color.r = 0.2;
    rot_r.color.g = 0.7;
    rot_r.color.b = 0.2;
    rot_r.color.a = 0.1;

    if (ui->comboBox_subGraspingPoseAxis->currentText().toStdString() == "x"){
        rx = ui->doubleSpinBox_rx -> value();
        ry = ui->doubleSpinBox_ry -> value();
        rz = ui->doubleSpinBox_rz -> value();
        rx_angle = ui->doubleSpinBox_subGraspingPoseAngle-> value();
        ry_angle = 0;
        rz_angle = 0;
    }
    else if (ui->comboBox_subGraspingPoseAxis->currentText().toStdString() == "y"){
        rx = ui->doubleSpinBox_rx -> value();
        ry = ui->doubleSpinBox_ry -> value();
        rz = ui->doubleSpinBox_rz -> value();
        rx_angle = 0;
        ry_angle = ui->doubleSpinBox_subGraspingPoseAngle-> value();
        rz_angle = 0;
    }
    else if (ui->comboBox_subGraspingPoseAxis->currentText().toStdString() == "z"){
        rx = ui->doubleSpinBox_rx -> value();
        ry = ui->doubleSpinBox_ry -> value();
        rz = ui->doubleSpinBox_rz -> value();
        rx_angle = 0;
        ry_angle = 0;
        rz_angle = ui->doubleSpinBox_subGraspingPoseAngle-> value();
    }

    for (int i = 0; i <= ui->spinBox_subGraspingPoseNum->value(); i++){
        q1 = axisAngleToQuaternion(rx, ry, rz);
        q2 = axisAngleToQuaternion(rx_angle * i, ry_angle * i, rz_angle * i);
        q = q1 * q2;
        create_rotation(q, i * 2);
        rotation_array.markers.push_back(rot_b);
        rotation_array.markers.push_back(rot_r);
        rotation_array.markers.push_back(rot_l);
        q1 = axisAngleToQuaternion(rx, ry, rz);
        q2 = axisAngleToQuaternion(-rx_angle * i, -ry_angle * i, -rz_angle * i);
        q = q1 * q2;
        create_rotation(q, i * 2 + 1);
        rotation_array.markers.push_back(rot_b);
        rotation_array.markers.push_back(rot_r);
        rotation_array.markers.push_back(rot_l);
    }
    marker_pub_->publish(rotation_array);
}

void MainWindow_gripper::move_gripper() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::MarkerArray rotation_array;

    Eigen::Quaterniond q = axisAngleToQuaternion(ui->doubleSpinBox_rx -> value(), ui->doubleSpinBox_ry -> value() , ui->doubleSpinBox_rz -> value());
	Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();

	Eigen::MatrixXd dist_mat(3,1);
	dist_mat << 0, -0.035*(1950 -1 * (ui->doubleSpinBox_gripperOpenLength-> value())), 0; // 실제 비율 맞춰 바꿔야함
	Eigen::MatrixXd tip_mat = 0.001 * rotation_matrix * dist_mat;

	Eigen::MatrixXd base_trans(3,1);
	base_trans << (ui->doubleSpinBox_x -> value()), (ui->doubleSpinBox_y -> value()), (ui->doubleSpinBox_z -> value());
	Eigen::MatrixXd base_pos = 0.001 * rotation_matrix * base_trans;

	Eigen::MatrixXd point_trans(3,1);
	point_trans << (ui->doubleSpinBox_px -> value()), (ui->doubleSpinBox_py -> value()), (ui->doubleSpinBox_pz -> value());
	Eigen::MatrixXd point_pos = 0.001 * rotation_matrix * point_trans;

    rot_b.action = visualization_msgs::msg::Marker::DELETEALL;
    rot_r.action = visualization_msgs::msg::Marker::DELETEALL;
    rot_l.action = visualization_msgs::msg::Marker::DELETEALL;
	base.pose.position.x = 0.001 * base_trans(0) - point_pos(0);
	base.pose.position.y = 0.001 * base_trans(1) - point_pos(1);
	base.pose.position.z = 0.001 * base_trans(2) - point_pos(2);
	base.pose.orientation.x = q.x();
	base.pose.orientation.y = q.y();
	base.pose.orientation.z = q.z();
	base.pose.orientation.w = q.w();
	point.pose.position.x =  0.001 * base_trans(0);
	point.pose.position.y =  0.001 * base_trans(1);
	point.pose.position.z =  0.001 * base_trans(2);
	joint_L.pose.position.x = 0.001 * base_trans(0) - point_pos(0) + tip_mat(0);
	joint_L.pose.position.y = 0.001 * base_trans(1) - point_pos(1) + tip_mat(1);
	joint_L.pose.position.z = 0.001 * base_trans(2) - point_pos(2) + tip_mat(2);
	joint_L.pose.orientation.x = q.x();
	joint_L.pose.orientation.y = q.y();
	joint_L.pose.orientation.z = q.z();
	joint_L.pose.orientation.w = q.w();
	joint_R.pose.position.x = 0.001 * base_trans(0) - point_pos(0) - tip_mat(0);
	joint_R.pose.position.y = 0.001 * base_trans(1) - point_pos(1) - tip_mat(1);
	joint_R.pose.position.z = 0.001 * base_trans(2) - point_pos(2) - tip_mat(2);
	joint_R.pose.orientation.x = q.x();
	joint_R.pose.orientation.y = q.y();
	joint_R.pose.orientation.z = q.z();
	joint_R.pose.orientation.w = q.w();

    rotation_array.markers.push_back(rot_b);
    rotation_array.markers.push_back(rot_r);
    rotation_array.markers.push_back(rot_l);
    marker_pub_->publish(rotation_array);
	marker_array.markers.push_back(base);
	marker_array.markers.push_back(joint_L);
	marker_array.markers.push_back(joint_R);
	marker_array.markers.push_back(point);
	marker_pub_->publish(marker_array);
    Frame_pub();
    visualize_rotation();
}

void MainWindow_gripper::saveGraspingPose() {
    printf("Save the grasping pose!\n");
    std::stringstream ss;
    // index update

    size_t pose_idx = ui->spinBox_poseIndex->value();


    std::vector<double> grasping_pose(6);
    for (size_t i = 0; i < 6; i++) {
        grasping_pose[i] = qlist_grasping_pose_[i]->value();
    }
    printf("Current grasping pose [mm, deg]: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f\n", grasping_pose[0], grasping_pose[1], grasping_pose[2], grasping_pose[3], grasping_pose[4], grasping_pose[5]);

    //// json file load
    std::stringstream pose_config_path;
    pose_config_path << "/home/" << USER_NAME << "/bin_picking_ws/src/bin_picking_process/config/template_" << target_object_name_ << ".json";
    std::cout << pose_config_path.str() << std::endl;
    std::ifstream ifs_config(pose_config_path.str().c_str());
    json j_in = json::parse(ifs_config);
    json j_out = j_in;


    printf("(Before saving the pose) object_sub_grasping_pose size: %zu\n", j_out["object_sub_grasping_pose"].size());
    printf("(Current pose index) pose_idx: %zu\n", pose_idx);
    size_t pose_now = j_out["object_sub_grasping_pose"].size();
    // if(pose_idx > pose_now) {
    if(1) {
        ss.str("");
        ss << "pose_" << pose_idx;
        j_out["object_sub_grasping_pose"][pose_idx-1][ss.str()] = grasping_pose;
        j_out["object_sub_grasping_pose"][pose_idx-1]["gripper_open_length"] = ui->doubleSpinBox_gripperOpenLength->value();
        j_out["object_sub_grasping_pose"][pose_idx-1]["axis"] = ui->comboBox_subGraspingPoseAxis->currentText().toStdString();
        j_out["object_sub_grasping_pose"][pose_idx-1]["angle"] = ui->doubleSpinBox_subGraspingPoseAngle->value();
        j_out["object_sub_grasping_pose"][pose_idx-1]["pose_num"] = ui->spinBox_subGraspingPoseNum->value();
        j_out["object_sub_grasping_pose"][pose_idx-1]["gripper_tip_index"] = ui->spinBox_gripperTipIndex->value();
    }

    printf("(After saving the pose) object_sub_grasping_pose size: %zu\n", j_out["object_sub_grasping_pose"].size());

    std::ofstream ofs(pose_config_path.str().c_str());
    ofs << j_out.dump(4) << std::endl;


    pose_idx++;
    ui->spinBox_poseIndex->setValue(pose_idx);
}


void MainWindow_gripper::Object_pub() {
    object.header.frame_id = "map";
    object.header.stamp = node_->now();
    object.ns = "Object";
    object.id = 0;
    object.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    object.mesh_resource = "file://" + file_path;
    object.action = visualization_msgs::msg::Marker::ADD;
    object.scale.x = 0.001;
    object.scale.y = 0.001;
    object.scale.z = 0.001;
    object.color.a = 1.0;
    object.color.r = 0.0;
    object.color.g = 0.0;
    object.color.b = 1.0;
    object_pub_->publish(object);
}

void MainWindow_gripper::Frame_pub(){
    visualization_msgs::msg::Marker frame_x, frame_y, frame_z;
    Eigen::Quaterniond q;
    frame.header.frame_id = "map";
    frame.header.stamp = node_->now();
    frame.ns = "frame";
    frame.id = 0;
    frame.type = visualization_msgs::msg::Marker::CUBE;
    frame.action = visualization_msgs::msg::Marker::ADD;
    // frame.scale.x = 0.05;
    // frame.scale.y = 0.007;
    // frame.scale.z = 0.007;
    frame.pose.position.x = 0.001*(ui->doubleSpinBox_x -> value());
    frame.pose.position.y = 0.001*(ui->doubleSpinBox_y -> value());
    frame.pose.position.z = 0.001*(ui->doubleSpinBox_z -> value());
    q = axisAngleToQuaternion(ui->doubleSpinBox_rx -> value(), ui->doubleSpinBox_ry -> value(), ui->doubleSpinBox_rz -> value());
    if(ui->comboBox_subGraspingPoseAxis->currentText().toStdString() == "x"){
        // q = axisAngleToQuaternion(ui->doubleSpinBox_rx -> value(), ui->doubleSpinBox_ry -> value(), ui->doubleSpinBox_rz -> value());
        frame.scale.x = 0.05;
        frame.scale.y = 0.005;
        frame.scale.z = 0.005;
        frame.color.a = 1.0;
        frame.color.r = 1.0;
        frame.color.g = 0.0;
        frame.color.b = 0.0;
        // frame.pose.position.x = frame.pose.position.x + frame.scale.z/2;
    }
    else if(ui->comboBox_subGraspingPoseAxis->currentText().toStdString() == "y"){
        // q = axisAngleToQuaternion(- ui->doubleSpinBox_rx -> value(), ui->doubleSpinBox_ry -> value(), ui->doubleSpinBox_rz -> value() + 90);
        frame.scale.x = 0.005;
        frame.scale.y = 0.05;
        frame.scale.z = 0.005;
        frame.color.a = 1.0;
        frame.color.r = 0.0;
        frame.color.g = 1.0;
        frame.color.b = 0.0;
        // frame.pose.position.y = frame.pose.position.y + frame.scale.z/2;
    }
    else if(ui->comboBox_subGraspingPoseAxis->currentText().toStdString() == "z"){
        // q = axisAngleToQuaternion(ui->doubleSpinBox_rx -> value(), ui->doubleSpinBox_ry -> value() - 90, ui->doubleSpinBox_rz -> value());
        frame.scale.x = 0.005;
        frame.scale.y = 0.005;
        frame.scale.z = 0.05;
        frame.color.a = 1.0;
        frame.color.r = 0.0;
        frame.color.g = 0.0;
        frame.color.b = 1.0;
        // frame.pose.position.z = frame.pose.position.z + frame.scale.z/2;
    }
    frame.pose.orientation.x = q.x();
	frame.pose.orientation.y = q.y();
	frame.pose.orientation.z = q.z();
	frame.pose.orientation.w = q.w();
    // frame_pub_->publish(frame);
}

void MainWindow_gripper::DisplayGrid(){
    manager_->setFixedFrame(QString::fromStdString("map"));
    Grid_ = manager_->createDisplay("rviz_default_plugins/Grid","Grid",true);
	Gripper_ = manager_->createDisplay("rviz_default_plugins/MarkerArray", "Gripper", true);
    Rotation_ = manager_->createDisplay("rviz_default_plugins/PoseArray", "rotation", true);
    Object_ = manager_->createDisplay("rviz_default_plugins/Marker", "object", true);
    assert(Gripper_!=NULL);
    assert(Object_!=NULL);
    assert(Rotation_!=NULL);
    assert(Grid_!=NULL);

    Grid_->subProp("Cell Size")->setValue(1.0 / 10.0f);
    Gripper_->subProp("Topic")->setValue("/marker_array");
    Rotation_->subProp("Topic")->setValue("/pose_array");
    Rotation_->subProp("Shape")->setValue("Arrow (3D)");
    Object_->subProp("Topic")->setValue("/object");
}