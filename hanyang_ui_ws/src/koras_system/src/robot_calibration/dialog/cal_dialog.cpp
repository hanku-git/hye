#include "robot_calibration/dialog/cal_dialog.hpp"
#include "ui_cal_dialog.h"


using json = nlohmann::json;

const double DEGTORAD = M_PI/180.0;
const double RADTODEG = 180.0/M_PI;

#define ROBOT_6DOF 6

cal_dialog::cal_dialog(QMainWindow *parent, QNode *qnode_)
    : QDialog(parent)
    , cal_ui(new Ui::cal_dialog)
{
    qnode_cal = qnode_;
    cal_ui->setupUi(this);
	readRobotParam();

    m_robot_dof = ROBOT_6DOF;

    init_dh_params_a  << cal_ui->lineEdit_init_j1_a  << cal_ui->lineEdit_init_j2_a  << cal_ui->lineEdit_init_j3_a
					  << cal_ui->lineEdit_init_j4_a  << cal_ui->lineEdit_init_j5_a  << cal_ui->lineEdit_init_j6_a << cal_ui->lineEdit_init_j7_a;
    init_dh_params_ap << cal_ui->lineEdit_init_j1_ap << cal_ui->lineEdit_init_j2_ap << cal_ui->lineEdit_init_j3_ap
					  << cal_ui->lineEdit_init_j4_ap << cal_ui->lineEdit_init_j5_ap << cal_ui->lineEdit_init_j6_ap << cal_ui->lineEdit_init_j7_ap;
    init_dh_params_d  << cal_ui->lineEdit_init_j1_d  << cal_ui->lineEdit_init_j2_d  << cal_ui->lineEdit_init_j3_d
					  << cal_ui->lineEdit_init_j4_d  << cal_ui->lineEdit_init_j5_d  << cal_ui->lineEdit_init_j6_d << cal_ui->lineEdit_init_j7_d;
    init_dh_params_q  << cal_ui->lineEdit_init_j1_q  << cal_ui->lineEdit_init_j2_q  << cal_ui->lineEdit_init_j3_q
					  << cal_ui->lineEdit_init_j4_q  << cal_ui->lineEdit_init_j5_q  << cal_ui->lineEdit_init_j6_q << cal_ui->lineEdit_init_j7_q;

    cal_dh_params_a  << cal_ui->lineEdit_cal_j1_a  << cal_ui->lineEdit_cal_j2_a  << cal_ui->lineEdit_cal_j3_a
					 << cal_ui->lineEdit_cal_j4_a  << cal_ui->lineEdit_cal_j5_a  << cal_ui->lineEdit_cal_j6_a << cal_ui->lineEdit_cal_j7_a;
    cal_dh_params_ap << cal_ui->lineEdit_cal_j1_ap << cal_ui->lineEdit_cal_j2_ap << cal_ui->lineEdit_cal_j3_ap
					 << cal_ui->lineEdit_cal_j4_ap << cal_ui->lineEdit_cal_j5_ap << cal_ui->lineEdit_cal_j6_ap << cal_ui->lineEdit_cal_j7_ap;
    cal_dh_params_d  << cal_ui->lineEdit_cal_j1_d  << cal_ui->lineEdit_cal_j2_d  << cal_ui->lineEdit_cal_j3_d
					 << cal_ui->lineEdit_cal_j4_d  << cal_ui->lineEdit_cal_j5_d  << cal_ui->lineEdit_cal_j6_d << cal_ui->lineEdit_cal_j7_d;
    cal_dh_params_q  << cal_ui->lineEdit_cal_j1_q  << cal_ui->lineEdit_cal_j2_q  << cal_ui->lineEdit_cal_j3_q
					 << cal_ui->lineEdit_cal_j4_q  << cal_ui->lineEdit_cal_j5_q  << cal_ui->lineEdit_cal_j6_q << cal_ui->lineEdit_cal_j7_q;

	cal_tcp_params << cal_ui->lineEdit_cal_tcp_1  << cal_ui->lineEdit_cal_tcp_2  << cal_ui->lineEdit_cal_tcp_3
					 << cal_ui->lineEdit_cal_tcp_4  << cal_ui->lineEdit_cal_tcp_5  << cal_ui->lineEdit_cal_tcp_6;

    list_tcp_ << cal_ui->lineEdit_tcp_x_1 << cal_ui->lineEdit_tcp_x_2 << cal_ui->lineEdit_tcp_x_3
              << cal_ui->lineEdit_tcp_x_4 << cal_ui->lineEdit_tcp_x_5 << cal_ui->lineEdit_tcp_x_6;

	list_check_result << cal_ui->lineEdit_cur_x << cal_ui->lineEdit_cur_y << cal_ui->lineEdit_cur_z
					  << cal_ui->lineEdit_cur_r << cal_ui->lineEdit_cur_p << cal_ui->lineEdit_cur_y_2
					  << cal_ui->lineEdit_mes_x << cal_ui->lineEdit_mes_y << cal_ui->lineEdit_mes_z
					  << cal_ui->lineEdit_mes_r << cal_ui->lineEdit_mes_p << cal_ui->lineEdit_mes_y_2
					  << cal_ui->lineEdit_err_x << cal_ui->lineEdit_err_y << cal_ui->lineEdit_err_z
					  << cal_ui->lineEdit_err_r << cal_ui->lineEdit_err_p << cal_ui->lineEdit_err_y_2;

	list_init_pose << cal_ui->lineEdit_init_x << cal_ui->lineEdit_init_y << cal_ui->lineEdit_init_z
				   << cal_ui->lineEdit_init_r << cal_ui->lineEdit_init_p << cal_ui->lineEdit_init_yr;

	list_pose_boundary << cal_ui->lineEdit_bound_x << cal_ui->lineEdit_bound_y << cal_ui->lineEdit_bound_z
				   	   << cal_ui->lineEdit_bound_r << cal_ui->lineEdit_bound_p << cal_ui->lineEdit_bound_yr;

    // // Task planner tab
    // auto taskCallbackFn = [=] (vector<UnitTask> task) {
	// 	ROS_LOG_INFO("# of Tasks: %zu", task.size());
    //     qnode_cal->current_task_list_ = task;
	// 	ROS_LOG_INFO("# of qnode_cal->current_task_list_: %zu", qnode_cal->current_task_list_.size());
    //     qnode_cal->beforeTaskStart();
	// 	ROS_LOG_INFO("# of qnode_cal->current_task_list_: %zu", qnode_cal->current_task_list_.size());
    //     qnode_cal->task_cycle_ = 0;
    // };

    // connect(cal_ui->pushButton_cal_do_task, &QPushButton::clicked,
    //     [=] () {taskCallbackFn(qnode_cal->task_planner_.cal_task_list_);});

    connect(cal_ui->pushButton_cal_open_folder, &QPushButton::clicked, this,
            &cal_dialog::OpenFolderCallback);
    connect(cal_ui->pushButton_task_generate, &QPushButton::clicked, this,
            &cal_dialog::CalJSTaskGenerateCallback);
	connect(cal_ui->pushButton_cal_check, &QPushButton::clicked, this,
            &cal_dialog::CalCheckCallback);
    connect(cal_ui->pushButton_capture_base, &QPushButton::clicked, this,
            &cal_dialog::CalCapBaseCallback);
	connect(cal_ui->pushButton_capture_base_2, &QPushButton::clicked, this,
            &cal_dialog::CalCapBaseCallback);
	connect(cal_ui->pushButton_cal_dh, &QPushButton::clicked, this,
            &cal_dialog::DHcalibrationCallback);
	connect(cal_ui->pushButton_refresh, &QPushButton::clicked, this,
            &cal_dialog::CalCheckResultCallback);

	connect(cal_ui->pushButton_cal_do_task, &QPushButton::clicked, this,
            &cal_dialog::pushButtonDoTaskRobotCalibrationClickedCallback);

	connect(cal_ui->pushButton_cal_CS_generate, &QPushButton::clicked, this,
            &cal_dialog::CalCSTaskGenerateCallback);
	connect(cal_ui->pushButton_cal_CS_do_task, &QPushButton::clicked, this,
            &cal_dialog::pushButtonDoTaskRobotCalibrationClickedCallback);

	connect(cal_ui->pushButton_pose_gen, &QPushButton::clicked, this,
            &cal_dialog::CalPoseGenCallback);
	connect(cal_ui->pushButton_check_col, &QPushButton::clicked, this,
            &cal_dialog::CalCheckCollisonCallback);
	connect(cal_ui->pushButton_get_cur_pose, &QPushButton::clicked, this,
            &cal_dialog::CalGetPoseCallback);
}

cal_dialog::~cal_dialog()
{
	rclcpp::shutdown();
    delete cal_ui;
}

void cal_dialog::testDialog(unsigned int idx) {
    if(idx != 0) {
        qnode_cal->test_cnt_ = idx;
    }
    ROS_LOG_INFO("(after)cal_dialog->qnode_->test_cnt_: %i", qnode_cal->test_cnt_);
}

void cal_dialog::OpenFolderCallback() {
    QString folder_name = QFileDialog::getExistingDirectory(this, tr("Open folder"),"./");
    // printf(filename);
    cal_ui->label_cal_folder_name->setText(folder_name);
}

/********************************************************************************************
***********************                  Pose Generator                  ********************
*********************************************************************************************/

void cal_dialog::readRobotParam() {
    // double dh_param   [JS_DOF][4]; // alpha, a, d, theta

    // double dh_param[JS_DOF][4] = // KR0509 RPR
    // {   { 90,    0,   145, 0},
    //     {  0,  450,     0, 0},
    //     {-90,    0,   -10, 0},
    //     { 90,    0,   385, 0},
    //     {-90, -134,     0, 0},
    //     {  0,    0, 142.5, 0}};

    // double dh_param[JS_DOF][4] = // UR5
    // {   { 90,      0, 89.159, 0},
    //     {  0,   -425,      0, 0},
    //     {  0, -392.2,      0, 0},
    //     { 90,      0, 109.15, 0},
    //     {-90,      0,  94.65, 0},
    //     {  0,      0,   82.3, 0}};

        double dh_param[JS_DOF][4] = // UR10e
    {   { 90,       0,  180.7, 0},
        {  0,  -612.7,      0, 0},
        {  0, -571.55,      0, 0},
        { 90,       0, 174.15, 0},
        {-90,       0, 119.85, 0},
        {  0,       0, 116.55, 0}};

    for (int i = 0; i < JS_DOF; i++) {
        // alpha, a, d, theta
        dh_param[i][0] = dh_param[i][0] * kDeg2Rad; // deg -> rad
        dh_param[i][1] = dh_param[i][1] / 1000;     // mm -> m
        dh_param[i][2] = dh_param[i][2] / 1000;     // mm -> m
        dh_param[i][3] = dh_param[i][3] * kDeg2Rad; // deg -> rad
    }

    robot_model_.setDH(dh_param);
}

void cal_dialog::saveToCSV(const std::vector<std::vector<double>>& data, const std::string& filePath) {
    std::ofstream outputFile(filePath);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open the file " << filePath << std::endl;
        return;
    }
    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            outputFile << std::fixed << row[i];
            if (i < row.size() - 1) {
                outputFile << ",";
            }
        }
        outputFile << std::endl;
    }
	std::cout << "Calibration pose saved: " << filePath << std::endl;
    outputFile.close();
}

void cal_dialog::CalPoseGenCallback() {
	vector<double> init_angle_prev(JS_DOF);
	vector<double> home_pose(CS_DOF);
	vector<double> cube_size(CS_DOF);
	rnd_points.clear();

	num_of_pose = cal_ui->lineEdit_cal_num_of_pose->text().toInt();
	for(int i=0; i<JS_DOF; i++){
		init_angle_prev[i] = qnode_cal->params_.meas.q[i];
	}
	for(int i=0; i<CS_DOF; i++){
		if(i<3) home_pose[i] = list_init_pose[i]->text().toDouble();
		else home_pose[i] = list_init_pose[i]->text().toDouble();
		if(i<3) cube_size[i] = list_pose_boundary[i]->text().toDouble();
		else cube_size[i] = list_pose_boundary[i]->text().toDouble();
	}
    vector<double> init_angle(JS_DOF);
    robot_model_.inverseKinematics(home_pose, init_angle_prev, init_angle);

    for (int i = 0; i < num_of_pose; i++){
        random_device rd;
        mt19937 mt(rd());
        uniform_real_distribution<> dist(-1.0, 1.0);
        auto randNum1 = dist(mt);
        auto randNum2 = dist(mt);
        auto randNum3 = dist(mt);
        auto randNum4 = dist(mt);
        auto randNum5 = dist(mt);
        auto randNum6 = dist(mt);

        double x = home_pose[0] + randNum1 * cube_size[0];
        double y = home_pose[1] + randNum2 * cube_size[1];
        double z = home_pose[2] + randNum3 * cube_size[2];

        double rx = home_pose[3] + randNum4 * cube_size[3];
        double ry = home_pose[4] + randNum5 * cube_size[4];
        double rz = home_pose[5] + randNum6 * cube_size[5];
        // vector<double> init_q = current_q();
        // set_start_pose(init_q);
        // vector<double> des_angle_prev(JS_DOF);

        // for (int i = 0; i < JS_DOF; i++) {
        //     des_angle_prev[i] = des.q[i];
        // }
		std::cout << "Generated pose: " << x << y << z << rx << ry << rz << std::endl;
        vector<double> des_angle(JS_DOF);
        robot_model_.inverseKinematics({x, y, z, rx, ry, rz}, init_angle, des_angle);
        // for (int i = 0; i < JS_DOF; i++) {
        //     des.q[i]   = des_angle[i];
        // }

        // robot_model_.setq(qnode_cal->params_.meas.q[i]);
        rnd_points.push_back(des_angle);
    }
	string folder_name = cal_ui->label_cal_folder_name->text().toStdString();

	std::string filePath = folder_name + "/P_JS_CAL.csv";
    saveToCSV(rnd_points, filePath);
}

void cal_dialog::CalGetPoseCallback() {
	// std::vector<double> current_xpose = qnode_cal->params_.meas.x;
	for (int i=0; i<6; i++){
		list_init_pose[i]->setText(QString::number(qnode_cal->params_.meas.x[i], 'f', 5));
	}
}

void cal_dialog::CalCheckCollisonCallback(){
	return;
}

void cal_dialog::CalJSTaskGenerateCallback() {
    string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
    stringstream fname;
    fname << folder_name << "/P_JS_CAL.csv";
    cout<<"file name: "<<fname.str()<<endl;

    double qd  = cal_ui->lineEdit_qd_target->text().toDouble();
    double qdd = cal_ui->lineEdit_qdd_target->text().toDouble();
    vector<vector<double>> pose_set;
    vector<double> pose;
    string line, deg;
    fstream file (fname.str(), ios::in);
    if(file.is_open()) {
        while(getline(file, line)) {
            pose.clear();
            stringstream str(line);
            while(getline(str, deg, ',')) {
                pose.push_back(stod(deg));
            }
            pose_set.push_back(pose);
        }
    }
    else {
        cout<<"Could not open the file"<<endl;
    }
    qnode_cal->task_planner_.CalGenerationJSTask(pose_set, qd, qdd, folder_name);
    cout<<"Done Generate cal pose"<<endl;
}

void cal_dialog::CalCSTaskGenerateCallback() {
    string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
    stringstream fname;
    // fname << folder_name << "/P_JS_CAL.csv";
	fname << folder_name << "/P_CS_ISO5.csv";
    cout<<"file name: "<<fname.str()<<endl;

    double xd  = cal_ui->lineEdit_xd_target->text().toDouble();
    double xdd = cal_ui->lineEdit_xdd_target->text().toDouble();
    vector<vector<double>> pose_set;
    vector<double> pose;
    string line, deg;
    fstream file (fname.str(), ios::in);
    if(file.is_open()) {
        while(getline(file, line)) {
            pose.clear();
            stringstream str(line);
            while(getline(str, deg, ',')) {
                pose.push_back(stod(deg));
            }
            pose_set.push_back(pose);
        }
    }
    else {
        cout<<"Could not open the file"<<endl;
    }
    qnode_cal->task_planner_.CalGenerationCSTask(pose_set, xd, xdd, folder_name);
    cout<<"Done Generate cal pose"<<endl;
}

void cal_dialog::CalCheckCallback(){
    string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
    stringstream fname;
    cout<<"DH parameter Check"<<endl;

	CsDouble x_current = qnode_cal->params_.meas.x;
    qnode_cal->CalCheckCalibration(folder_name, x_current);
}

void cal_dialog::CalCapBaseCallback(){
    string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
    // int num_of_pose = cal_ui->lineEdit_cal_num_of_pose->text().toInt();
    stringstream fname;
    fname << folder_name << "/robot_cal_scan_base.ply";
    cout<<"file: "<<fname.str()<<endl;
    qnode_cal->CalCapBaseCallback(folder_name);
}

void cal_dialog::CalCheckResultCallback() {
	std::vector<double> cal_check_err = qnode_cal->cal_check_err;
	for (int i=0; i<18; i++){
		list_check_result[i]->setText(QString::number(cal_check_err[i], 'f', 5));
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////Robot DH Calibrator////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


void cal_dialog::LoadDHParameters(){

	//// 241115 이전
    // ifstream file(std::string("/home/") + USER_NAME + "/bp_gui_ws_sim/src/robot_control_config/controller_config/dynamics_param.json");
	//// 241115 경태님
    ifstream file(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_config/controller_config/dynamics_param.json");

    json Config = json::parse(file);

    vector<double> a = Config["dh"]["a"].get<std::vector<double> >();
    vector<double> ap = Config["dh"]["alpha"].get<std::vector<double> >();
    vector<double> d = Config["dh"]["d"].get<std::vector<double> >();
    vector<double> q = Config["dh"]["theta"].get<std::vector<double> >();

    Eigen::MatrixXd dDH_Init(m_robot_dof,4);

    for(int i=0; i<m_robot_dof; i++){
        init_dh_params_a[i]->setText(QString::number(a[i], 'f', 4));
        dDH_Init(i,1) = 0.001*init_dh_params_a[i]->text().toDouble();
        init_dh_params_ap[i]->setText(QString::number(ap[i], 'f', 4));
        dDH_Init(i,0) = DEGTORAD*init_dh_params_ap[i]->text().toDouble();
        init_dh_params_d[i]->setText(QString::number(d[i], 'f', 4));
        dDH_Init(i,2) = 0.001*init_dh_params_d[i]->text().toDouble();
        init_dh_params_q[i]->setText(QString::number(q[i], 'f', 4));
        dDH_Init(i,3) = DEGTORAD*init_dh_params_q[i]->text().toDouble();
	}
    m_dDH_Init = dDH_Init;

	Eigen::VectorXd dDH_threshold(4*m_robot_dof);
	// mm to m
	for(int i=0; i<m_robot_dof; i++){

		dDH_threshold(i+m_robot_dof) = DEGTORAD*10.0; // alpha[rad]
		dDH_threshold(i) = 0.001*50.0; // a[m]
		dDH_threshold(i+2*m_robot_dof) = 0.001*50.0; // d[m]
		dDH_threshold(i+3*m_robot_dof) = DEGTORAD*10.0; // theta[rad]
	}
	m_dDH_Err_threshold = dDH_threshold;
}

void cal_dialog::LoadTCP(){

    for(int i=0; i<6; i++){
		if(i<3) m_TCP[i] = list_tcp_[i]->text().toDouble();
		else m_TCP[i] = DEGTORAD*list_tcp_[i]->text().toDouble();
	}
    m_RobotCal.SetTcp(m_TCP);
	// m_inv_kine.SetTcp(m_TCP);
}

void cal_dialog::LoadMeasuredPosition(){
	string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
    stringstream fname;
    fname << folder_name << "/cal_scan_CS.csv";
    m_iPoseCount = cal_ui->lineEdit_cal_num_of_pose->text().toInt();

    vector<vector<double>> pose_set;
    vector<double> pose;
    string line, deg;
    fstream file (fname.str(), ios::in);
    if(file.is_open()) {
        while(getline(file, line)) {
            pose.clear();
            stringstream str(line);
            while(getline(str, deg, ',')) {
                pose.push_back(stod(deg));
            }
            pose_set.push_back(pose);
        }
    }
    else {
        cout<<"Could not open the file"<<endl;
    }

	Eigen::MatrixXd MeasPose_Cali(m_iPoseCount, m_robot_dof);
    for (int i = 0; i < pose_set.size(); i++)
    {
        for (int j = 0; j < pose_set[i].size(); j++)
        {
            MeasPose_Cali(i, j) = pose_set[i][j];
        }
    }

	for(int i=0; i<m_iPoseCount; i++)
	{
		m_dMeasuredPose[i][0] = MeasPose_Cali(i,0);
		m_dMeasuredPose[i][1] = MeasPose_Cali(i,1);
		m_dMeasuredPose[i][2] = MeasPose_Cali(i,2);
		m_dMeasuredPose[i][3] = MeasPose_Cali(i,3);
		m_dMeasuredPose[i][4] = MeasPose_Cali(i,4);
		m_dMeasuredPose[i][5] = MeasPose_Cali(i,5);
	}
}

void cal_dialog::LoadDesiredJointAngle(){
    string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
    stringstream fname;
    fname << folder_name << "/cal_input_JS.csv";
    m_iPoseCount = cal_ui->lineEdit_cal_num_of_pose->text().toInt();

    vector<vector<double>> pose_set;
    vector<double> pose;
    string line, deg;
    fstream file (fname.str(), ios::in);
    if(file.is_open()) {
        while(getline(file, line)) {
            pose.clear();
            stringstream str(line);
            while(getline(str, deg, ',')) {
                pose.push_back(stod(deg));
            }
            pose_set.push_back(pose);
        }
    }
    else {
        cout<<"Could not open the file"<<endl;
    }

    Eigen::MatrixXd JointAngle_Cali(m_iPoseCount, m_robot_dof);
    for (int i = 0; i < pose_set.size(); i++)
    {
        for (int j = 0; j < pose_set[i].size(); j++)
        {
            JointAngle_Cali(i, j) = pose_set[i][j];
        }
    }

	m_dDesiredAngleCali = JointAngle_Cali;
	for(int i=0; i<m_iPoseCount; i++)
	{
		// m_dDesiredAngleCali(i,0) = m_dDesiredAngleCali(i,0) + RADTODEG*m_dDH_Init(0,3);
		// m_dDesiredAngleCali(i,1) = m_dDesiredAngleCali(i,1) + RADTODEG*m_dDH_Init(1,3);
		// m_dDesiredAngleCali(i,2) = m_dDesiredAngleCali(i,2) + RADTODEG*m_dDH_Init(2,3);
		// m_dDesiredAngleCali(i,3) = m_dDesiredAngleCali(i,3) + RADTODEG*m_dDH_Init(3,3);
		// m_dDesiredAngleCali(i,4) = m_dDesiredAngleCali(i,4) + RADTODEG*m_dDH_Init(4,3);
		// m_dDesiredAngleCali(i,5) = m_dDesiredAngleCali(i,5) + RADTODEG*m_dDH_Init(5,3);
		m_dDesiredAngleCali(i,0) = m_dDesiredAngleCali(i,0);
		m_dDesiredAngleCali(i,1) = m_dDesiredAngleCali(i,1);
		m_dDesiredAngleCali(i,2) = m_dDesiredAngleCali(i,2);
		m_dDesiredAngleCali(i,3) = m_dDesiredAngleCali(i,3);
		m_dDesiredAngleCali(i,4) = m_dDesiredAngleCali(i,4);
		m_dDesiredAngleCali(i,5) = m_dDesiredAngleCali(i,5);
	}

}

void cal_dialog::LoadData(){
	LoadDHParameters();
	LoadTCP();
	LoadDesiredJointAngle();
	LoadMeasuredPosition();
}

void cal_dialog::DHcalibrationCallback(){
	bool is_tool_calibration = cal_ui->radioButton_tool_calibration->isChecked();
	if(cal_ui->radioButton_all->isChecked()){
		RunRobotPoseCalibrationWithTool(101, is_tool_calibration);
	}
	else if(cal_ui->radioButton_without_zero->isChecked()){
		RunRobotPoseCalibrationWithTool(102, is_tool_calibration);
	}
	else if(cal_ui->radioButton_without_alpha->isChecked()){
		RunRobotPoseCalibrationWithTool(103, is_tool_calibration);
	}
	else if(cal_ui->radioButton_without_zero_alpha->isChecked()){
		RunRobotPoseCalibrationWithTool(104, is_tool_calibration);
	}
}


void cal_dialog::RunRobotPoseCalibrationWithTool(unsigned int n_case, bool is_tool_calibration)
{
	m_iIteration = cal_ui->lineEdit_iter->text().toInt();
	double lamda = cal_ui->lineEdit_lamda->text().toDouble(); // least square parameter																			// user parm
	int pose_num = cal_ui->lineEdit_pose_num->text().toInt();																								// ui-> pose_num
	int robot_dof = m_robot_dof;																								// fix parm
	int param_num = 4*m_robot_dof + 6; // Tool 포함
	LoadData();
	printf("lamda: %f\n", lamda);
	printf("pose_num: %i\n", pose_num);
	printf("robot_dof: %i\n", robot_dof);
	printf("param_num: %i\n", param_num);

	//// 1. Initial Position
	Eigen::VectorXd JSDesAngle(robot_dof);
	Eigen::MatrixXd P_Cali(pose_num,6); // Caculated pose from F.K.
	Eigen::MatrixXd P_Init(pose_num,6); // Caculated pose from F.K.
	Eigen::MatrixXd P_Real(pose_num,6); // Measured position by laser tracker
	Eigen::VectorXd Pose_Err(6*pose_num);
	Eigen::MatrixXd CJ_tmp(6, param_num);
	Eigen::MatrixXd CalibrationJacobian(6*pose_num, param_num);

	std::vector<double> dis_err;
	dis_err.resize(pose_num);

	std::vector<double> ori_err;
	ori_err.resize(pose_num);
	int idx_max;
	double err_max, err_mean, err_std, ori_err_max, ori_err_mean, ori_err_std;
	double err_max_before, err_mean_before, ori_err_max_before, ori_err_mean_before;
	double err_max_after, err_mean_after, ori_err_max_after, ori_err_mean_after;

	//// Measured position
	for(int i=0; i<pose_num; i++){
		for(int j=0; j<3; j++)
		{
			P_Real(i,j) = m_dMeasuredPose[i][j];  // [m]
			P_Real(i,j+3) = DEGTORAD*m_dMeasuredPose[i][j+3];  // [rad]
		}
	}

	Eigen::MatrixXd DH_init(robot_dof,4);
	Eigen::MatrixXd DH_Cali(robot_dof,4);
	Eigen::VectorXd TCP_init(6);
	Eigen::VectorXd TCP_Cali(6);

	// Init DH update
	for(int i=0; i<robot_dof; i++){
		for(int j=0; j<4; j++)
		{
			DH_init(i,j) = m_dDH_Init(i,j); 										// ui -> Init DH
			DH_Cali(i,j) = m_dDH_Init(i,j);
		}
	}
	// Init TCP update
	for(int i=0; i<6; i++)
	{
		TCP_init(i) = m_TCP[i]; // [m], [rad]										// ui -> TCP
		TCP_Cali(i) = m_TCP[i]; // [m], [rad]
	}

	double tcp_now[6];
	// Forward Kinematics - initial position
	printf("\n\n*************** Initial CS pose (forward kinematics, (x, y, z) [mm], (Rx, Ry, Rz) [deg]) ****************\n");
	for(int i=0; i<pose_num; i++)
	{
		for(int j=0; j<robot_dof; j++) JSDesAngle[j] = DEGTORAD*m_dDesiredAngleCali(i,j);						// CSV -> after JS

		printf("Des_JS %3i:  %0.3f\t%0.3f\t\t%0.3f\t\t   %0.3f\t\t%0.3f\t\t%0.3f\n", i+1, RADTODEG*JSDesAngle[0], RADTODEG*JSDesAngle[1], RADTODEG*JSDesAngle[2], RADTODEG*JSDesAngle[3], RADTODEG*JSDesAngle[4], RADTODEG*JSDesAngle[5]);

		for (int p = 0; p < 6; p++) tcp_now[p] = TCP_init[p]; // [m], [rad]
		m_RobotCal.SetTcp(tcp_now);
		std::vector<double> pose_tmp = m_RobotCal.forwardKinematics(DH_Cali,JSDesAngle);
		printf("Des_CS %3i:  %0.3f\t%0.3f\t\t%0.3f\t\t   %0.3f\t\t%0.3f\t\t%0.3f\n", i+1, 1000.0*pose_tmp[0], 1000.0*pose_tmp[1], 1000.0*pose_tmp[2], RADTODEG*pose_tmp[3], RADTODEG*pose_tmp[4], RADTODEG*pose_tmp[5]);
		printf("Scan_CS %3i: %0.3f\t%0.3f\t\t%0.3f\t\t   %0.3f\t\t%0.3f\t\t%0.3f\n\n", i+1, 1000.0*P_Real(i, 0), 1000.0*P_Real(i, 1), 1000.0*P_Real(i, 2), RADTODEG*P_Real(i, 3), RADTODEG*P_Real(i, 4), RADTODEG*P_Real(i, 5));



		for(int j=0; j<6; j++) P_Init(i,j) = pose_tmp[j];

		// dis_err
		double tmp1 = P_Real(i,0)-P_Init(i,0);
		double tmp2 = P_Real(i,1)-P_Init(i,1);
		double tmp3 = P_Real(i,2)-P_Init(i,2);
		double err_tmp = sqrt(tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3);
		dis_err[i] = err_tmp;

		// ori_err
		Eigen::Vector3d ori_vec_tmp;
		for (int j = 0; j < 3; j++) ori_vec_tmp[j] = P_Real(i,j+3)-P_Init(i,j+3);

		for (int j = 0; j < 3; j++)
		{
            if (ori_vec_tmp[j] > M_PI) ori_vec_tmp[j] = ori_vec_tmp[j] - 2 * M_PI;
            else if (ori_vec_tmp[j] < -M_PI) ori_vec_tmp[j] = ori_vec_tmp[j] + 2 * M_PI;
        }
		ori_err[i] = ori_vec_tmp.norm();
	}
	printf("*********************************************************************************************\n\n");
	// Initial error view
	// position
	CalMeanStd(dis_err, err_mean, err_std);
	CalMinMax(dis_err, idx_max, err_max, 1); // max
	err_mean_before = err_mean;
	err_max_before = err_max;

	// orientation
	CalMeanStd(ori_err, ori_err_mean, ori_err_std);
	CalMinMax(ori_err, idx_max, ori_err_max, 1); // max
	ori_err_mean_before = ori_err_mean;
	ori_err_max_before = ori_err_max;


	// Initial error view

	printf("\n\n**************************** Error view(Initial) *****************************\n");
	printf("**** Position error between the commanded and measured positions ****\n");
	for (int i=0; i<pose_num; i++) printf("**** Pose %3i - error: %0.3f, orientation error: %0.3f deg\n", i+1, 1000.0*dis_err[i], RADTODEG*ori_err[i]);
	printf("*****************************************\n");
	printf("**** mean(trans.): %0.3f mm, max(trans.): %0.3f mm\n", 1000.0*err_mean, 1000.0*err_max);
	printf("**** mean(ori.): %0.3f deg, max(ori.): %0.3f deg\n", RADTODEG*ori_err_mean, RADTODEG*ori_err_max);
	printf("*****************************************\n\n");

	////////////////////////////////////////////////
	////////// 1. Calculate Calibration Jacobian
	for(int i=0; i<pose_num; i++)
	{
		for(int j=0; j<robot_dof; j++) JSDesAngle[j] = DEGTORAD*m_dDesiredAngleCali(i,j);

		//// Cal Calibration Jacobian,

		m_RobotCal.CalCalibrationJacobianPoseWithTool(n_case, DH_init, JSDesAngle, m_TCP, CJ_tmp, is_tool_calibration); // 6x30(=24+6)

		//std::cout << "CJ_tmp: " << CJ_tmp << std::endl << std::endl;

		for (int j=0; j<param_num; j++) // 24(DH) + 6(TCP)
		{
			CalibrationJacobian(6*i,j) = CJ_tmp(0,j);
			CalibrationJacobian(6*i+1,j) = CJ_tmp(1,j);
			CalibrationJacobian(6*i+2,j) = CJ_tmp(2,j);
			CalibrationJacobian(6*i+3,j) = CJ_tmp(3,j);
			CalibrationJacobian(6*i+4,j) = CJ_tmp(4,j);
			CalibrationJacobian(6*i+5,j) = CJ_tmp(5,j);
		}
	}

	Eigen::VectorXd DH_init_now(param_num); // 30(= 24 + 6)x1
	Eigen::VectorXd DH_cali_now(param_num); // 30(= 24 + 6)x1
	Eigen::VectorXd DH_err_now(param_num);
	for(int i=0; i<robot_dof; i++)
	{
		DH_init_now[i] = DH_init(i,1); // a
		DH_init_now[i+robot_dof] = DH_init(i,0); // alpha
		DH_init_now[i+2*robot_dof] = DH_init(i,2); // d
		DH_init_now[i+3*robot_dof] = DH_init(i,3); // theta
	}

	// TCP
	for(int i=0; i<6; i++)
	{
		DH_init_now[i+4*robot_dof] = TCP_init(i); // idx: 24, 25, ..., 29
	}


	//////////////////////////////////////////////
	// 2. Robot calibration - Least-square(Levenberg & Marquardt)
	// Iteration
	for(int k=0; k<m_iIteration; k++)
	{
		// Update current data
		for(int i=0; i<pose_num; i++)
		{
			// DH theta error compenstation into joint angle
			for(int j=0; j<robot_dof; j++){
				JSDesAngle[j] = DEGTORAD*m_dDesiredAngleCali(i,j) + (DH_Cali(j,3)-m_dDH_Init(j,3));
			}

			for (int p = 0; p < 6; p++) tcp_now[p] = TCP_Cali[p]; // [m], [rad]
			m_RobotCal.SetTcp(tcp_now);
			std::vector<double> pose_tmp = m_RobotCal.forwardKinematics(DH_Cali,JSDesAngle); // [m], [rad]
			//printf("Cali. Pose %3i: %0.3f\t%0.3f\t\t%0.3f\t\t   %0.3f\t\t%0.3f\t\t%0.3f\n", i+1, 1000.0*pose_tmp[0], 1000.0*pose_tmp[1], 1000.0*pose_tmp[2], RADTODEG*pose_tmp[3], RADTODEG*pose_tmp[4], RADTODEG*pose_tmp[5]);

			for(int j=0; j<6; j++) P_Cali(i,j) = pose_tmp[j]; // [m], [rad]

			// Pose error
			Eigen::Vector3d ori_err_cali;
			for(int j=0; j<3; j++)
			{
				// 위치
				Pose_Err[6*i + j] = P_Real(i,j)-P_Cali(i,j); // [m]
				// 방위
				ori_err_cali[j] = P_Real(i,j+3)-P_Cali(i,j+3); // [rad]
				if (ori_err_cali[j] > M_PI) ori_err_cali[j] -= 2 * M_PI; // 방위 검사
				else if (ori_err_cali[j] < -M_PI) ori_err_cali[j] += 2 * M_PI; // 방위 검사
				Pose_Err[6*i + j+3] = ori_err_cali[j]; // [rad]
			}
		}
		//std::cout << "Pose_Err" << Pose_Err << std::endl;

		Eigen::MatrixXd Tmp1 = CalibrationJacobian; // 360 by 24
		Eigen::MatrixXd Tmp2 = CalibrationJacobian.transpose(); // 24 by 360
		Eigen::MatrixXd Tmp3 = Tmp2*Tmp1; // 30 by 30
		Eigen::MatrixXd IdentityMatrix = lamda*Eigen::MatrixXd::Identity(param_num,param_num); // 30 by 30
		Eigen::MatrixXd Tmp4 = Tmp3 + IdentityMatrix;
		Eigen::MatrixXd Tmp4_inv = Tmp4.inverse(); // 30 by 30
		Eigen::MatrixXd Tmp5 = Tmp4_inv*Tmp2; // 30 by 360
		Eigen::VectorXd DH_Err = Tmp5*Pose_Err; // 30 by 1 (a, alpha, d, theta) + (tx, ty, tz, rx, ry, rz)

		//std::cout << "DH_Err" << DH_Err << std::endl;
		// Update DH parameters
		//DH_Err: a, alpha, d, theta
		//DH_Cali: alpha, a, d, theta
		for(int i=0; i<robot_dof; i++)
		{
			DH_Cali(i,0) = DH_Cali(i,0) + DH_Err[robot_dof+i]; // alpha
			DH_Cali(i,1) = DH_Cali(i,1) + DH_Err[i]; // a
			DH_Cali(i,2) = DH_Cali(i,2) + DH_Err[2*robot_dof+i]; // d
			DH_Cali(i,3) = DH_Cali(i,3) + DH_Err[3*robot_dof+i]; // theta

			DH_cali_now[i] = DH_Cali(i,1); // a(m)
			DH_cali_now[i+robot_dof] = DH_Cali(i,0); // alpha(rad)
			DH_cali_now[i+2*robot_dof] = DH_Cali(i,2); // d(m)
			DH_cali_now[i+3*robot_dof] = DH_Cali(i,3); // theta(rad)
		}
		// Update TCP parameters
		// idx: 24, 25, ..., 29
		for(int i=0; i<6; i++)
		{
			TCP_Cali(i) = TCP_Cali(i) + DH_Err[4*robot_dof+i]; // [m], [rad]
			DH_cali_now[i+4*robot_dof] = TCP_Cali(i);
		}

		for(int i=0; i<param_num; i++) DH_err_now[i] = DH_cali_now[i] - DH_init_now[i];

		// // DH errors threshold
		for(int i=0; i<param_num; i++)
		{
			if(i<4*robot_dof) // DH 파라미터에만 적용
			{
				if(fabs(DH_err_now[i]) > m_dDH_Err_threshold[i])
				{
					for(int j=0; j<pose_num*6; j++) CalibrationJacobian(j,i) = 0.0;
				}
			}
		}

		//// Error view
		for(int i=0; i<pose_num; i++)
		{
			for(int j=0; j<robot_dof; j++)
			{
				JSDesAngle[j] = DEGTORAD*m_dDesiredAngleCali(i,j) + (DH_Cali(j,3)-m_dDH_Init(j,3));
			}

			for (int p = 0; p < 6; p++) tcp_now[p] = TCP_Cali[p]; // [m], [rad]
			m_RobotCal.SetTcp(tcp_now);
			std::vector<double> pose_tmp = m_RobotCal.forwardKinematics(DH_Cali,JSDesAngle);
			//printf("Cali. Pose %3i: %0.3f\t%0.3f\t\t%0.3f\t\t   %0.3f\t\t%0.3f\t\t%0.3f\n", i+1, 1000.0*pose_tmp[0], 1000.0*pose_tmp[1], 1000.0*pose_tmp[2], RADTODEG*pose_tmp[3], RADTODEG*pose_tmp[4], RADTODEG*pose_tmp[5]);

			for(int j=0; j<6; j++) P_Cali(i,j) = pose_tmp[j];

			//std::cout << "P_Real" << P_Real.row(i)  << std::endl;
			//std::cout << "P_Cali" << P_Cali.row(i)  << std::endl << std::endl;

			// dis_err
			double tmp1 = P_Real(i,0)-P_Cali(i,0);
			double tmp2 = P_Real(i,1)-P_Cali(i,1);
			double tmp3 = P_Real(i,2)-P_Cali(i,2);
			double err_tmp = sqrt(tmp1*tmp1 + tmp2*tmp2 + tmp3*tmp3);
			dis_err[i] = err_tmp;


			// ori_err
			Eigen::Vector3d ori_vec_tmp;
			for (int j = 0; j < 3; j++) ori_vec_tmp[j] = P_Real(i,j+3)-P_Cali(i,j+3);
			for (int j = 0; j < 3; j++)
			{
				if (ori_vec_tmp[j] > M_PI) ori_vec_tmp[j] = ori_vec_tmp[j] - 2 * M_PI;
				else if (ori_vec_tmp[j] < -M_PI) ori_vec_tmp[j] = ori_vec_tmp[j] + 2 * M_PI;
			}
			ori_err[i] = ori_vec_tmp.norm();
		}
		// Process error view
		// position
		CalMeanStd(dis_err, err_mean_after, err_std);
		CalMinMax(dis_err, idx_max, err_max_after, 1); // max

		// orientation
		CalMeanStd(ori_err, ori_err_mean_after, err_std);
		CalMinMax(ori_err, idx_max, ori_err_max_after, 1); // max

/*
		printf("\n\n**************************** Error view(iter: %i) *****************************\n", k+1);
		printf("**** Position error between the commanded and measured positions ****\n");
		for (int i=0; i<pose_num; i++) printf("**** Pose %3i - error: %0.3f mm\n", i+1, 1000.0*dis_err[i]);
		printf("*****************************************\n");
		printf("**** mean: %0.3f mm, max: %0.3f mm\n", 1000.0*err_mean, 1000.0*err_max);
		printf("*****************************************\n\n");*/


	}

	// Final results
	m_dDH_Calibrated = DH_Cali;
	m_dTCP_Calibrated = TCP_Cali;

	// (m, rad) to (mm, deg)
	for(int i=0; i<robot_dof; i++)
	{
		m_dDH_Calibrated(i,0) = RADTODEG*m_dDH_Calibrated(i,0); // alpha(deg)
		m_dDH_Calibrated(i,1) = 1000.0*m_dDH_Calibrated(i,1); // a(mm)
		m_dDH_Calibrated(i,2) = 1000.0*m_dDH_Calibrated(i,2); // d(mm)
		m_dDH_Calibrated(i,3) = RADTODEG*m_dDH_Calibrated(i,3); // theta(deg)
	}

	for (int i = 0; i < 6; i++)
	{
		if(i<3) m_dTCP_Calibrated[i] = 1000.0*m_dTCP_Calibrated[i]; // [mm]
		else m_dTCP_Calibrated[i] = RADTODEG*m_dTCP_Calibrated[i]; // [deg]
	}

	// Save txt file
	string folder_name = cal_ui->label_cal_folder_name->text().toStdString();
	stringstream dh_config_path;
	json j_out;
	dh_config_path << folder_name << "/Cali_DH_pose.json";

	for(int i=0; i<robot_dof; i++)
		{
			j_out["dh"]["a"][i] = m_dDH_Calibrated(i,1);
			j_out["dh"]["alpha"][i] = m_dDH_Calibrated(i,0);
			j_out["dh"]["d"][i] = m_dDH_Calibrated(i,2);
			j_out["dh"]["theta"][i] = m_dDH_Calibrated(i,3);
		}

	std::ofstream ofs(dh_config_path.str().c_str());
	ofs << j_out.dump(4) <<endl;


	// FILE *f;
	// FILE *f_out;
	// FILE *f_tcp_out;
	// CString str;
	// CString str_output;
	// CString str_tcp_output;
	// switch(n_case)
	// {
	// case 101:
	// 	str.Format(_T("output/Cali_DH(all)_pose.txt"));
	// 	break;
	// case 102:
	// 	str.Format(_T("output/Cali_DH(all_no_zero_change)_pose.txt"));
	// 	break;
	// case 103:
	// 	str.Format(_T("output/Cali_DH(all_no_alpha_change)_pose.txt"));
	// 	break;
	// case 104:
	// 	str.Format(_T("output/Cali_DH(a,d,q)_pose.txt"));
	// 	break;
	// }
	// str_output.Format(_T("output/Cali_DH_results.txt"));
	// str_tcp_output.Format(_T("output/Cali_TCP_results.txt"));

	// f = fopen((CStringA)str, "w");
	// f_out = fopen((CStringA)str_output, "w");
	// f_tcp_out = fopen((CStringA)str_tcp_output, "w");
	// if(f_out == NULL){
	// 	printf("File load(Cali_DH_results) failed!\n");
	// 	return;
	// }

	// for(int i=0; i<robot_dof; i++) // robot_dof by 4
	// {
	// 	fprintf(f, "%f\t%f\t%f\t%f\n", m_dDH_Calibrated(i,1), m_dDH_Calibrated(i,0), m_dDH_Calibrated(i,2), m_dDH_Calibrated(i,3));
	// 	fprintf(f_out, "%f\t%f\t%f\t%f\n", m_dDH_Calibrated(i,1), m_dDH_Calibrated(i,0), m_dDH_Calibrated(i,2), m_dDH_Calibrated(i,3));
	// }

	// if(f_tcp_out == NULL){
	// 	printf("File load(Cali_TCP_results.txt) failed!\n");
	// 	return;
	// }
	// for (int i = 0; i < 6; i++)
	// {
	// 	if(i!=5) fprintf(f_tcp_out, "%f\t", m_dTCP_Calibrated(i));
	// 	else fprintf(f_tcp_out, "%f\n", m_dTCP_Calibrated(i));
	// }

	// fclose(f);
	// fclose(f_out);
	// fclose(f_tcp_out);

	//CString str;
	// for(int i=0; i<robot_dof; i++){


	for(int i=0; i<m_robot_dof; i++){
        cal_dh_params_a[i]->setText(QString::number(m_dDH_Calibrated(i, 1), 'f', 4));
        cal_dh_params_ap[i]->setText(QString::number(m_dDH_Calibrated(i, 0), 'f', 4));
        cal_dh_params_d[i]->setText(QString::number(m_dDH_Calibrated(i, 2), 'f', 4));
        cal_dh_params_q[i]->setText(QString::number(m_dDH_Calibrated(i, 3), 'f', 4));
	}

	for(int i=0; i<3; i++){
        cal_tcp_params[i]->setText(QString::number(m_dTCP_Calibrated(i), 'f', 4));
        cal_tcp_params[i + 3]->setText(QString::number(m_dTCP_Calibrated(i + 3), 'f', 4));
	}

	// 	if(!is_redundant)
	// 	{
	// 		str.Format(_T("%0.2f"), m_dDH_Calibrated(i,1)); // a, mm
	// 		GetDlgItem(1026+i)->SetWindowText(str);
	// 		str.Format(_T("%0.3f"), m_dDH_Calibrated(i,0)); // alpha, deg
	// 		GetDlgItem(1032+i)->SetWindowText(str);
	// 		str.Format(_T("%0.2f"), m_dDH_Calibrated(i,2)); // d, mm
	// 		GetDlgItem(1038+i)->SetWindowText(str);
	// 		str.Format(_T("%0.3f"), m_dDH_Calibrated(i,3)); // theta, deg
	// 		GetDlgItem(1044+i)->SetWindowText(str);
	// 		if(i == robot_dof-1)
	// 		{
	// 			GetDlgItem(IDC_EDIT_DH_INIT77)->SetWindowText(_T("-"));
	// 			GetDlgItem(IDC_EDIT_DH_INIT78)->SetWindowText(_T("-"));
	// 			GetDlgItem(IDC_EDIT_DH_INIT79)->SetWindowText(_T("-"));
	// 			GetDlgItem(IDC_EDIT_DH_INIT80)->SetWindowText(_T("-"));
	// 		}
	// 	}
	// 	// else
	// 	// {
	// 	// 	if(i != robot_dof-1)
	// 	// 	{
	// 	// 		str.Format(_T("%0.2f"), m_dDH_Calibrated(i,1)); // a, mm
	// 	// 		GetDlgItem(1026+i)->SetWindowText(str);
	// 	// 		str.Format(_T("%0.3f"), m_dDH_Calibrated(i,0)); // alpha, deg
	// 	// 		GetDlgItem(1032+i)->SetWindowText(str);
	// 	// 		str.Format(_T("%0.2f"), m_dDH_Calibrated(i,2)); // d, mm
	// 	// 		GetDlgItem(1038+i)->SetWindowText(str);
	// 	// 		str.Format(_T("%0.3f"), m_dDH_Calibrated(i,3)); // theta, deg
	// 	// 		GetDlgItem(1044+i)->SetWindowText(str);
	// 	// 	}
	// 	// 	else
	// 	// 	{
	// 	// 		str.Format(_T("%0.2f"), m_dDH_Calibrated(i,1)); // a, mm
	// 	// 		GetDlgItem(IDC_EDIT_DH_INIT77)->SetWindowText(str);
	// 	// 		str.Format(_T("%0.3f"), m_dDH_Calibrated(i,0)); // alpha, deg
	// 	// 		GetDlgItem(IDC_EDIT_DH_INIT78)->SetWindowText(str);
	// 	// 		str.Format(_T("%0.2f"), m_dDH_Calibrated(i,2)); // d, mm
	// 	// 		GetDlgItem(IDC_EDIT_DH_INIT79)->SetWindowText(str);
	// 	// 		str.Format(_T("%0.3f"), m_dDH_Calibrated(i,3)); // theta, deg
	// 	// 		GetDlgItem(IDC_EDIT_DH_INIT80)->SetWindowText(str);
	// 	// 	}
	// 	// }
	// }

	// // Calibrated TCP
	// for (int i = 0; i < 6; i++)
	// {
	// 	if(i<3) str.Format(_T("%0.2f"), m_dTCP_Calibrated(i)); // mm
	// 	else str.Format(_T("%0.3f"), m_dTCP_Calibrated(i)); // deg
	// 	GetDlgItem(1137+i)->SetWindowText(str);
	// }

	// //
	// str.Format(_T("%0.2f"), 1000.0*err_mean_before); // mm
	cal_ui-> lineEdit_trans_err_mean_before->setText(QString::number(1000.0*err_mean_before, 'f', 2));
	cal_ui-> lineEdit_trans_err_max_before->setText(QString::number(1000.0*err_max_before, 'f', 2));
	cal_ui-> lineEdit_trans_err_mean_after->setText(QString::number(1000.0*err_mean_after, 'f', 2));
	cal_ui-> lineEdit_trans_err_max_after->setText(QString::number(1000.0*err_max_after, 'f', 2));

	cal_ui-> lineEdit_ori_err_mean_before->setText(QString::number(RADTODEG*ori_err_mean_before, 'f', 2));
	cal_ui-> lineEdit_ori_err_max_before->setText(QString::number(RADTODEG*ori_err_max_before, 'f', 2));
	cal_ui-> lineEdit_ori_err_mean_after->setText(QString::number(RADTODEG*ori_err_mean_after, 'f', 2));
	cal_ui-> lineEdit_ori_err_max_after->setText(QString::number(RADTODEG*ori_err_max_after, 'f', 2));

	// GetDlgItem(1072)->SetWindowText(str);
	// str.Format(_T("%0.2f"), 1000.0*err_max_before); // mm
	// GetDlgItem(1073)->SetWindowText(str);
	// str.Format(_T("%0.2f"), 1000.0*err_mean_after); // mm
	// GetDlgItem(1074)->SetWindowText(str);
	// str.Format(_T("%0.2f"), 1000.0*err_max_after); // mm
	// GetDlgItem(1075)->SetWindowText(str);

	// str.Format(_T("%0.2f"), RADTODEG*ori_err_mean_before); // deg
	// GetDlgItem(1131)->SetWindowText(str);
	// str.Format(_T("%0.2f"), RADTODEG*ori_err_max_before); // deg
	// GetDlgItem(1132)->SetWindowText(str);
	// str.Format(_T("%0.2f"), RADTODEG*ori_err_mean_after); // deg
	// GetDlgItem(1133)->SetWindowText(str);
	// str.Format(_T("%0.2f"), RADTODEG*ori_err_max_after); // deg
	// GetDlgItem(1134)->SetWindowText(str);


	printf("\n\n**************************** Error view (Final) *****************************\n");
	printf("**** Position error between the commanded and measured positions ****\n");
	for (int i=0; i<pose_num; i++) printf("**** Pose %3i - error: %0.3f, orientation error: %0.3f deg\n", i+1, 1000.0*dis_err[i], RADTODEG*ori_err[i]);
	printf("*****************************************\n");
	printf("**** mean(trans.): %0.3f mm, max(trans.): %0.3f mm\n", 1000.0*err_mean_after, 1000.0*err_max_after);
	printf("**** mean(ori.): %0.3f deg, max(ori.): %0.3f deg\n", RADTODEG*ori_err_mean_after, RADTODEG*ori_err_max_after);
	printf("*****************************************\n\n");


	switch(n_case)
	{
	case 101:
		printf("Calibration finished! (all DH modified)\n");
		break;
	case 102:
		printf("Calibration finished! (without changing the parameters for zero (a,d))\n");
		break;
	case 103:
		printf("Calibration finished! (without changing the parameters for all alpha)\n");
		break;
	case 104:
		printf("Calibration finished! (without changing the parameters for zero (a,d) and all alpha)\n");
		break;
	}
}

void cal_dialog::CalMinMax(const std::vector<double>& input, int &idx_out, double &dis_out, int b_select)
{
	std::vector<double> vec_in = input;
	if (b_select) // max
	{
		double dis = *std::max_element(vec_in.begin(), vec_in.end());
		std::vector<double>::iterator it_dis = std::max_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
	else // min
	{
		double dis = *std::min_element(vec_in.begin(), vec_in.end());
		std::vector<double>::iterator it_dis = std::min_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
}

void cal_dialog::CalMeanStd(const std::vector<double>& input, double& m, double& std)
{
	double sum = std::accumulate(input.begin(), input.end(), 0.0);
	double mean = sum / input.size();

	std::vector<double> diff(input.size());
	std::transform(input.begin(), input.end(), diff.begin(),
		std::bind2nd(std::minus<double>(), mean));
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stdev = std::sqrt(sq_sum / (input.size() - 1));

	m = mean;
	std = stdev;
}

void cal_dialog::pushButtonDoTaskRobotCalibrationClickedCallback() {
    qnode_cal->current_task_list_ = qnode_cal->task_planner_.cal_task_list_;
	//ROS_LOG_INFO("# of qnode_cal->current_task_list_: %zu", qnode_cal->current_task_list_.size());
    qnode_cal->beforeTaskStart();
	//ROS_LOG_INFO("# of qnode_cal->current_task_list_: %zu", qnode_cal->current_task_list_.size());
    qnode_cal->task_cycle_ = 0;
}