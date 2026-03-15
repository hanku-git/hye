#include <QtWidgets/QApplication>
#include <QtWidgets/QTabWidget>
#include <iostream>
#include <string>
#include <unistd.h>
#include <signal.h>
#include <execinfo.h>
#include <termios.h>
#include <chrono>
#include <thread>

#include "signature.h"
#include "ssd_binding.h"
#include "serial_number.h"

#include "mainwindow_custom.hpp"
#include "mainwindow_node.hpp"
#include "mainwindow_gripper.hpp"
#include "mainwindow_control.hpp"
#include "mainwindow_initial_setup.hpp"
#include "mainwindow_widgets_monitoring.hpp"
#include "mainwindow_widgets_hyundai_llm.hpp"
#include "mainwindow_widgets_koras_cooking.hpp"
#include "mainwindow_widgets_hanyang_eng.hpp"
#include "mainwindow_widgets_camera_view.hpp"
// #include "mainwindow_cook_server.hpp"
#include "mainwindow_cooking_aw.hpp"

#include "mainwindow_widgets_launch_package.hpp"
#include "mainwindow_widgets_hanyang_launch_package.hpp"
#include "mainwindow_widgets_robot_handling.hpp"



#include "ui_mainwindow.h"
#include "qnode.hpp"


#include "toggle_widget.hpp"

#include "login_widget.hpp"
#include "sqliteHelper.hpp"

#include "taskPlannerWidget.hpp"
// #include "food_ai_dialog.hpp"
//#include "food_tag_dialog.hpp"

#include <signal.h>

#include <QUiLoader>
#include <QFile>
#include <QPushButton>

#define COUT_RESET   "\033[0m"   // 색상 초기화
#define COUT_BOLD    "\033[1m"   // 굵게
#define COUT_RED     "\033[1;31m"
#define COUT_GREEN   "\033[1;32m"
#define COUT_YELLOW  "\033[1;33m"
#define COUT_BLUE    "\033[1;34m"
#define COUT_BLUE_GRREN    "\033[1;36m"


#define COMPILER_NAME __cplusplus
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

static std::shared_ptr<NodeDelegateModelRegistry> registerDataModels()
{
    auto ret = std::make_shared<NodeDelegateModelRegistry>();

    //KCR 관련 커맨드들을 모두 템플릿화 <KCRCommandModel, KCR::commandname(enum)> ();
    ret->registerModel<TaskStartModel>("Task");

    ret->registerModel<JsTargetModel>("Robot Move Task");
    ret->registerModel<CsTargetModel>("Robot Move Task");
    ret->registerModel<CsTargetToolFrameModel>("Robot Move Task");
    ret->registerModel<CsTargetRedundantModel>("Robot Move Task");
    ret->registerModel<CsTargetTaskRecogModel>("Robot Move Task");
    ret->registerModel<TCPMoveModel>("Robot Move Task");
    ret->registerModel<MoveBlendingModel>("Robot Move Task");
    ret->registerModel<CSAccVelUpdateModel>("Robot Move Task");

    ret->registerModel<SetTeachingTargetPoseModel>("Robot Teaching");
    ret->registerModel<CsTargetBaseFrameTeachingTargetPoseModel>("Robot Teaching");

    // ret->registerModel<ScanModel>("Bin Picking Task");
    // ret->registerModel<TemplateMatchingModel>("Bin Picking Task");

    ret->registerModel<SetTCPModel>("Bin Picking Task");
    ret->registerModel<BP_SelectTCPModel>("Bin Picking Task");
    ret->registerModel<BP_PLCModel>("Bin Picking Task");
    ret->registerModel<BP_3DScanModel>("Bin Picking Task");
    ret->registerModel<BP_TemplateMatchingModel>("Bin Picking Task");
    ret->registerModel<BP_ScanTemplateMatchingModel>("Bin Picking Task");

    ret->registerModel<BP_GripperRelayCmdModel>("Gripper Task");
    ret->registerModel<BP_Recog_KORASGripperModel>("Gripper Task");
    ret->registerModel<BP_setGripperDriverModel>("Gripper Task");
    ret->registerModel<BP_KORASGripperModel>("Gripper Task");
    ret->registerModel<BP_DoGraspingModel>("Gripper Task");
    ret->registerModel<BP_DoSubGraspingModel>("Gripper Task");

    ret->registerModel<InitalizeGripperModel>("Gripper Task");

    ret->registerModel<BP_CountDetachingPose>("STACKING Task");
    ret->registerModel<StackingTaskModel>("STACKING Task");

    ret->registerModel<DelayModel>("Task");
    ret->registerModel<PauseModel>("Task");
    ret->registerModel<SavePointModel>("Task");
    ret->registerModel<RewindModel>("Task");
    ret->registerModel<TaskConfirmModel>("Task");

    // // ret->registerModel<ImpedanceOnModel>("Impedance Task");
    // // ret->registerModel<ImpedanceOffModel>("Impedance Task");
    ret->registerModel<IrlGrpModel>("Irl Gripper Task");
    ret->registerModel<IrlGrpCmdModel>("Irl Gripper Task");
    ret->registerModel<IrlGrpInitModel>("Irl Gripper Task");

    ret->registerModel<PLC_MODBUS_TCP_READ_COIL>("PLC_Command");
    ret->registerModel<PLC_MODBUS_TCP_WriteModel>("PLC_Command");
    ret->registerModel<PLC_MODBUS_TCP_ReadModel>("PLC_MODBUS_TCP_ReadModel");

    ret->registerModel<SetTipIndexTaskModel>("Tip Changing");
    ret->registerModel<TipChangingSetTcpModel>("Tip Changing");
    ret->registerModel<TipChangingDefaultAttachModel>("Tip Changing");
    ret->registerModel<TipChangingDefaultDetachModel>("Tip Changing");
    ret->registerModel<TipChangingAttachModel>("Tip Changing");
    ret->registerModel<TipChangingDetachModel>("Tip Changing");

    ret->registerModel<AprilTagDetectionTaskModel>("TAG");
    ret->registerModel<MoveToTagTaskModel>("TAG");
//related to PLC // + connection specific to PLC communication
//    ret->registerModel<MotionModel>("PLC"); ip, port , slave id , 4가지 함수중 선택

    return ret;
}

void setStdinEcho(bool enable) {
    struct termios tty;
    tcgetattr(STDIN_FILENO, &tty);
    if (!enable)
        tty.c_lflag &= ~ECHO;
    else
        tty.c_lflag |= ECHO;

    (void)tcsetattr(STDIN_FILENO, TCSANOW, &tty);
}

bool waitForUserInput() {
    setStdinEcho(false);  // 입력 숨기기

    std::string userInput;
    auto start = std::chrono::high_resolution_clock::now();

    while (true) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start).count() > 10) {
            break;
        }

        if (std::cin.peek() != EOF) {
            std::getline(std::cin, userInput);
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    setStdinEcho(true);  // 입력 표시

    return userInput == "skalsdn";
}

// 👇 전역 포인터 정의 (UI 종료 시 접근용)
MainWindow_widgetsLaunchPackage* g_mainWindowLaunchPackage = nullptr;
MainWindow_widgetsHanyangLaunchPackage* g_mainWindowHanyangLaunchPackage = nullptr;

// 👇 시그널 핸들러 함수 정의
void signalHandler(int signum)
{
    qDebug() << "[SIGNAL] Interrupt signal (" << signum << ") received.";

    if (g_mainWindowLaunchPackage) {
        qDebug() << "[SIGNAL] Terminating QProcess and ROS nodes...";
        g_mainWindowLaunchPackage->terminateQProcessAndRosNode();  // ⬅︎ 수동 종료 함수
    }

    if (g_mainWindowHanyangLaunchPackage) {
        qDebug() << "[SIGNAL] Terminating Hanyang QProcess and ROS nodes...";
        g_mainWindowHanyangLaunchPackage->terminateQProcessAndRosNode();  // ⬅︎ 수동 종료 함수
    }

    rclcpp::shutdown();  // ROS 종료
    std::exit(signum);   // 프로세스 종료
}



void segfault_handler(int sig) {
    void *array[10];
    size_t size = backtrace(array, 10);
    fprintf(stderr, "Caught signal %d\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    _exit(1);
}

// SW 전역 변수 정의 (기본값: false)
bool SW_MODE_DEFAULT = false;
bool SW_MODE_HD_LLM = false;
bool SW_MODE_COOKING = false;
bool SW_MODE_HANYANG_ENG = false;
bool SW_MODE_HANYANG_LAUNCH = false;
bool SW_MODE_GRAPHY = false;

bool GRIPPER_MODE_KORAS = true;
bool GRIPPER_MODE_FAIRINO = false;

std::string USER_NAME = "";


int main(int argc, char** argv)
{
#ifdef _MSC_VER
  //std::cout << "The compiler is MSVC." << std::endl;
#else
  //std::cout << "The compiler is not MSVC." << std::endl;
#endif
    signal(SIGSEGV, segfault_handler);

    const char* username = getenv("USER");  // Linux/macOS
    USER_NAME = username;
    std::cout << "Current user: " << USER_NAME << std::endl;

    // 환경 변수에서 경로를 가져옴
    const char* dataPathEnv = std::getenv("SECUREAPP_DATA_PATH");
    const char* signaturePathEnv = std::getenv("SECUREAPP_SIGNATURE_PATH");
    const char* publicKeyPathEnv = std::getenv("SECUREAPP_PUBLIC_KEY_PATH");

    // 기본 경로 설정
    std::string buildPath = "./"; // 빌드 후 테스트를 위한 상대 경로

    // 경로가 설정되지 않았을 경우 기본값 설정
    std::string dataPath = dataPathEnv ? dataPathEnv : buildPath + "kr_sys";
    std::string signaturePath = signaturePathEnv ? signaturePathEnv : buildPath + "example.sig";
    std::string publicKeyPath = publicKeyPathEnv ? publicKeyPathEnv : buildPath + "public.pem";

    // std::cout << "Data Path: " << dataPath << std::endl;
    // std::cout << "Signature Path: " << signaturePath << std::endl;
    // std::cout << "Public Key Path: " << publicKeyPath << std::endl;
    std::cout << getSSDSerialNumber(getSSDDevice()) << std::endl;
    //bool overrideAccepted = false;
    // 코드 서명 검증
    // if (!verifySignature(dataPath, signaturePath, publicKeyPath)) {
    //     std::cerr << "Signature verification failed." << std::endl;
    //     // if (!waitForUserInput()) {
    //     //     setStdinEcho(true);
    //     return 1;
    //     // } else {
    //     //     overrideAccepted = true;
    //     // }
    // }
    // // SSD 하드웨어 바인딩 검증
    // //if (!overrideAccepted && !verifyHardwareBinding(expected_serial)) {
    // if (!verifyHardwareBinding(expected_serial)) {
    //     std::cerr << "Hardware verification failed. This program is not licensed to run on this machine." << std::endl;
    //     // if (!waitForUserInput()) {
    //     //     setStdinEcho(true);
    //         return 1;
    //     //}
    // }
    // //setStdinEcho(true);
    //std::cout << "Program execution authorized." << std::endl;

    signal(SIGINT, signalHandler);  // Ctrl+C (SIGINT) 발생 시 처리
    signal(SIGTERM, signalHandler); // 종료 요청 (kill 시 등)


    QCoreApplication::setAttribute(Qt::AA_UseSoftwareOpenGL);

    QApplication app(argc, argv);

    // LoginWidget* login_widget = new LoginWidget();
    // login_widget->setWindowModality(Qt::ApplicationModal);
    // login_widget->show();
    // app.exec();

    //  if (login_widget->isLoginSuccessful()) {
    //if (true) {

    std::unordered_map<std::string, int> argumentFlags = {
        {"HD_LLM", 1},
        {"COOKING", 2},
        {"HANYANG_ENG", 3},
        {"HANYANG_LAUNCH", 4}
    };
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////
    //// S/W Argument check

    int sw_flag = 0; // 기본값 (알 수 없는 argument)
    if (argc > 1) {
        std::string argument = argv[1];
        auto it = argumentFlags.find(argument);
        if (it != argumentFlags.end()) {
            sw_flag = it->second;
        }
    }

    if(sw_flag == 0 || !(argc > 1)) {
        std::cout << COUT_BLUE_GRREN;
        std::cout << "*************************************************************************" << std::endl;
        std::cout << "************************** KORAS SYSTEM S/W *****************************" << std::endl;
        std::cout << "*************************************************************************" << std::endl;
        std::cout << COUT_RESET; // 색상 초기화
        std::cout << COUT_RED; // 기본 출력 색상을 노란색으로 설정
        if(!(argc > 1)) {
            std::cout << "No argument provided." << std::endl;
        } else {
            std::cout << "Unavailable Arguments!" << std::endl;
        }
        std::cout << COUT_RESET; // 색상 초기화
        std::cout << COUT_YELLOW; // 기본 출력 색상을 노란색으로 설정
        std::cout << "Please enter the argument for the software you want to use.\n"
                << "Ex) ros2 run hanyang_eng_koras_system kr_sys " << COUT_BLUE << COUT_BOLD << "COOKING" << COUT_RESET << COUT_YELLOW << std::endl
                << "[Available Arguments] 1: "
                << COUT_GREEN << COUT_BOLD << "HD_LLM" << COUT_RESET << COUT_YELLOW << ", 2: "
                << COUT_BLUE << COUT_BOLD << "COOKING" << COUT_RESET << COUT_YELLOW << ", 3: "
                << COUT_GREEN << COUT_BOLD << "HANYANG_ENG" << COUT_RESET << COUT_YELLOW << ", 4: "
                << COUT_GREEN << COUT_BOLD << "HANYANG_LAUNCH" << COUT_RESET << COUT_YELLOW << std::endl;

        std::cout << COUT_RESET; // 색상 초기화

        std::cout << COUT_BLUE_GRREN;
        std::cout << "*************************************************************************" << std::endl;
        std::cout << "*************************************************************************" << std::endl;
        std::cout << "*************************************************************************" << std::endl;
        std::cout << COUT_RESET; // 색상 초기화
        return 0;
    }

    /////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////





    // 소프트웨어 실행 인자 확인
    std::string sw_argument = (argc > 1) ? argv[1] : "";

    // 런타임 전역 변수 설정
    if (sw_argument == "HD_LLM") {
        SW_MODE_HD_LLM = true;
        std::cout << "SW_MODE_HD_LLM: TRUE" << std::endl;
    } else if (sw_argument == "COOKING") {
        SW_MODE_COOKING = true;
        std::cout << "SW_MODE_COOKING: TRUE" << std::endl;
    } else if (sw_argument == "HANYANG_ENG") {
        SW_MODE_HANYANG_ENG = true;
        std::cout << "SW_MODE_HANYANG_ENG: TRUE" << std::endl;
    } else if (sw_argument == "HANYANG_LAUNCH") {
        SW_MODE_HANYANG_LAUNCH = true;
        std::cout << "SW_MODE_HANYANG_LAUNCH: TRUE" << std::endl;
    } else {
        SW_MODE_DEFAULT = true;
        std::cout << "SW_MODE_DEFAULT: TRUE" << std::endl;
    }

    // QNode 인스턴스 생성 시 argument 전달
    // QNode* qnode = &QNode::getInstance(argc, argv);
    QNode* qnode = &QNode::getInstance(argc, argv, sw_argument);

    // 기존
    // QMainWindow* mainWindow = new QMainWindow();
    // 변경 (closeEvent추가)
    MainWindow_custom* mainWindow = new MainWindow_custom();

    Ui::MainWindow *ui = new Ui::MainWindow();
    ui->setupUi(mainWindow);

    //add custom widget -> simulation toggle widget
    ToggleWidget* toggleWidget = new ToggleWidget();
    ui->horizontalLayout_simul->addWidget(toggleWidget);

    QObject::connect(toggleWidget, &ToggleWidget::checked, [&ui, toggleWidget, qnode] () {
        if(toggleWidget->isChecked()) {
            ui->is_simulation->setText("ON");
            qnode->setTaskMode(false);
        } else {
            ui->is_simulation->setText("OFF");
            qnode->setTaskMode(true);
        }
    });



    MainWindow_node *taskWindow = new MainWindow_node(registerDataModels(), qnode, mainWindow);

    MainWindow_control* controlWindow = new MainWindow_control(argc, argv, qnode, mainWindow);
    controlWindow->setWindowModality(Qt::NonModal);

    
    MainWindow_widgetsLaunchPackage *launchPackageWidget = nullptr;
    if (!SW_MODE_HANYANG_LAUNCH) {
        launchPackageWidget = new MainWindow_widgetsLaunchPackage(taskWindow, mainWindow);
        // ui 객체 주입
        launchPackageWidget->setStatusIconLabel("robot", ui->label_robotStatusIcon);
        launchPackageWidget->setStatusIconLabel("gripper", ui->label_gripperStatusIcon);
        launchPackageWidget->setStatusIconLabel("3d scanner", ui->label_3DScannerStatusIcon);
        launchPackageWidget->setStatusIconLabel("mask detection", ui->label_maskDetectionStatusIcon);
        launchPackageWidget->setStatusIconLabel("rviz view", ui->label_rvizViewStatusIcon);
        launchPackageWidget->setStatusIconLabel("scan node", ui->label_scanNodeStatusIcon);
        launchPackageWidget->setStatusIconLabel("matching node", ui->label_matchingNodeStatusIcon);
        launchPackageWidget->initializeDialog(); // Status Initialize
        g_mainWindowLaunchPackage = launchPackageWidget;
    }

    // 새로운 Hanyang Launch Package 생성
    MainWindow_widgetsHanyangLaunchPackage *hanyangLaunchPackageWidget = nullptr;
    if (SW_MODE_HANYANG_LAUNCH) {
        hanyangLaunchPackageWidget = new MainWindow_widgetsHanyangLaunchPackage(taskWindow, mainWindow);
        // ui 객체 주입 (기존과 동일한 UI 사용)
        hanyangLaunchPackageWidget->setStatusIconLabel("robot", ui->label_robotStatusIcon);
        hanyangLaunchPackageWidget->setStatusIconLabel("gripper", ui->label_gripperStatusIcon);
        hanyangLaunchPackageWidget->setStatusIconLabel("3d scanner", ui->label_3DScannerStatusIcon);
        hanyangLaunchPackageWidget->setStatusIconLabel("mask detection", ui->label_maskDetectionStatusIcon);
        hanyangLaunchPackageWidget->setStatusIconLabel("rviz view", ui->label_rvizViewStatusIcon);
        hanyangLaunchPackageWidget->setStatusIconLabel("scan node", ui->label_scanNodeStatusIcon);
        hanyangLaunchPackageWidget->setStatusIconLabel("matching node", ui->label_matchingNodeStatusIcon);
        hanyangLaunchPackageWidget->initializeDialog(); // Status Initialize
        g_mainWindowHanyangLaunchPackage = hanyangLaunchPackageWidget;
    }


    // UI를 동적으로 생성하여 초기 화면을 설정
    QWidget* selectedUI = nullptr;
    // argv 인자를 확인하여 적절한 UI를 생성
    if (argc > 1) {
        std::string argument = argv[1];
        if (argument == "HD_LLM") {
            selectedUI = new MainWindow_widgetsHyundaiLlm(taskWindow, mainWindow);
            qnode->executeHyundaiLlmTask();
            std::cout << "Starting Hyundai LLM UI" << std::endl;
        } else if (argument == "COOKING") {
            selectedUI = new MainWindow_widgetsKorasCooking(taskWindow, mainWindow);
            qnode->executeCookingTask();
            std::cout << "Starting COOKING UI" << std::endl;
        } else if (argument == "HANYANG_ENG") {
            selectedUI = new MainWindow_widgetsHanyangEng(taskWindow, mainWindow);
            qnode->executeHanyangEngTask();
            std::cout << "[HANYANG_ENG] Starting HANYANG_ENG UI" << std::endl;
        } else if (argument == "HANYANG_LAUNCH") {
            selectedUI = new MainWindow_widgetsHanyangEng(taskWindow, mainWindow);
            qnode->executeHanyangEngTask();
            std::cout << "[HANYANG_LAUNCH] Starting HANYANG_ENG UI" << std::endl;
        } else {
            selectedUI = new MainWindow_widgetsMonitoring(taskWindow, mainWindow);
            std::cout << "No argument in the UI lists. Defaulting to HD_LLM UI. // [UI lists] 1. HD_LLM, 2: COOKING, 3: ..." << std::endl;
        }
    } else {
        // 기본값: HD_LLM UI
        selectedUI = new MainWindow_widgetsMonitoring(taskWindow, mainWindow);
        std::cout << "No argument provided. Defaulting to HD_LLM UI. // [UI lists] 1. HD_LLM, 2: COOKING, 3: ..." << std::endl;
    }

    // UI가 선택되었을 경우, 초기 화면으로 표시
    if (selectedUI) {
        selectedUI->setWindowModality(Qt::NonModal);
     

        if(0) { // 기존 이 창만 띄우기
            QObject::connect(ui->pushButton_taskSystemUI, &QPushButton::clicked, [&]() {
                selectedUI->show();
            });
            Q_EMIT ui->pushButton_taskSystemUI->clicked(true);
        }

        if (auto hyundaiLlmUI = dynamic_cast<MainWindow_widgetsHyundaiLlm*>(selectedUI)) {
            QObject::connect(hyundaiLlmUI, &MainWindow_widgetsHyundaiLlm::openIRLDeveloperWindow, [&]() {
                controlWindow->setVisible(true);
            });
        } else if (auto cookingUI = dynamic_cast<MainWindow_widgetsKorasCooking*>(selectedUI)) {
            QObject::connect(cookingUI, &MainWindow_widgetsKorasCooking::openIRLDeveloperWindow, [&]() {
                controlWindow->setVisible(true);
            });
        } else if (auto hanyangEngUI = dynamic_cast<MainWindow_widgetsHanyangEng*>(selectedUI)) {
            QObject::connect(hanyangEngUI, &MainWindow_widgetsHanyangEng::openIRLDeveloperWindow, [&]() {
                controlWindow->setVisible(true);
            });
        } else if (auto monitoringUI = dynamic_cast<MainWindow_widgetsMonitoring*>(selectedUI)) {
            QObject::connect(monitoringUI, &MainWindow_widgetsMonitoring::openIRLDeveloperWindow, [&]() {
                controlWindow->setVisible(true);
            });
        }
    }

    QObject::connect(controlWindow, &MainWindow_control::closeIRLDeveloperWindow, [&ui, taskWindow, controlWindow] () {
        controlWindow->setVisible(false);
    });

    MainWindow_widgetsRobotHandling *robotHandlingWidget = new MainWindow_widgetsRobotHandling(qnode, mainWindow);
    MainWindow_initialSetup *initialSetupWindow = new MainWindow_initialSetup(qnode, mainWindow);

    initialSetupWindow->setWindowModality(Qt::NonModal);

    // MainWindow_initialSetup *initialSetupWindow = new MainWindow_initialSetup(qnode);

    //Initialize the VisualizationFrame
    //auto ros_node_abs = qnode->ros_node_abs_;
    //gripper window

    auto ros_node_abs = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");
    rviz_common::VisualizationFrame* frame = new rviz_common::VisualizationFrame(ros_node_abs);

    frame->setApp(&app);
    // frame->initialize(ros_node_abs, nullptr);

    QString configFilePath = QString("/home/%1/.rviz2/default_config.rviz").arg(USER_NAME.c_str());
    try {
        frame->initialize(ros_node_abs, configFilePath.toStdString().c_str());
    } catch (const std::exception& e) {
        std::cerr << "[RVIZ INIT] Exception: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "[RVIZ INIT] Unknown exception during RViz initialization" << std::endl;
        return -1;
    }


    MainWindow_gripper *gripperWindow = new MainWindow_gripper(frame, ros_node_abs);


    ////////////////////////////////////////////////////////////
    //// [1] Launch Program Widget
    if (launchPackageWidget) {
        ui->stackedWidget->addWidget(launchPackageWidget);
    }
    if (hanyangLaunchPackageWidget) {
        ui->stackedWidget->addWidget(hanyangLaunchPackageWidget);
    }
    //// [2] Task GUI
    ui->stackedWidget->addWidget(selectedUI);
    //// [3] Robot Handling
    ui->stackedWidget->addWidget(robotHandlingWidget);  // index 예: 6
    ////
    //// [4] Gripper GUI
    ui->stackedWidget->addWidget(gripperWindow);
    //// [5] Settings
    ui->stackedWidget->addWidget(taskWindow);
    //// [6] Task Visualized Widget
    ui->stackedWidget->addWidget(taskWindow);
    ////



    ////
    ////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////
    ui->stackedWidget->setCurrentIndex(0);

    QObject::connect(ui->pushButton_launchPackage, &QPushButton::clicked, [&ui] () {
        ui->stackedWidget->setCurrentIndex(0);
    });

    QObject::connect(ui->pushButton_taskSystemUI, &QPushButton::clicked, [&ui] () {
        ui->stackedWidget->setCurrentIndex(1);
    });

    QObject::connect(ui->pushButton_robotHandling, &QPushButton::clicked, [&ui] () {
        ui->stackedWidget->setCurrentIndex(2);
    });

    QObject::connect(ui->pushButton_gripperVisualize, &QPushButton::clicked, [&ui] () {
        ui->stackedWidget->setCurrentIndex(3);
    });

    QObject::connect(ui->pushButton_setting, &QPushButton::clicked, [&ui] () {
        ui->stackedWidget->setCurrentIndex(4);
    });

    QObject::connect(ui->pushButton_taskPlannerGuiBased, &QPushButton::clicked, [&ui] () {
        ui->stackedWidget->setCurrentIndex(5);
    });

    QObject::connect(ui->pushButton_taskPlannerGuiBasedCheckAndPlay, &QPushButton::clicked, [&taskWindow] () {
        if(taskWindow->getTaskManager()->isVisible()) {
            taskWindow->getTaskManager()->activateWindow();
        } else {
            taskWindow->getTaskManager()->GenTaskScriptFromGraph();
            taskWindow->getTaskManager()->show();
        }
    });
    QObject::connect(launchPackageWidget, &MainWindow_widgetsLaunchPackage::openDeveloperWindow, [&ui] () {
        ui->stackedWidget->setCurrentIndex(2);  // Robot Handling UI로 전환
    });
    QObject::connect(hanyangLaunchPackageWidget, &MainWindow_widgetsHanyangLaunchPackage::openDeveloperWindow, [&ui] () {
        ui->stackedWidget->setCurrentIndex(2);  // Robot Handling UI로 전환
    });
    ////////////////////////////////////////////////////////////



// //########################################for non logo visual version
//   auto ros_node_abs2 =
//     std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node2");
//   auto w_robot_state = new Widget_robot_state(&app, ros_node_abs2);
//   //w_robot_state->show();
//     ui->stackedWidget->addWidget(w_robot_state);

// //#######################################
//## w_robot_state
    // ui->pushButton_taskSystemUI->hide();
    // ui->picking_btn_widgetsMonitoring->hide();

    // ui->icon_only_menu->hide();


    // MainWindow_initialSetup *initialSetupWindow = new MainWindow_initialSetup(qnode, mainWindow);

    // initialSetupWindow->setWindowModality(Qt::NonModal);

    // QObject::connect(ui->pushButton_setting, &QPushButton::clicked, [&ui, taskWindow, mainWindow, initialSetupWindow] () {
    //     // ui->stackedWidget->setCurrentIndex(1);
    //     if(taskWindow->getTaskManager()->_qnode->do_open_developer_window_) {
    //         initialSetupWindow->show();
    //     }
    // });

    // UI가 선택되었을 경우
    // 선택된 UI가 MainWindow_widgetsMonitoring 또는 MainWindow_widgetsKorasCooking connect 실행
    if (selectedUI) {
        if (auto hyundaiLlmUI = dynamic_cast<MainWindow_widgetsHyundaiLlm*>(selectedUI)) {
            QObject::connect(hyundaiLlmUI, &MainWindow_widgetsHyundaiLlm::openDeveloperWindow, [&ui, taskWindow, initialSetupWindow] () {
                    taskWindow->getTaskManager()->_qnode->is_developer_window_open_ = true;
                    initialSetupWindow->setVisible(true);
            });
        }
        else if (auto cookingUI = dynamic_cast<MainWindow_widgetsKorasCooking*>(selectedUI)) {
            QObject::connect(cookingUI, &MainWindow_widgetsKorasCooking::openDeveloperWindow, [&ui, taskWindow, initialSetupWindow] () {
                    taskWindow->getTaskManager()->_qnode->is_developer_window_open_ = true;
                    initialSetupWindow->setVisible(true);
            });
        }
        else if (auto hanyangEngUI = dynamic_cast<MainWindow_widgetsHanyangEng*>(selectedUI)) {
            QObject::connect(hanyangEngUI, &MainWindow_widgetsHanyangEng::openDeveloperWindow, [&ui, taskWindow, initialSetupWindow] () {
                    taskWindow->getTaskManager()->_qnode->is_developer_window_open_ = true;
                    initialSetupWindow->setVisible(true);
            });
        }
        else if (auto monitoringUI = dynamic_cast<MainWindow_widgetsMonitoring*>(selectedUI)) {
            QObject::connect(monitoringUI, &MainWindow_widgetsMonitoring::openDeveloperWindow, [&ui, taskWindow, initialSetupWindow] () {
                    taskWindow->getTaskManager()->_qnode->is_developer_window_open_ = true;
                    initialSetupWindow->setVisible(true);
            });
        }
    }




    QObject::connect(initialSetupWindow, &MainWindow_initialSetup::closeDeveloperWindow, [&ui, taskWindow, initialSetupWindow] () {
        initialSetupWindow->setVisible(false);
    });




    //////////////////////////////////////
    // QProcess를 사용하여 Python 애플리케이션 실행
    if (selectedUI) {
        if (auto hyundaiLlmUI = dynamic_cast<MainWindow_widgetsHyundaiLlm*>(selectedUI)) {
            //// TODO:

        } else if (auto cookingUI = dynamic_cast<MainWindow_widgetsKorasCooking*>(selectedUI)) {
            QProcess *pythonProcess = new QProcess();

            // 실행할 Python 스크립트 경로 설정
            QString pythonExecutable = "python3";  // Windows의 경우 "python" 또는 절대 경로 지정 필요
            QString pythonScript = QString("/home/%1/llm_ws/koras_cooking_ui.py").arg(USER_NAME.c_str());

            // Python 프로세스 시작
            pythonProcess->start(pythonExecutable, QStringList() << pythonScript);

            // 실행 로그를 콘솔에 출력
            QObject::connect(pythonProcess, &QProcess::readyReadStandardOutput, [=]() {
                QByteArray output = pythonProcess->readAllStandardOutput();
                std::cout << "Python Output: " << output.toStdString() << std::endl;
            });

            QObject::connect(pythonProcess, &QProcess::readyReadStandardError, [=]() {
                QByteArray errorOutput = pythonProcess->readAllStandardError();
                std::cerr << "Python Error: " << errorOutput.toStdString() << std::endl;
            });

            QObject::connect(pythonProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                            [=](int exitCode, QProcess::ExitStatus exitStatus) {
                std::cout << "Python process finished with exit code: " << exitCode << std::endl;
                if (exitStatus == QProcess::CrashExit) {
                    std::cerr << "Python process crashed!" << std::endl;
                }
            });
        } else if (auto hanyangEngUI = dynamic_cast<MainWindow_widgetsHanyangEng*>(selectedUI)) {
            //// TODO:


        } else if (auto monitoringUI = dynamic_cast<MainWindow_widgetsMonitoring*>(selectedUI)) {
            //// TODO:

        }
    }
    /////////////////////////////////////





    /////////////////////////////////////
    // LOG 출력
    std::cout << COUT_BLUE_GRREN;
    std::cout << "*************************************************************************" << std::endl;
    std::cout << "************************** KORAS SYSTEM S/W *****************************" << std::endl;
    std::cout << "*************************************************************************" << std::endl;
    std::cout << COUT_RESET; // 색상 초기화

    std::cout << COUT_YELLOW; // 기본 출력 색상을 노란색으로 설정

    if (argc > 1) {
        std::string argument = argv[1];
        if (argument == "HD_LLM") {
            std::cout << "Starting " << COUT_GREEN << COUT_BOLD << "HD_LLM (Hyundai LLM, 24.05~24.12)" << COUT_RESET << COUT_YELLOW << " UI" << std::endl;
        } else if (argument == "COOKING") {
            std::cout << "Starting " << COUT_BLUE << COUT_BOLD << "COOKING (Koras Robotics)" << COUT_RESET << COUT_YELLOW << " UI" << std::endl;
        } else if (argument == "HANYANG_ENG") {
            std::cout << "Starting " << COUT_BLUE << COUT_BOLD << "HANYANG_ENG (Chemical Coupler, 24.09~)" << COUT_RESET << COUT_YELLOW << " UI" << std::endl;
        }


    }
    std::cout << COUT_RESET; // 색상 초기화

    std::cout << COUT_BLUE_GRREN;
    std::cout << "*************************************************************************" << std::endl;
    std::cout << "*************************************************************************" << std::endl;
    std::cout << "*************************************************************************" << std::endl;

    std::cout << COUT_RESET; // 색상 초기화
    /////////////////////////////////////


    QObject::connect(initialSetupWindow, &MainWindow_initialSetup::openTaskGeneratorWindow, [&]() {
        taskplan::TaskPlannerWidget* taskPlannerWidget = new taskplan::TaskPlannerWidget(qnode, nullptr);
        taskPlannerWidget->setWindowTitle("Task Planner");
        taskPlannerWidget->show();
    });


    // mainWindow->show();
    mainWindow->showMaximized();
    return app.exec();
    // } else {
    //     // 로그인 실패 시 프로그램 종료
    //     return 0;
    // }

}