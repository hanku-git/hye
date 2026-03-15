#include "mainwindow_cooking_aw.hpp"
#include "ui_mainwindow_cooking_aw.h"
#undef slots  // Qt의 slots 키워드 비활성화
#include <pybind11/embed.h>
#define slots Q_SLOTS  // Qt의 slots 키워드를 다시 활성화

namespace py = pybind11;



MainWindow_cookingAW::MainWindow_cookingAW(QNode* qnode,QWidget* parent)
    : qnode_(qnode)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_cookingAW)
    , ros_thread(py::none())  // Python ROS2 thread 객체 초기화 (py::none()으로 초기화)
{
    ROS_LOG_WARN("[%s] [line: %d]", __func__, __LINE__);
    ui->setupUi(this);
    ////////////////////////////// definition groupboxes, layouts

    // Python 인터프리터를 한 번만 초기화
    py::initialize_interpreter();

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_cookingAW::timerCallback);
    timer_->start(33);


    QObject::connect(ui->pushButton_recording, SIGNAL(clicked()), this, SLOT(pushButtonRecordingClickedCallback()));
    QObject::connect(ui->pushButton_start, SIGNAL(clicked()), this, SLOT(pushButtonStartClickedCallback()));
    QObject::connect(ui->pushButton_stop, SIGNAL(clicked()), this, SLOT(pushButtonStopClickedCallback()));


}

MainWindow_cookingAW::~MainWindow_cookingAW()
{
    // 프로그램 종료 시 Python 인터프리터 종료
    py::finalize_interpreter();
    delete ui;
    rclcpp::shutdown();
}

void MainWindow_cookingAW::timerCallback() {
    // QImage image = _taskWindow->getTaskManager()->_qnode->getCameraImage();
    // QPixmap pix;
    // // initialize raw image to qpixmap
    // pix = QPixmap::fromImage(image, Qt::AutoColor);
    // // show image on qlabel
    // ui->label_image_viewer->setPixmap(pix);
}

void MainWindow_cookingAW::statusTimerCallback() {


}

void MainWindow_cookingAW::testBtnCallback() {

}

void MainWindow_cookingAW::pushButtonStartClickedCallback() {
    try {
        if (ros_thread.is_none()) {  // 스레드가 없을 경우에만 초기화
            py::module_ sys = py::module_::import("sys");
            sys.attr("path").attr("append")("/home/bp/graphy_ws/src/graphy/hanyang_eng_koras_system/src/llm_node");

            // ros2_node_test 모듈 및 ROS2_Thread 클래스 가져오기
            py::module_ ros2_node = py::module_::import("ros2_node_test");
            ros_thread = ros2_node.attr("ROS2_Thread")(nullptr);  // 스레드 객체 생성
        }

        // Python ROS2 스레드 시작
        std::cout << "Starting Python ROS2 thread...\n";
        py::gil_scoped_acquire acquire;  // GIL을 확보하여 Python 스레드를 시작
        ros_thread.attr("start")();

    } catch (py::error_already_set &e) {
        std::cerr << "Python Error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "C++ Exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception occurred!" << std::endl;
    }
}

void MainWindow_cookingAW::pushButtonStopClickedCallback() {
    try {
        if (!ros_thread.is_none()) {  // 스레드가 있을 경우에만 중지
            py::gil_scoped_acquire acquire;  // GIL을 확보하여 Python 스레드를 중지
            ros_thread.attr("stop")();
            std::cout << "Python ROS2 thread stopped.\n";
        }
    } catch (py::error_already_set &e) {
        std::cerr << "Python Error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "C++ Exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception occurred!" << std::endl;
    }
}

// Python MainWindow_cookingAW 호출 예시
void MainWindow_cookingAW::pushButtonGraspClickedCallback() {
    try {
        if (!ros_thread.is_none()) {
            py::gil_scoped_acquire acquire;  // Python 함수 호출 전 GIL 확보
            // py::object bboxes = ros_thread.attr("get_bboxes")("any object");
            py::object test_attr = ros_thread.attr("test");

            // Python 문자열을 C++의 std::u32string으로 변환
            std::u32string str = test_attr.cast<std::u32string>();

            // std::u32string을 const char32_t*로 변환
            const char32_t* result = str.c_str();

            // std::cout << "Bounding boxes from Python: " << result << std::endl;
        }
    } catch (py::error_already_set &e) {
        std::cerr << "Python Error: " << e.what() << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "C++ Exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception occurred!" << std::endl;
    }
}

