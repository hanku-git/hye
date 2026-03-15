#include "mainwindow_custom.hpp"
#include <QMessageBox>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>
#include "mainwindow_widgets_launch_package.hpp"

// // 외부 전역 포인터 예시
extern MainWindow_widgetsLaunchPackage* g_mainWindowLaunchPackage;

MainWindow_custom::MainWindow_custom(QWidget* parent)
    : QMainWindow(parent)
{
}

void MainWindow_custom::closeEvent(QCloseEvent* event) {
    // qDebug() << "[MainWindow_custom] closeEvent triggered";

    QMessageBox::StandardButton reply = QMessageBox::question(this, "Confirm Exit", "Are you sure you want to exit?",
                                    QMessageBox::Yes | QMessageBox::No);
      

    if (reply == QMessageBox::Yes) {
        if (g_mainWindowLaunchPackage) {
            g_mainWindowLaunchPackage->terminateQProcessAndRosNode();
        }
        rclcpp::shutdown();  // ROS2 안전 종료
        event->accept();     // 창 종료
    } else {
        event->ignore();     // 창 유지
    }
}
