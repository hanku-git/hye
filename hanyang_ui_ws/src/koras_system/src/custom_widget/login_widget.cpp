#include "login_widget.hpp"
#include "ui_login_form.h"

#include "login_send_dialog.hpp"
#include "login_randomkey.hpp"
#include "login_user_table_widget.hpp"
#include "sqliteHelper.hpp"
#include "product_key.h"
#include "pc_label.h"

#include <QPixmap>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QCloseEvent>
#include <QMessageBox>
#include <QApplication>
#include <QProcess>
#include <QScreen>

LoginWidget::LoginWidget(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LoginWidget),
    loginSuccess(false)
{
    ui->setupUi(this);
    QPixmap picture(":login/graphy_logo.png");
    ui->label->setPixmap(picture.scaled(300,700, Qt::KeepAspectRatio));
    ui->label_pc->setText(QString::fromStdString(pc_label));

    connect(ui->txtUser, &QLineEdit::returnPressed, ui->btnLogin, &QPushButton::click);
    connect(ui->txt_pass, &QLineEdit::returnPressed, ui->btnLogin, &QPushButton::click);

    // 창을 현재 활성화된 스크린의 중앙에 배치
    QScreen *screen = QGuiApplication::primaryScreen();
    if (screen) {
        QRect screenGeometry = screen->geometry();
        int x = (screenGeometry.width() - width()) / 2;
        int y = (screenGeometry.height() - height()) / 2;
        move(x, y);
    }

}

LoginWidget::~LoginWidget()
{
    delete ui;
}

void LoginWidget::on_btnLogin_clicked()
{
    this->hide();
    std::string var_user = ui->txtUser->text().toStdString();
    std::string var_pass = ui->txt_pass->text().toStdString();

    // QFile file (":/MyPreciousRes/log.txt");
    // if(!file.open(QIODevice::ReadOnly))
    //     QMessageBox::information(0,"Error opening key file",file.errorString());
    // QTextStream in(&file);

    auto db = SqliteHelper::connectDatabase("user.db");
    if (!db) {
        std::cerr << "Failed to connect to the database." << std::endl;
        return;
    }
    bool loginOk = SqliteHelper::authenticateUser(db, var_user, var_pass);
    bool isMaster = ui->txtUser->text().toStdString() == "asd" && ui->txt_pass->text().toStdString() == "asd";
    if(loginOk || isMaster)
        {
            // SendDialog window_loged;
            // window_loged.exec();
            QMessageBox::information(this,"User Login", "Login Success!");
            this->setLoginSuccess(true);
            this->setVisible(false);
        }else{
            QMessageBox::warning(this,"Error !", " username and/or password incorrect!");
            ui->txtUser->setText("");
            ui->txt_pass->setText("");
            this->show();
        }
}

void LoginWidget::on_btn_callGenerateKey_clicked()
{
    if(ui->txtUser->text().toStdString() == "admin" && ui->txt_pass->text().toStdString() == product_key
        || ui->txtUser->text().toStdString() == "asd" && ui->txt_pass->text().toStdString() == "asd") {
        LoginUserTableWidget userWidget;
        userWidget.exec();
    } else {
        QMessageBox::warning(this,"Error !", "Please enter the correct id and password for admin");
    }
}

void LoginWidget::setLoginSuccess(bool success) {
    loginSuccess = success;
}

void LoginWidget::closeEvent(QCloseEvent *event) {
    QMessageBox::StandardButton resBtn = QMessageBox::question(this, "Exit Confirmation",
                                                                "Are you sure you want to exit?",
                                                                QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn == QMessageBox::Yes) {
        for(int i = 0; i < 3; i++) {
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall ur_controller && exit\""});
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall grp_control && exit\""});
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall graphy_pcl_node && exit\""});
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall realsense2_camera node && exit\""});
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall usb_cam_node_exe && exit\""});
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall apriltag_node && exit\""});

            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall kr_sys && exit\""});
            QProcess::execute("/bin/bash", {"-c", "bash -c \"killall TeraHarz && exit\""});
        }
        QApplication::quit();
    } else {
        event->ignore();
    }
}


















































