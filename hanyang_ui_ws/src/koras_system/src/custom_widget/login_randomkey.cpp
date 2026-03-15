#include "ui_enter_serial_form.h"
#include "login_randomkey.hpp"
#include "login_widget.hpp"
#include "login_user_table_widget.hpp"
#include "nocopy/product_key.h"

#include <QStringList>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>



EnterSerialDialog::EnterSerialDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::EnterSerial)
{
    ui->setupUi(this);
    QPixmap picture(":login/encryption_1.png");
        ui->lblKey->setPixmap(picture.scaled(30,30, Qt::KeepAspectRatio));
}

EnterSerialDialog::~EnterSerialDialog()
{
    delete ui;
}

void EnterSerialDialog::on_btn_enter_serial_clicked()
{
    // QFile file (":/MyPreciousRes/log.txt");
    // if(!file.open(QIODevice::ReadOnly))
    //     //extern QString key;
    //     QMessageBox::information(0,"Error opening key file",file.errorString());
    // QTextStream in(&file);

    // ui->textBrowser->setText(in.readLine(4));
    // ui->textBrowser_2->setText(in.readLine(4));
    // ui->textBrowser_3->setText(in.readLine(4));
    // ui->textBrowser_4->setText(in.readLine(4));
    QString combinedText = QString("%1%2%3%4")
                            .arg(ui->lineEdit->text())
                            .arg(ui->lineEdit_2->text())
                            .arg(ui->lineEdit_3->text())
                            .arg(ui->lineEdit_4->text());

    if (product_key == combinedText.toStdString()) {
        LoginUserTableWidget userWidget;
        userWidget.exec();
    }
}
