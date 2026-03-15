#include "login_send_dialog.hpp"
#include "ui_send_form.h"
#include <QtCore/QString>
#include <QtCore/QFile>
#include <QtCore/QDebug>
#include <QtCore/QTextStream>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QString>
#include <QTimer>
#include <QDateTime>
#include <QProcess>
#include <QDesktopServices>

SendDialog::SendDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SendDialog)
{
    ui->setupUi(this);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(clock()));
    timer->start(1000);
    QPixmap picture_mail(":login/email.png");
        ui->lblEmail->setPixmap(picture_mail.scaled(50,50, Qt::KeepAspectRatio));
    QPixmap picture_delete(":login/trash.png");
        ui->lblDelete->setPixmap(picture_delete.scaled(20,20, Qt::KeepAspectRatio));
    QPixmap picture_reload(":login/reload.png");
        ui->lblreset->setPixmap(picture_reload.scaled(15,15, Qt::KeepAspectRatio));

}

SendDialog::~SendDialog()
{
    delete ui;
}

void SendDialog::clock()
{
    QTime time = QTime::currentTime();
    QString time_text = time.toString("hh : mm : ss");
    if((time.second() % 2) == 0)
    {
        time_text[3] = ' ';
        time_text[8] = ' ';
    }
    ui->lbl_time->setText(time_text);
}

void SendDialog::on_btnRead_clicked()
{
    QString str;
    QString var_num1 = ui->txtnum1->text();
    QString var_num2 = ui->txtnum2->text();
    if(ui->rbtn_suma->isChecked())
    {
        int sum = var_num1.toInt() + var_num2.toInt();
        ui->lcdNumber->display(sum);
    }
}

void SendDialog::on_btnCalculate_clicked()
{

    QString var_num1 = ui->txtnum1->text();
    QString var_num2 = ui->txtnum2->text();


    // Suma check box
    if(ui->rbtn_suma->isChecked())
    {
        double sum = var_num1.toDouble() + var_num2.toDouble();
        ui->lcdNumber->display(sum);
        QFile file("out.txt");

        if(!file.open(QIODevice::Append | QIODevice::Text))
            return;
        QTextStream out(&file);
        QTime time = QTime::currentTime();
        QString time_text = time.toString("hh : mm : ss");
        out << "Log sum: " << sum << ", time: " << time_text << "\n";


    }else if(ui->rbtn_resta->isChecked()){
        double res = var_num1.toDouble() - var_num2.toDouble();
        ui->lcdNumber->display(res);
        QFile file("out.txt");

        if(!file.open(QIODevice::Append | QIODevice::Text))
            return;
        QTextStream out(&file);
        QTime time = QTime::currentTime();
        QString time_text = time.toString("hh : mm : ss");
        out << "Log res: " << res << ", time: " << time_text << "\n";

    }else if(ui->rbtn_multi->isChecked()){
        double multi = var_num1.toDouble() * var_num2.toDouble();
        ui->lcdNumber->display(multi);
        QFile file("out.txt");

        if(!file.open(QIODevice::Append | QIODevice::Text))
            return;
        QTextStream out(&file);
        QTime time = QTime::currentTime();
        QString time_text = time.toString("hh : mm : ss");
        out << "Log mul: " << multi << ", time: " << time_text << "\n";

    }else if(ui->rbtn_div->isChecked()){
        double div;
        div = var_num1.toDouble() / var_num2.toDouble();
        if(var_num2 == 0)
        {
            QMessageBox::warning(this,"Error !", " In div operations denominator must be different than zero!");
            ui->txtnum2->setText("");
        }else{
        ui->lcdNumber->display(div);
        QFile file("out.txt");

        if(!file.open(QIODevice::Append | QIODevice::Text))
            return;
        QTextStream out(&file);
        QTime time = QTime::currentTime();
        QString time_text = time.toString("hh : mm : ss");
        out << "Log div: " << div << ", time: " << time_text << "\n";
        }
    }
}

void SendDialog::on_btnReset_clicked()
{
    ui->txtnum1->setText("");
    ui->txtnum2->setText("");
}


void SendDialog::on_btnSendEmail_clicked()
{
    QMessageBox::StandardButton answer = QMessageBox::question(this,"Send Email",
                                                               "Do you really want to send email?",
                                         QMessageBox::StandardButton::No | QMessageBox::StandardButton::Yes);
    if(answer == QMessageBox::StandardButton::Yes)
    {
        QProcess *process_send_email;
        QFile file("out.txt");

        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            return;
            QMessageBox::information(0, "Info",file.errorString());
        }

        QTextStream in(&file);
        QString results = in.readAll();
        qDebug() << results;


            process_send_email = new QProcess();
            process_send_email->start("mail -s Test QT - youremail@yourdomain.com");
            process_send_email->waitForStarted();
            process_send_email->write(results.toLatin1());
            process_send_email->closeWriteChannel();
            QMessageBox::information(this,"Information",
                                     "The email was sent successfully!", "OK");
        file.close();

    }
}

void SendDialog::on_btnDeleteData_clicked()
{

    QMessageBox::StandardButton answer = QMessageBox::question(this,"Erase Data",
                                                               "Do you really want to erase data?",
                                         QMessageBox::StandardButton::No | QMessageBox::StandardButton::Yes);
    if(answer == QMessageBox::StandardButton::Yes)
    {
        QFile file("out.txt");

        if(!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
            return;

        QMessageBox::information(this,"Information",
                                 "The data was erased", "OK");

    }



}
