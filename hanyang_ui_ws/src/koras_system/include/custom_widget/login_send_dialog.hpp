#ifndef LOGIN_SEND_DIALOG_HPP
#define LOGIN_SEND_DIALOG_HPP

#include <QDialog>
#include <QTimer>


namespace Ui {
class SendDialog;
}

class SendDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SendDialog(QWidget *parent = 0);
    ~SendDialog();

private Q_SLOTS:
    void on_btnRead_clicked();

    void on_btnCalculate_clicked();

    void on_btnReset_clicked();

    void clock();


    void on_btnSendEmail_clicked();

    void on_btnDeleteData_clicked();

private:
    Ui::SendDialog *ui;
    QTimer *timer;
};

#endif // DIALOG_H
