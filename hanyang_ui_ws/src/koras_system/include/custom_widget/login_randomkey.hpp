#ifndef LOGIN_RANDOMKEY_HPP
#define LOGIN_RANDOMKEY_HPP

#include <QDialog>

namespace Ui {
class EnterSerial;
}

class EnterSerialDialog : public QDialog
{
    Q_OBJECT

public:
    explicit EnterSerialDialog(QWidget *parent = 0);
    ~EnterSerialDialog();

private Q_SLOTS:

    void on_btn_enter_serial_clicked();

private:
    Ui::EnterSerial *ui;
};

#endif // LOGIN_RANDOMKEY_HPP
