#ifndef LOGIN_WIDGET_HPP
#define LOGIN_WIDGET_HPP

#include <QMainWindow>

namespace Ui {
class LoginWidget;
}

class LoginWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit LoginWidget(QWidget *parent = 0);
    void setLoginSuccess(bool success);
    ~LoginWidget();
    
    bool isLoginSuccessful() const { return loginSuccess; }
protected:
    void closeEvent(QCloseEvent *event) override;
private Q_SLOTS:
    void on_btnLogin_clicked();

    void on_btn_callGenerateKey_clicked();

private:
    Ui::LoginWidget *ui;
    bool loginSuccess;
};

#endif // LOGINWIDGET_H
