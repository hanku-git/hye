#ifndef LOGIN_USER_TABLE_WIDGET_HPP
#define LOGIN_USER_TABLE_WIDGET_HPP

#include <QDialog>
#include <QStringList>
#include <QMap>
#include <QList>
#include <QAbstractTableModel>

#include "sqliteHelper.hpp"

namespace Ui {
class LoginUserTableWidget;
}

class LoginUserTableWidget : public QDialog
{
    Q_OBJECT

public:
    explicit LoginUserTableWidget(QWidget *parent = nullptr);
    ~LoginUserTableWidget();
    
    void loadUserData();
    

private Q_SLOTS:
    int saveUserData();
    void dataLoad();
    void addRow();
    void removeRow();

private:
    Ui::LoginUserTableWidget *ui;
    std::vector<UserInfo> info;
};

#endif // LOGIN_USER_TABLE_WIDGET_HPP
