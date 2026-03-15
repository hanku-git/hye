#include "ui_login_user_table_widget.h"
#include "login_user_table_widget.hpp"

LoginUserTableWidget::LoginUserTableWidget(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::LoginUserTableWidget)
{
    ui->setupUi(this);
    
    loadUserData();

    ui->userTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->userTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
    QObject::connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &LoginUserTableWidget::saveUserData);
    QObject::connect(ui->pushButton_add, &QPushButton::clicked, this, &LoginUserTableWidget::addRow);    
    QObject::connect(ui->pushButton_remove, &QPushButton::clicked, this, &LoginUserTableWidget::removeRow);
    QObject::connect(ui->pushButton_refresh, &QPushButton::clicked, this, &LoginUserTableWidget::dataLoad);
    
}

LoginUserTableWidget::~LoginUserTableWidget()
{
    delete ui;
}

void LoginUserTableWidget::loadUserData() {
    ui->userTableWidget->setColumnCount(3);
    QStringList header_list;
    header_list << "ID" << "User name" << "Password (Hashed)";
    ui->userTableWidget->setHorizontalHeaderLabels(header_list);
    int columnWidth = 300;
    ui->userTableWidget->setColumnWidth(2, columnWidth);

    auto db = SqliteHelper::connectDatabase("user.db");
    if (!db) {
        std::cerr << "Failed to connect to the database." << std::endl;
        return;
    }
    SqliteHelper::createUserTable(db);

    info.clear();
    info = SqliteHelper::selectUser(db);
    dataLoad();
}

void LoginUserTableWidget::dataLoad() {
    ui->userTableWidget->setRowCount(0);
    for(int row = 0; row < info.size(); ++row) {
        ui->userTableWidget->insertRow(row);
        QTableWidgetItem* item = new QTableWidgetItem(QString::number(row));
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        ui->userTableWidget->setItem(row, 0, item);
        ui->userTableWidget->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(info[row].username)));
        ui->userTableWidget->setItem(row, 2, new QTableWidgetItem(QString::fromStdString(info[row].password)));
    }
}

int LoginUserTableWidget::saveUserData() {
    //widget item -> new info -> if ok save or recover info
    std::vector<UserInfo> new_info;
    std::vector<int> new_ids;
    std::vector<int> old_ids;
    for(int row = 0; row < ui->userTableWidget->rowCount(); ++row) {

        QTableWidgetItem* item0 = ui->userTableWidget->item(row, 0);
        QTableWidgetItem* item1 = ui->userTableWidget->item(row, 1);
        QTableWidgetItem* item2 = ui->userTableWidget->item(row, 2);

        if (item0 && item1 && item2) { 
            new_info.push_back(UserInfo{item0->text().toInt(), item1->text().toStdString(), item2->text().toStdString()});
        }
    }

    auto db = SqliteHelper::connectDatabase("user.db");
    if (!db) {
        std::cerr << "Failed to connect to the database." << std::endl;
        return 0;
    }
    int count;

    new_ids.reserve(new_info.size()); 
    old_ids.reserve(info.size());

    std::transform(new_info.begin(), new_info.end(), std::back_inserter(new_ids),
                   [](const UserInfo& data) -> int {
                       return data.id;
                   });
    std::transform(info.begin(), info.end(), std::back_inserter(old_ids),
                   [](const UserInfo& data) -> int {
                       return data.id;
                   });

    auto deleteIDs = SqliteHelper::computeDifference(old_ids, new_ids);
    auto insertIDs = SqliteHelper::computeDifference(new_ids, old_ids);
    auto updateIDs = SqliteHelper::computeDifference(new_ids, insertIDs);

    for (auto id : deleteIDs) {
        SqliteHelper::deleteUser(db, id);
    }
    for (auto id : insertIDs) {
        for (auto _info : new_info) {
            if (id == _info.id) {
                SqliteHelper::insertUser(db, id, _info.username, _info.password);
            }
        }
    }
    // if new info password changed -> SHA512
    for (auto id : updateIDs) {
        for (auto _info : new_info) {
            if (id == _info.id) {    
                for (auto __info : info) {
                    if (id == __info.id && _info.password != __info.password) {
                        auto new_password = SqliteHelper::SHA512(_info.password);
                        _info.password = new_password;
                    }
                }            
                SqliteHelper::updateUser(db, _info);
            }
        }
    }

    return 1;
}

void LoginUserTableWidget::addRow() {
    int row = ui->userTableWidget->rowCount();
    ui->userTableWidget->insertRow(row);
    QTableWidgetItem* item = new QTableWidgetItem(QString::number(row));
    item->setTextAlignment(Qt::AlignCenter);
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    ui->userTableWidget->setItem(row, 0, item);
}

void LoginUserTableWidget::removeRow() {
    ui->userTableWidget->removeRow(ui->userTableWidget->currentRow());
    for(int row = 0; row < ui->userTableWidget->rowCount(); ++row) {
        QTableWidgetItem* item = new QTableWidgetItem(QString::number(row));
        item->setTextAlignment(Qt::AlignCenter);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        ui->userTableWidget->setItem(row, 0, item);
    }    
}
