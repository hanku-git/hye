#pragma once

#include <QMainWindow>
#include <QCloseEvent>

class MainWindow_custom : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow_custom(QWidget* parent = nullptr);

protected:
    void closeEvent(QCloseEvent* event) override;
};