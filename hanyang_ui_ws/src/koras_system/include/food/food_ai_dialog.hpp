#ifndef FOOD_AI_DIALOG_HPP
#define FOOD_AI_DIALOG_HPP

#include <QDialog>
#include <QMessageBox>
#undef slots  // Qt의 slots 키워드 비활성화
#include <pybind11/embed.h>
#define slots Q_SLOTS  // Qt의 slots 키워드를 다시 활성화
#include <iostream>

namespace py = pybind11;

namespace Ui {
class FoodAIDialog;
}

class FoodAIDialog : public QDialog
{
    Q_OBJECT

public:
    FoodAIDialog(QWidget *parent = 0);
    ~FoodAIDialog();
    // void initializeDialog();

public Q_SLOTS:
    void pushButtonStartClickedCallback();
    void pushButtonStopClickedCallback();
    void pushButtonGraspClickedCallback();

private:
    Ui::FoodAIDialog* ui;
    py::object ros_thread;
};

#endif // FOOD_AI_DIALOG_HPP
