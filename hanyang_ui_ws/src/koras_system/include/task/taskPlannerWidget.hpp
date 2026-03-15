#ifndef TASKPLANNERWIDGET_HPP
#define TASKPLANNERWIDGET_HPP

#include "ui_task_planner_widget.h"  // Qt가 자동 생성한 헤더

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QComboBox>
#include <QMap>
#include <QLineEdit>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFileDialog>
#include <QJsonObject>
#include <QJsonArray>
#include <QMessageBox>
#include <QListWidget>
#include <QInputDialog>
#include <QProgressBar>

#include "qnode.hpp"
#include "exprtk.hpp"
#include "lua.hpp"
#include "sol/sol.hpp"

namespace taskplan {

struct UnitTask_ {
    QString taskType;
    QJsonArray params;
    UnitTask_(QString taskType, QJsonArray params);
};

struct Task {
    QString name;
    QList<UnitTask_> unitTasks;
    QString scriptText;
    Task(QString name);
};

class CompositeTask {
public:
    QString name;
    QList<std::shared_ptr<Task>> moduleTasks;
    CompositeTask(QString name);
};
// TaskDefinition 구조체
struct TaskDefinition {
    QString taskType; // 태스크 타입
    QList<QPair<QString, QString>> parameters; // 파라미터 리스트
    QString luaFunctionName; // Lua 함수 이름
    std::function<void(sol::state&, const QString&)> luaRegistration; // Lua 등록 함수
    QString luaScriptTemplate; // Lua 스크립트 템플릿
    int expectedArgCount = 0; // Lua function compare c++ function arguments
};

// TaskDefinitionBuilder 빌더 클래스
class TaskDefinitionBuilder {
public:
    TaskDefinitionBuilder& setTaskType(const QString& taskType, const QList<QPair<QString, QString>>& parameters) {
        definition_.taskType = taskType;
        definition_.parameters = parameters;
        return *this;
    }
    TaskDefinitionBuilder& setLuaFunctionName(const QString& luaFunctionName) {
        definition_.luaFunctionName = luaFunctionName;
        return *this;
    }
    // template <typename Ret, typename ClassType, typename... Args>
    // TaskDefinitionBuilder& setLuaRegistration(
    //     Ret (ClassType::*method)(std::vector<UnitTask>&, Args...),
    //     ClassType* instance,
    //     std::vector<UnitTask>& targetTasks
    // ) {
    //     definition_.luaRegistration = [method, instance, &targetTasks](sol::state& lua, const QString& luaFunctionName) {
    //         lua.set_function(luaFunctionName.toStdString(), [method, instance, &targetTasks](Args... args) {
    //             // targetTasks를 전달하여 method 호출
    //             (instance->*method)(targetTasks, args...);
    //         });
    //     };
    //     return *this;
    // }
    template <typename Ret, typename ClassType, typename... Args>
    TaskDefinitionBuilder& setLuaRegistration(
        Ret (ClassType::*method)(std::vector<UnitTask>&, Args...),
        ClassType* instance,
        std::vector<UnitTask>& targetTasks
    ) {
        definition_.luaRegistration = [method, instance, &targetTasks](sol::state& lua, const QString& luaFunctionName) {
            lua.set_function(luaFunctionName.toStdString(), [method, instance, &targetTasks](Args... args) {
                (instance->*method)(targetTasks, args...);
            });
        };
        // Args...의 개수를 definition_에 기록
        definition_.expectedArgCount = static_cast<int>(sizeof...(Args));
        return *this;
    }
    TaskDefinitionBuilder& setLuaScriptTemplate(const QString& scriptTemplate) {
        definition_.luaScriptTemplate = scriptTemplate;
        return *this;
    }
    // TaskDefinition build() {
    //     return definition_;
    // }
    TaskDefinition build() {
        if (definition_.parameters.size() != definition_.expectedArgCount) {
            QStringList paramDetails;
            for (const auto& p : definition_.parameters) {
                paramDetails << QString("%1(%2)").arg(p.first, p.second);
            }
            throw std::runtime_error(
                QString("Parameter count mismatch in task '%1' with function '%2': expected %3 args, but got %4 parameters.\n"
                        "Expected Args: %3\n"
                        "Actual Parameters: %5")
                    .arg(definition_.taskType)
                    .arg(definition_.luaFunctionName)
                    .arg(definition_.expectedArgCount)
                    .arg(definition_.parameters.size())
                    .arg(paramDetails.join(", "))
                    .toStdString()
            );
        }
        return definition_;
    }
private:
    TaskDefinition definition_;
};

class TaskPlannerWidget : public QWidget {
    Q_OBJECT

public:
    TaskPlannerWidget(QNode* qnode, QWidget* parent = nullptr);
    ~TaskPlannerWidget();

private Q_SLOTS:
    void addModuleTaskToCompositeTask();
    void removeModuleTaskFromCompositeTask();
    void saveCompositeTask();
    void onModuleTaskNameChanged(QTableWidgetItem* item);
    void addTask();
    void removeTask();
    void addUnitTask();
    void saveAllTasks();
    void loadAllTasks();
    void onTaskSelectionChanged(int currentRow, int currentColumn, int previousRow, int previousColumn);
    void updateParamInputs();
    void executeScript();
    void onRunButtonClicked();
    void onStopButtonClicked();

    void updateProgressBar();
    void onTaskExecutionFinished();

private:
    Ui::TaskPlannerWidget ui;  // UI 인스턴스
    void connectSignals();

    void initUI();
    std::vector<TaskDefinition> createTaskDefinitions(TaskPlannerDefault& planner, std::vector<UnitTask>& targetModuleTask);

    std::vector<UnitTask_> parseScriptToUnitTasks(const QString& script);

    QString generateUniqueTaskName(const QString& baseName);
    void runParsedScript(const QString& script);
    void registerTaskFunctions();
    void initConditionalAndLoopUI();
    void addConditionalBlockUI();
    void addLoopBlockUI();
    void closeBlock();
    void executeTaskCycle();
    QVBoxLayout* mainLayout;
    QHBoxLayout* topLayout;

    QTableWidget* taskTable;

    QVBoxLayout* paramInputLayout;
    QWidget* paramInputWidget;
    QComboBox* taskTypeSelector;

    QPushButton* addTaskBtn;
    QPushButton* removeTaskBtn;
    QPushButton* addUnitTaskBtn;
    QPushButton* saveAllTasksBtn;
    QPushButton* loadAllTasksBtn;

    CompositeTask compositeTask;
    QListWidget* compositeTaskList;
    QPushButton* addModuleTaskBtn;
    QPushButton* removeModuleTaskBtn;
    QPushButton* saveCompositeTaskBtn;
    QTextEdit* scriptEditor_lua;
    QPushButton* runLuaScriptBtn;
    QPushButton* runButton;
    QPushButton* stopButton;
    QProgressBar* progressBar;
    std::vector<std::shared_ptr<Task>> tasks;
    QMap<QString, QWidget*> paramInputFields;

    QNode* qnode_;
    sol::state lua_;
    std::vector<UnitTask> targetModuleTask_;
    std::vector<TaskDefinition> taskDefinitions;
    QSpinBox* repeatSpinBox;
    int currentIteration = 0;
    bool isRepeating = false;
};

} // namespace taskplan

#endif // TASKPLANNERWIDGET_HPP
