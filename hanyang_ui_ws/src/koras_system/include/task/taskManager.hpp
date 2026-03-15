#ifndef TASKMANAGER_HPP_ 
#define TASKMANAGER_HPP_

#include <iostream>
#include <vector>
#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>

#include <QtNodes/DataFlowGraphModel>
#include <QtNodes/ConnectionIdUtils>
#include <QtNodes/NodeDelegateModel>

#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>
#include <QtCore/QTimer>
#include <QtCore/QByteArray>
#include <QtCore/QVariant>

#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QListWidgetItem>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QHeaderView>

#include <QtCore/QAbstractItemModel>
#include <QtCore/QItemSelectionModel>

#include "protocolDefine.hpp"
#include "modbusTcpDefine.hpp"
//#include "rosNode.hpp"

#include "qnode.hpp"
#include "KCRCommandModel.hpp"

//robot image 도 나중에 추가할 것. 탈부착?
QT_BEGIN_NAMESPACE
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QTextEdit;
class QPushButton;
class QProgressBar;
class QListView;
class QThread;
class QJsonDocument;
QT_END_NAMESPACE
class QNode;

using QtNodes::DataFlowGraphModel;
using QtNodes::ConnectionId;
using QtNodes::NodeId;
using QtNodes::NodeDelegateModel;

class TaskManager : public QWidget
{
    Q_OBJECT
// loading from Model (serializing) <-> (deserializing)script & itemModel(treeItemWidget) List visual timer-based width
// --> 플레이시

public:
    TaskManager(DataFlowGraphModel &graphModel, QNode *qnode);
    ~TaskManager() = default;

    template<typename DelegateModel>
    QJsonObject createScript(DelegateModel& model) {
        QJsonObject jsonObject;
        jsonObject = model->save();
        //taskScriptBuilder builder;
        return jsonObject;
    }
public : 
    void setBeforeTaskSignal();

public Q_SLOTS:
    void GenTaskScriptFromGraph();
    void OnPlayerButtonClicked();
    void OnPauseButtonClicked();
    void OnTaskStopButtonClicked();

private:
    void zoomAction();
    void makeSimulTaskFromScript();

Q_SIGNALS:
    void unitTaskFinished();
    void taskPlayed();
    void taskPaused();
    void taskResumed();

protected Q_SLOTS:
    //void InitializeTaskPlanner(NodeId starting_id, std::unordered_set<ConnectionId> starting_conn_ids);
    void addIconsFromGraph();
    void OnProgressCompleted();
    void OnEditButtonClicked();
    void OnRefreshButtonClicked();
    //Emergency Stop.. or Robot Error received from out-boundary like PLCs or Robots
    void OnEmergencyState();
   // void setTaskMode();
public:
    std::vector<NodeId> task_nodes;
    uint task_count;
    std::vector<int> tag_counts;
    DataFlowGraphModel &_graphmodel;
private:
    QLabel *IconLabel;

    QTextEdit *scriptEdit;
    QPushButton *editButton;
    QPushButton *refreshButton;

    QJsonObject scriptJson;
    QCheckBox *zoom_checkbox;
    QPushButton *zoom_button;
    
    QTreeWidget *moduleTaskTreeView;
    QPushButton *loadTaskbutton;

    QTextEdit *sendEdit;
    QTextEdit *receiveEdit;

    QLineEdit *hostLineEdit;
    QLineEdit *portLineEdit;

    QLabel *progressStatusLabel;
    QProgressBar *progressStatusBar;

    QPushButton *playButton;
    QPushButton *pauseButton;
    QPushButton *resumeButton;
    QPushButton *stopButton;
    //to do : return to start point after confirm dialogue
    QPushButton *simulToggleButton;
    QPushButton *simulStartButton;
    QPushButton *simulStopButton;


    //QNode &_qnode;
    //3가지를 구현해야한다. 일단은 스크립트 부터 짠다
    QString *taskScript;  // from model -> json string  with id index played in sequence by serializing object especially for nodes which are connected
    QList<QString> *taskList;  // -> 스크립트에 따른 모델의 순서 -->>>>> ★★★★★★★★★★graph모델의 뷰어로만 작용한다.
    QListView *iconView;  // -> 프로그래스 진행에 따른 모델 시각화 및 아이콘배치, 이 역시 ★★★★★★★★★★graph모델의 뷰어로만 작용한다.
    //SubProgram subProgram;
    //QThread workerThread;
    //QWaitCondition condition;
    //QMutex mutex;
    //bool isStopped = false;
    std::vector<QJsonObject> messages;
    std::vector<std::string> task_msgs;


    bool startFlag_;
    NodeId startingNodeId_;
    
    //network관련
    modbus* _client;
    
    bool isRobotStop;
    bool isPlcStop;

    bool isTaskPaused;

    QTreeWidget *taskTreeView;

    uint16_t teeth_index_in;
    //The QTreeWidgetItem must be added to the QTreeWidget
public:
    QNode *_qnode;
};

#endif // HEADER_H_