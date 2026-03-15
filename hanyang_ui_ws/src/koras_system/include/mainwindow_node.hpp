// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause
#ifndef MAINWINDOW_NODE_HPP_
#define MAINWINDOW_NODE_HPP_
#include <QtGui/QScreen>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QTabBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QDialog>
#include <QtCore/QLibraryInfo>
#include <QtCore/QList>
#include <QtCore/QDebug>

#include <QtNodes/ConnectionStyle>
#include <QtNodes/DataFlowGraphModel>
#include <QtNodes/DataFlowGraphicsScene>
#include <QtNodes/GraphicsView>
#include <QtNodes/NodeData>
#include <QtNodes/NodeDelegateModelRegistry>
#include <QtNodes/GraphicsViewStyle>
#include <QtNodes/NodeStyle>
#include <QtNodes/ConnectionStyle>

#include "TaskStartModel.hpp"
#include "KCRCommandModel.hpp"
#include "PLCCommandModel.hpp"
#include "BPCommandModel.hpp"
#include "taskManager.hpp"
// #include "scanning_dialog.hpp"

//#include "other/nodeModelVStickWidget.hpp"

using QtNodes::ConnectionStyle;
using QtNodes::DataFlowGraphicsScene;
using QtNodes::DataFlowGraphModel;
using QtNodes::GraphicsView;
using QtNodes::GraphicsViewStyle;
using QtNodes::NodeStyle;
using QtNodes::NodeDelegateModelRegistry;
using QtNodes::NodeRole;

class NodeDelegateModelRegistry;
class DataFlowGraphModel;
class TaskStartModel;
class KCRCommandModel;
class BPCommandModel;

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
QT_END_NAMESPACE
class QNode;
class TaskManager;

class MainWindow_node : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow_node(std::shared_ptr<NodeDelegateModelRegistry> registry, QNode* qnode, QWidget* parent = nullptr);
    QWidget* get_widget_content(QWidget* widget);

    // Recursively log all child widgets.
    void logChildren(QWidget *widget) {
      for (QObject *child : widget->children()) {
        QWidget *childWidget = qobject_cast<QWidget*>(child);
        if (childWidget) {
          qDebug() << childWidget->objectName();
          logChildren(childWidget);
        }
      }
    }
    // Log all child widgets of the specified widget.
    void logWidget(QWidget *widget) {
      qDebug() << widget->objectName();
      logChildren(widget);
    }

    void traverseWidgets(QLayout* layout)
    {
        if (!layout)
            return;

        for (int i = 0; i < layout->count(); ++i) {
            QLayoutItem* item = layout->itemAt(i);
            if (item) {
                QWidget* widget = item->widget();
                if (widget) {
                    qDebug() << "위젯 " << widget->objectName();
                }
                else {
                    QLayout* childLayout = item->layout();
                    if (childLayout) {
                        traverseWidgets(childLayout);
                    }
                }
            }
        }
    }
    TaskManager* getTaskManager();
private Q_SLOTS:
    void about();
    void showDocumentation();

private:
    DataFlowGraphicsScene* scene;
    DataFlowGraphModel* dataFlowGraphModel;
    GraphicsView* view;
    // ScanningDialog* scan_dialog;
    TaskManager* _taskManager;

    QMenu *fileMenu;
    QMenu *connMenu;
    QMenu *taskMenu;
    QMenu *nodeMenu;
    QMenu *parameterSettingMenu;

    QAction *newAction;
    QAction *saveAction;
    QAction *loadAction;
    QAction *tcpConnectAction;
    QAction *plcConnectAction;
    QAction *taskAction;
    QAction *connectAction;
    QAction *popupAction;

    QToolBar *toolbar;
    QDockWidget *dockWidget;
    QAction *act[5];
};

#endif