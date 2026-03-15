// Copyright (C) 2017 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "mainwindow_node.hpp"
#include "std_msgs/msg/string.hpp"

// static void setStyles()
// {
//     GraphicsViewStyle::setStyle(
//         R"(
//   {
//     "GraphicsViewStyle": {
//       "BackgroundColor": [255, 255, 240],
//       "FineGridColor": [245, 245, 230],
//       "CoarseGridColor": [235, 235, 220]
//     }
//   }
//   )");

//     NodeStyle::setNodeStyle(
//         R"(
//   {
//     "NodeStyle": {
//       "NormalBoundaryColor": "darkgray",
//       "SelectedBoundaryColor": "deepskyblue",
//       "GradientColor0": "mintcream",
//       "GradientColor1": "mintcream",
//       "GradientColor2": "mintcream",
//       "GradientColor3": "mintcream",
//       "ShadowColor": [200, 200, 200],
//       "FontColor": [10, 10, 10],
//       "FontColorFaded": [100, 100, 100],
//       "ConnectionPointColor": "white",
//       "PenWidth": 2.0,
//       "HoveredPenWidth": 2.5,
//       "ConnectionPointDiameter": 10.0,
//       "Opacity": 1.0
//     }
//   }
//   )");

//     ConnectionStyle::setConnectionStyle(
//         R"(
//   {
//     "ConnectionStyle": {
//       "ConstructionColor": "gray",
//       "NormalColor": "black",
//       "SelectedColor": "gray",
//       "SelectedHaloColor": "deepskyblue",
//       "HoveredColor": "deepskyblue",

//       "LineWidth": 3.0,
//       "ConstructionLineWidth": 2.0,
//       "PointDiameter": 10.0,

//       "UseDataDefinedColors": false
//     }
//   }
//   )");
// }

static void setStyles()
{
    ConnectionStyle::setConnectionStyle(
        R"(
  {
    "ConnectionStyle": {
      "ConstructionColor": "gray",
      "NormalColor": "black",
      "SelectedColor": "gray",
      "SelectedHaloColor": "deepskyblue",
      "HoveredColor": "deepskyblue",

      "LineWidth": 3.0,
      "ConstructionLineWidth": 2.0,
      "PointDiameter": 10.0,

      "UseDataDefinedColors": true
    }
  }
  )");
}

MainWindow_node::MainWindow_node(std::shared_ptr<NodeDelegateModelRegistry> registry, QNode* qnode, QWidget *parent) {    
    setStyles();    
    menuBar()->setStyleSheet("QMenuBar { background-color: lightgray; }");
        
    QWidget* mainWidget = new QWidget();
    setCentralWidget(mainWidget);

    fileMenu = new QMenu(tr("&File"), this);
    menuBar()->addMenu(fileMenu);
    newAction = new QAction("New Scene");
    saveAction = new QAction("Save Scene");
    loadAction = new QAction("Load Scene");
    fileMenu->addAction(newAction);
    fileMenu->addAction(saveAction);
    fileMenu->addAction(loadAction);
    

    connMenu = new QMenu(tr("&Connection"), this);
    menuBar()->addMenu(connMenu);
    tcpConnectAction = new QAction("TCP/IP connection");
    plcConnectAction = new QAction("MODBUS-TCP connection");
    connMenu->addAction(tcpConnectAction);
    connMenu->addAction(plcConnectAction);


    taskMenu = new QMenu(tr("&Task"), this);
    menuBar()->addMenu(taskMenu);
    taskAction = new QAction("task List");
    taskMenu->addAction(taskAction);

    nodeMenu = new QMenu(tr("&Node"), this);
    menuBar()->addMenu(nodeMenu);
    connectAction = new QAction("auto connect nodes in id");
    connectAction ->setIcon(QIcon(":play.png"));
    nodeMenu->addAction(connectAction);

    parameterSettingMenu = new QMenu(tr("&Setting"), this);
    menuBar()->addMenu(parameterSettingMenu);
    popupAction = new QAction("Scan Parameter Setting");
    //connectAction ->setIcon(QIcon(":play.png"));
    parameterSettingMenu->addAction(popupAction);

    //add toolbar
    toolbar = new QToolBar(this);
    toolbar->setAllowedAreas(Qt::TopToolBarArea);             
    toolbar->setToolTip(tr("Task setting menu (click right mouse button on task view)"));

    QAction *act[4];
    /*act[0] = QSharedPointer<QAction>
            (new QAction(QIcon("c:\\qt_icon\\e.jpg"),"Browser",this));*/

    act[0] = new QAction(QIcon(":items/move.svg"), "Move", this);
    act[1] = new QAction(QIcon(":items/scan.svg"),"AI Scan",this);
    act[2] = new QAction(QIcon(":items/gripper.svg"),"Gripper",this);
    act[3] = new QAction(QIcon(":items/palletizing.svg"),"Palletizing",this);

    //act[0]->setShortcut(Qt::Key_Control | Qt::Key_E);       //단축키
    act[0]->setToolTip(tr("Robot Move Task"));   
    act[1]->setToolTip(tr("Camera Scan & AI Task"));   
    act[2]->setToolTip(tr("Gripper Task"));   
    act[3]->setToolTip(tr("Palletizing Task"));       

    toolbar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);//텍스트가 아이콘 옆에 나옴

    for(int i=0;i<4;i++)
    {
        toolbar->addAction(act[i]); //액션 추가
    }

    this->addToolBar(toolbar);

    QObject::connect(act[0], &QAction::triggered, [&]() {        

    });

    QVBoxLayout* l = new QVBoxLayout(mainWidget);
    dataFlowGraphModel = new DataFlowGraphModel(registry);

    // Initialize and connect two nodes.
    {
        NodeId id1 = dataFlowGraphModel->addNode("TASK START");
        dataFlowGraphModel->setNodeData(id1, NodeRole::Position, QPointF(300, 300));
    
        NodeId id2 = dataFlowGraphModel->addNode("POPUP MESSAGE");
        dataFlowGraphModel->setNodeData(id2, NodeRole::Position, QPointF(600, 300));

        // dataFlowGraphModel->setNodeData(id2, NodeRole::Style, 
        
   
        //        R"(
        //             {
        //                 "NodeStyle": {
        //                 "NormalBoundaryColor": "darkgray",
        //                 "SelectedBoundaryColor": "deepskyblue",
        //                 "GradientColor0": "mintcream",
        //                 "GradientColor1": "mintcream",
        //                 "GradientColor2": "mintcream",
        //                 "GradientColor3": "mintcream",
        //                 "ShadowColor": [10, 10, 10],
        //                 "FontColor": [10, 10, 10],
        //                 "FontColorFaded": [10, 10, 10],
        //                 "ConnectionPointColor": "white",
        //                 "PenWidth": 2.0,
        //                 "HoveredPenWidth": 2.5,
        //                 "ConnectionPointDiameter": 10.0,
        //                 "Opacity": 1.0
        //                 }
        //             }
        //         )"
        
        // );
      
        dataFlowGraphModel->addConnection(ConnectionId{id1, 0, id2, 0});
    }

    //set node style for each category
    // auto registry = dataFlowGraphModel->dataModelRegistry();

    // for (auto const &cat : registry->categories()) {
    //     auto item = new QTreeWidgetItem(treeView);
    //     item->setText(0, cat);
    //     item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    // }


    scene = new DataFlowGraphicsScene(*dataFlowGraphModel, mainWidget);
    view = new GraphicsView(scene);
    l->addWidget(view);

    // auto w_stick = scene->createModelStickVMenu();
    // w_stick->show();
    // w_stick->setParent(this);
    // w_stick->resize(100, 250);
    // w_stick->move(10, 50);
    // w_stick->show();
    
    //set vertical 2d image menu

    l->setContentsMargins(0, 0, 0, 0);
    l->setSpacing(0);

    QObject::connect(newAction, &QAction::triggered, [this]() {      
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "New Scene", "Do you want reset the Scene?",
                                        QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            scene->clearScene();
        } 
    });

    QObject::connect(saveAction, &QAction::triggered, scene, &DataFlowGraphicsScene::save);
    QObject::connect(loadAction, &QAction::triggered, scene, &DataFlowGraphicsScene::load);

    _taskManager = new TaskManager(*dataFlowGraphModel, qnode);
    QObject::connect(scene, &DataFlowGraphicsScene::sceneLoaded, view, &GraphicsView::centerScene);
    QObject::connect(taskAction, &QAction::triggered, [&]() {        
        if(_taskManager->isVisible()) {
            _taskManager->activateWindow();
        } else {
            _taskManager->show();
            _taskManager->GenTaskScriptFromGraph();
        }
    });

    
    dockWidget = new QDockWidget(tr("Data View"), this);
    dockWidget->setFeatures(QDockWidget::DockWidgetFloatable);
    //NoDockWidgetArea
    dockWidget->setAllowedAreas(Qt::RightDockWidgetArea);

    QLabel* default_label = new QLabel(tr("no Data required"));
    dockWidget->setWidget(default_label);
    dockWidget->setStyleSheet("::title {background: darkGray;}");

    //QRect currentUISize = this->frameGeometry();
    //dockWidget->setContentsMargins(0,0,0,0);
    //dockWidget->setGeometry(this->frameGeometry());

    // dockWidget->setGeometry(QRect(
    //                             QPoint(0,0), //top-left
    //                             QSize(
    //                                 this->frameGeometry().right()/2, 
    //                                 this->frameGeometry().bottom())));


    //dockWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    this->addDockWidget(Qt::RightDockWidgetArea , dockWidget);
    dockWidget->setFloating(false);   
    // float windowHeight = this->size().height()/2;
    // int dockHeight = 0.60 * windowHeight;
    // this->resizeDocks({dockWidget}, {dockHeight}, Qt::Vertical);
    
    QObject::connect(scene, &DataFlowGraphicsScene::nodeClicked, dockWidget, [&, default_label](NodeId nodeId) {        
        if (auto w =  dataFlowGraphModel->nodeData(nodeId, NodeRole::Widget).value<QWidget *>()) {      
            //dockWidget->layout()->setSizeConstraint(QLayout::SetMinimumSize);
            dockWidget->setWidget(w);
            dockWidget->widget()->layout()->setSizeConstraint(QLayout::SetFixedSize);
            dockWidget->update();
        } else {
            dockWidget->setWidget(default_label);
            dockWidget->update();
        }
    });

    QObject::connect(connectAction, &QAction::triggered, [&]() {
        std::unordered_set<NodeId> nodeSet = dataFlowGraphModel->allNodeIds();
        for (auto it = nodeSet.begin(); it != nodeSet.end() && std::next(it) != nodeSet.end(); ++it ) { // = std::next(it, 2)
            ConnectionId connId = ConnectionId{*it, 0, *std::next(it), 0};
            if(dataFlowGraphModel->connectionPossible(connId)) {
                dataFlowGraphModel->addConnection(connId);
            }
        }
    });

    // scan_dialog = new ScanningDialog(qnode);
    // QObject::connect(popupAction, &QAction::triggered, [&]() {
    //     if(scan_dialog->isVisible()) {
    //         scan_dialog->activateWindow();
    //     } else {
    //         scan_dialog->show();
    //     }
    // });

    

    // TODOTODO 
    // QObject::connect(dataFlowGraphModel, &DataFlowGraphModel::triggered, [&]() {
    //     if(scan_dialog->isVisible()) {
    //         scan_dialog->activateWindow();
    //     } else {
    //         scan_dialog->show();
    //     }
    // });

//    dockWidget = new QDockWidget("Docked Widget");

    //해당 노드를 클릭시 상세 설정 창이 뜨도록하려면 -> 노드 선택시 노드아이디에 의한 내용 dockContent에 업데이트 후 visible //  언노드 시 invisible
//     auto dockWidget = new QDockWidget();
//     QLabel *dockLabel = new QLabel("Node Detail");
//     dockWidget->setLayoutDirection(Qt::LayoutDirection::LeftToRight);
//     dockWidget->layout()->addWidget(dockLabel);
//     QWidget* transferredWidget = new QWidget();
//     dockWidget->layout()->addWidget(transferredWidget);
//     //dockWidget->setWidget(dockContent);
//     dockWidget->setAllowedAreas(Qt::RightDockWidgetArea);
//     dockWidget->setStyleSheet("QDockWidget { background-color: lightgray; }");
//     //this->addDockWidget(Qt::RightDockWidgetArea, dockWidget);
//     dockWidget->setAttribute(Qt::WA_TranslucentBackground, true);

//    QObject::connect(scene, &DataFlowGraphicsScene::nodeSelected, dockWidget, [&](NodeId const nodeId){
//        if (auto w = dataFlowGraphModel->nodeData(nodeId, NodeRole::Widget).value<QWidget *>()) {
//             //QGraphicsProxyWidget* pwidget = w->graphicsProxyWidget();
                
//             QList<QGraphicsItem*> items = scene->items();
//             for (QGraphicsItem* item : items) {            
//                 QGraphicsProxyWidget* proxyWidget;
//                 if (proxyWidget = qgraphicsitem_cast<QGraphicsProxyWidget *>(item)) {
//                     proxyWidget->setWidget(transferredWidget);
//                 }         

//            }
//        }
//        //dockWidget->setWidget(transferredWidget);
//    });
//    dockWidget->show();
    setWindowTitle(tr("BP-GUI"));
    //setCentralWidget(&mainWidget);
    //mainWidget->show();

    // showNormal();
}

void MainWindow_node::about()
{
    QMessageBox::about(this, tr("About Simple Text Viewer"),
                       tr("This example demonstrates how to use\n"
                          "Qt Assistant as help system for your\n"
                          "own application."));
}

void MainWindow_node::showDocumentation()
{
    //assistant->showDocumentation("index.html");
}


//void MainWindow_node::open()
//{
////    FindFileDialog dialog(textViewer, assistant);
////    dialog.exec();
//}
QWidget* MainWindow_node::get_widget_content(QWidget* widget) {
    QWidget* content = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout();
    content->setLayout(layout);
    for (QObject* child : widget->children()) {
        if (dynamic_cast<QGroupBox *>(child)) {
            QGroupBox* groupBox = static_cast<QGroupBox*>(child);

            QList<QWidget*> widgetList;
            for (QObject* object : groupBox->children()) {
                QWidget* widget = qobject_cast<QWidget*>(object);
                if (widget) {
                    widgetList.append(widget);
                }
            }
            for (QWidget *widget : widgetList) {
                if (dynamic_cast<QLineEdit*>(widget)) {
                    QLineEdit* line_edit = static_cast<QLineEdit*>(child);
                    QLineEdit* new_line_edit = new QLineEdit(content);
                    //new_line_edit->setText(line_edit->text());
                    new_line_edit->setObjectName("new_line_edit");
                    content->layout()->addWidget(new_line_edit);
                } else if (dynamic_cast<QTextEdit*>(widget)) {
                    QTextEdit* text_edit = static_cast<QTextEdit*>(child);
                    QTextEdit* new_text_edit = new QTextEdit(content);
                    //new_text_edit->setPlainText(text_edit->toPlainText());
                    new_text_edit->setObjectName("new_text_edit");
                    content->layout()->addWidget(new_text_edit);
                }
            }
        }
    }
    return content;
}

TaskManager* MainWindow_node::getTaskManager() {
    return _taskManager;
}