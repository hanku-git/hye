#include "DataFlowGraphicsScene.hpp"

#include "ConnectionGraphicsObject.hpp"
#include "GraphicsView.hpp"
#include "NodeDelegateModelRegistry.hpp"
#include "NodeGraphicsObject.hpp"
#include "UndoCommands.hpp"

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QGraphicsSceneMoveEvent>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QWidgetAction>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGroupBox>
#include <QtGui/QPainter>
#include <QtCore/QBuffer>
#include <QtCore/QByteArray>
#include <QtCore/QDataStream>
#include <QtCore/QDebug>
#include <QtCore/QFile>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QtGlobal>

#include <stdexcept>
#include <utility>

namespace QtNodes {
DataFlowGraphicsScene::DataFlowGraphicsScene(DataFlowGraphModel &graphModel, QObject *parent)
    : BasicGraphicsScene(graphModel, parent)
    , _graphModel(graphModel)
{
    connect(&_graphModel,
            &DataFlowGraphModel::inPortDataWasSet,
            [this](NodeId const nodeId, PortType const, PortIndex const) { onNodeUpdated(nodeId); });

    connect(&_graphModel,
            &DataFlowGraphModel::zoomInNode,
            [this](NodeId const nodeId) { onZoomInNode(nodeId); });
}

// TODO constructor for an empyt scene?

std::vector<NodeId> DataFlowGraphicsScene::selectedNodes() const
{
    QList<QGraphicsItem *> graphicsItems = selectedItems();

    std::vector<NodeId> result;
    result.reserve(graphicsItems.size());

    for (QGraphicsItem *item : graphicsItems) {
        auto ngo = qgraphicsitem_cast<NodeGraphicsObject *>(item);

        if (ngo != nullptr) {
            result.push_back(ngo->nodeId());
        }
    }

    return result;
}

QMenu *DataFlowGraphicsScene::createSceneMenu(QPointF const scenePos)
{
    QMenu *modelMenu = new QMenu();

    // Add filterbox to the context menu
    auto *txtBox = new QLineEdit(modelMenu);
    txtBox->setPlaceholderText(QStringLiteral("Filter"));
    txtBox->setClearButtonEnabled(true);

    auto *txtBoxAction = new QWidgetAction(modelMenu);
    txtBoxAction->setDefaultWidget(txtBox);

    // 1.
    modelMenu->addAction(txtBoxAction);

    // Add result treeview to the context menu
    QTreeWidget *treeView = new QTreeWidget(modelMenu);
    treeView->header()->close();

    auto *treeViewAction = new QWidgetAction(modelMenu);
    treeViewAction->setDefaultWidget(treeView);

    // 2.
    modelMenu->addAction(treeViewAction);

    auto registry = _graphModel.dataModelRegistry();

    for (auto const &cat : registry->categories()) {
        auto item = new QTreeWidgetItem(treeView);
        item->setText(0, cat);
        item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
    }

    for (auto const &assoc : registry->registeredModelsCategoryAssociation()) {
        QList<QTreeWidgetItem *> parent = treeView->findItems(assoc.second, Qt::MatchExactly);

        if (parent.count() <= 0)
            continue;

        auto item = new QTreeWidgetItem(parent.first());
        item->setText(0, assoc.first);
    }

    treeView->expandAll();

    connect(treeView,
            &QTreeWidget::itemClicked,
            [this, modelMenu, scenePos](QTreeWidgetItem *item, int) {
                if (!(item->flags() & (Qt::ItemIsSelectable))) {
                    return;
                }

                this->undoStack().push(new CreateCommand(this, item->text(0), scenePos));

                modelMenu->close();
            });

    //Setup filtering
    connect(txtBox, &QLineEdit::textChanged, [treeView](const QString &text) {
        QTreeWidgetItemIterator categoryIt(treeView, QTreeWidgetItemIterator::HasChildren);
        while (*categoryIt)
            (*categoryIt++)->setHidden(true);
        QTreeWidgetItemIterator it(treeView, QTreeWidgetItemIterator::NoChildren);
        while (*it) {
            auto modelName = (*it)->text(0);
            const bool match = (modelName.contains(text, Qt::CaseInsensitive));
            (*it)->setHidden(!match);
            if (match) {
                QTreeWidgetItem *parent = (*it)->parent();
                while (parent) {
                    parent->setHidden(false);
                    parent = parent->parent();
                }
            }
            ++it;
        }
    });

    // make sure the text box gets focus so the user doesn't have to click on it
    txtBox->setFocus();

    // QMenu's instance auto-destruction
    modelMenu->setAttribute(Qt::WA_DeleteOnClose);
    

    return modelMenu;
}

QWidget *DataFlowGraphicsScene::createModelStickVMenu()
{
    QWidget *modelMenu = new QWidget();
    QVBoxLayout *iconContainer = new QVBoxLayout();
    QList<QPushButton*> btnList = QList<QPushButton*>();
    btnList.append(new QPushButton(QIcon(":/items/move.svg"),"move"));
    btnList.append(new QPushButton(QIcon(":/items/delay.svg"),"delay"));
    btnList.append(new QPushButton(QIcon(":/items/halt.svg"),"halt"));
    btnList.append(new QPushButton(QIcon(":/items/loop.svg"),"loop"));
    btnList.append(new QPushButton(QIcon(":/items/screw.svg"),"screw"));
    btnList.append(new QPushButton(QIcon(":/items/cube.svg"),"cube"));
    for(auto i : btnList) {
        iconContainer->addWidget(i);
    }
    //iconContainer->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Ignored, QSizePolicy::Expanding));
    //iconContainer->set
    //modelMenu->
    //modelMenu->setStyleSheet("background-color:solid white;");

    //버튼 액션->노드 가져와서 드래깅 작업

    modelMenu->setLayout(iconContainer);
    return modelMenu;
}

void DataFlowGraphicsScene::save() const
{
    QString fileName = QFileDialog::getSaveFileName(nullptr,
                                                    tr("Open Flow Scene"),
                                                    QDir::homePath(),
                                                    tr("Flow Scene Files (*.flow)"));

    if (!fileName.isEmpty()) {
        if (!fileName.endsWith("flow", Qt::CaseInsensitive))
            fileName += ".flow";

        QFile file(fileName);
        if (file.open(QIODevice::WriteOnly)) {
            file.write(QJsonDocument(_graphModel.save()).toJson());
        }
    }
}

void DataFlowGraphicsScene::load()
{
    QString fileName = QFileDialog::getOpenFileName(nullptr,
                                                    tr("Open Flow Scene"),
                                                    QDir::homePath(),
                                                    tr("Flow Scene Files (*.flow)"));

    if (!QFileInfo::exists(fileName))
        return;

    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly))
        return;

    clearScene();

    QByteArray const wholeFile = file.readAll();

    _graphModel.load(QJsonDocument::fromJson(wholeFile).object());

    Q_EMIT sceneLoaded();
}

void DataFlowGraphicsScene::onZoomInNode(NodeId const nodeId)
{
    auto item = nodeGraphicsObject(nodeId);
    
    if (item)
    {        
        views().first()->centerOn(item);

            // QtNodes::NodeStyle style_;
            // style_.setNodeStyle(R"(
            // {
            // "NodeStyle": {
            //     "NormalBoundaryColor": "orange",
            //     "SelectedBoundaryColor": "deepskyblue",
            //     "GradientColor0": "mintcream",
            //     "GradientColor1": "mintcream",
            //     "GradientColor2": "mintcream",
            //     "GradientColor3": "mintcream",
            //     "ShadowColor": [200, 200, 200],
            //     "FontColor": [10, 10, 10],
            //     "FontColorFaded": [100, 100, 100],
            //     "ConnectionPointColor": "white",
            //     "PenWidth": 2.0,
            //     "HoveredPenWidth": 2.5,
            //     "ConnectionPointDiameter": 10.0,
            //     "Opacity": 1.0
            // }
            // }
            // )");
            // //TO DO node ID , connection ID -- get distinct item 
            // _graphModel.delegateModel<NodeDelegateModel>(nodeId)->setNodeStyle(style_);
    }
}


} // namespace QtNodes
