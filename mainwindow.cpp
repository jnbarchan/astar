#include <QBoxLayout>
#include <QElapsedTimer>

#include "astar.h"
#include "settingsdialog.h"

#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QLayout *central_layout = new QVBoxLayout;
    ui->centralwidget->setLayout(central_layout);

    grsc = new AStarGraphicsScene(this);
    grvw = new AStarGraphicsView(grsc, this);
    central_layout->addWidget(grvw);

    connect(ui->actionSettings, &QAction::triggered, this, &MainWindow::actionSettings);
    connect(ui->actionRun, &QAction::triggered, this, &MainWindow::actionRun);
    connect(ui->actionRun_Animation, &QAction::triggered, this, &MainWindow::actionRun_Animation);
    connect(ui->actionExit, &QAction::triggered, this, &MainWindow::actionExit);

    settings.x_coord_size = 20;
    settings.y_coord_size = 20;
    settings.start_point = QPoint(1, 1);
    settings.goal_point = QPoint(settings.x_coord_size - 2, settings.x_coord_size - 2);
    settings.edge_length = 1000;
    settings.node_selector_method = AStar::NodeSelectorLowestPriorityMap;
    settings.node_selector_heuristic_method = AStar::NodeSelectorHeuristicEuclideanWeighted;

    setAStarSceneFromSettings();

    connect(&aStar, &AStar::findPathAsyncStarted, this, [this]() { setRunning_async(true); } );
    connect(&aStar, &AStar::findPathAsyncStopped, this, [this]() { setRunning_async(false); } );
    setRunning_async(false);
}

MainWindow::~MainWindow()
{
    delete grvw;
    delete grsc;

    delete ui;
}

void MainWindow::setAStarSceneFromSettings()
{
    grsc->clear();

    QRect scene_rect(0, 0, settings.x_coord_size, settings.y_coord_size);
    grsc->setSceneRect(scene_rect);

    QGraphicsItem *start = grsc->addEllipse(0.1, 0.1, 0.8, 0.8, QPen(Qt::NoPen), QBrush(Qt::green));
    start->setPos(settings.start_point);
    QGraphicsItem *goal = grsc->addEllipse(0.1, 0.1, 0.8, 0.8, QPen(Qt::NoPen), QBrush(Qt::red));
    goal->setPos(settings.goal_point);
}


void MainWindow::actionSettings()
{
    SettingsDialog dlg(settings, this);

    if (!dlg.exec())
        return;

    setAStarSceneFromSettings();
}

void MainWindow::actionRun()
{
    setRunning_async(false);
    do_aStar_sync();
}

void MainWindow::actionRun_Animation()
{
    if (running_async())
    {
        // Stop
        setRunning_async(false);
        return;
    }
    // Run
    setRunning_async(true);
    do_aStar_async();
}

void MainWindow::actionExit()
{
    QApplication::quit();
}



bool MainWindow::running_async() const
{
    return _running_async;
}

void MainWindow::setRunning_async(bool newRunning_async)
{
    bool was_running_async = _running_async;
    _running_async = newRunning_async;
    if (was_running_async && !newRunning_async)
        aStar.cancel_find_path_async();
    ui->actionRun_Animation->setText(_running_async ? "Stop" : "Run Animation");
}

void MainWindow::do_aStar_init(bool show_progress)
{
    aStar.setShow_progress(show_progress);
    aStar.set_coord_sizes(settings.x_coord_size, settings.y_coord_size);
    aStar.set_edge_length(settings.edge_length);
    aStar.set_node_selector_method(settings.node_selector_method);
    aStar.set_node_selector_heuristic_method(settings.node_selector_heuristic_method);

    setAStarSceneFromSettings();

    qDebug() << "_________________________________________________________________________________________________________________";
    qDebug() << settings.x_coord_size << "x" << settings.y_coord_size
             << "," << settings.start_point << "->" << settings.goal_point
             << "," << settings.node_selector_method << "," << settings.node_selector_heuristic_method << "," << settings.edge_length;
}

void MainWindow::do_aStar_sync()
{
    do_aStar_init(false);
    disconnect(&aStar, &AStar::nodeStatusChanged, this, &MainWindow::onAStarNodeStatusChanged);

    QElapsedTimer elapsed;
    elapsed.start();

    NodeList path = aStar.find_path(settings.start_point, settings.goal_point);

    qDebug() << "path:" << "length:" << path.length();
    // for (const Node &node : path)
    //     qDebug() << node.coords;
    // qDebug() << "endpath.";

    qDebug() << "Elapsed time:" << elapsed.elapsed() << "msecs";
}

void MainWindow::do_aStar_async()
{
    do_aStar_init(true);
    connect(&aStar, &AStar::nodeStatusChanged, this, &MainWindow::onAStarNodeStatusChanged, Qt::UniqueConnection);

    aStar.find_path_async(settings.start_point, settings.goal_point);
}


void MainWindow::onAStarNodeStatusChanged(Node node, AStar::NodeState state)
{
    AStarGraphicsNodeItem *current = qgraphicsitem_cast<AStarGraphicsNodeItem *>(grsc->itemAt(node.coords, QTransform()));
    switch (state)
    {
    case AStar::StateRemoved:
        if (current != nullptr)
            grsc->removeItem(current);
        break;
    case AStar::StateCurrent:
    case AStar::StateOpen:
    case AStar::StateClosed:
    case AStar::StatePath:
        if (current == nullptr)
        {
            current = new AStarGraphicsNodeItem(0.1, 0.1, 0.8, 0.8);
            current->setPen(Qt::NoPen);
            grsc->addItem(current);
        }
        QColor color;
        switch (state)
        {
        case AStar::StateCurrent: color = Qt::yellow; break;
        case AStar::StateOpen: color = Qt::cyan; break;
        case AStar::StateClosed: color = Qt::gray; break;
        case AStar::StatePath: color = Qt::red; break;
        default: Q_ASSERT(false); break;
        }

        current->setBrush(color);
        current->setPos(node.coords);
        break;
    }
}
