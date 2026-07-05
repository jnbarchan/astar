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

    connect(ui->actionSettings, &QAction::triggered, this, &MainWindow::actionSettings);
    connect(ui->actionRun, &QAction::triggered, this, &MainWindow::actionRun);
    connect(ui->actionExit, &QAction::triggered, this, &MainWindow::actionExit);

    settings.x_coord_size = 50;
    settings.y_coord_size = 50;
    settings.node_selector_method = AStar::NodeSelectorLowestPriorityMap;
    settings.start_point = QPoint(1, 1);
    settings.goal_point = QPoint(48, 48);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::actionSettings()
{
    SettingsDialog dlg(settings, this);

    if (!dlg.exec())
        return;
}

void MainWindow::actionRun()
{
    do_aStar();
}

void MainWindow::actionExit()
{
    QApplication::quit();
}

void MainWindow::do_aStar()
{
    AStar aStar;
    aStar.set_coord_sizes(settings.x_coord_size, settings.y_coord_size);
    aStar.set_node_selector_method(settings.node_selector_method);

    qDebug() << "_________________________________________________________________________________________________________________";
    qDebug() << settings.x_coord_size << "x" << settings.y_coord_size
             << "," << settings.start_point << "->" << settings.goal_point
             << "," << settings.node_selector_method;

    QElapsedTimer elapsed;
    elapsed.start();

    QList<Node> path = aStar.find_path(settings.start_point, settings.goal_point);

    qDebug() << "path:" << "length:" << path.length();
    // for (const Node &node : path)
    //     qDebug() << node.coords;
    // qDebug() << "endpath.";

    qDebug() << "Elapsed time:" << elapsed.elapsed() << "msecs";
}

