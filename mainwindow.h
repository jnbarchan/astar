#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsView>
#include <QMainWindow>

#include "astargraphicsscene.h"
#include "astargraphicsview.h"
#include "settingsdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    bool running_async() const;
    void setRunning_async(bool newRunning_async);

private slots:
    void actionSettings();
    void actionRun();
    void actionRun_Animation();
    void actionExit();
    void onAStarNodeStatusChanged(Node node, AStar::NodeState state);
    void onAStarSceneMouseClicked(const QPoint &coords);
    void onAStarSceneItemDragged(QGraphicsItem *item);

private:
    Ui::MainWindow *ui;

    AStarGraphicsScene *grsc;
    AStarGraphicsView *grvw;
    SettingsDialog::Settings settings;

    bool _running_async;

    AStar aStar;

    void initAStarScene();
    void clearAStarSceneBlocks();
    void removeAStarSceneBlock(const QPoint &coords);
    void addAStarSceneBlock(const QPoint &coords);
    void setAStarSceneFromSettings();
    void do_aStar_init(bool show_progress);
    void do_aStar_sync();
    void do_aStar_async();
};
#endif // MAINWINDOW_H
