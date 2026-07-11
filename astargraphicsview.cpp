#include <QGraphicsItem>
#include <QMouseEvent>

#include "astargraphicsview.h"

AStarGraphicsView::AStarGraphicsView(QWidget *parent)
    : AStarGraphicsView(nullptr, parent)
{
}

AStarGraphicsView::AStarGraphicsView(QGraphicsScene *scene, QWidget *parent)
    : QGraphicsView{scene, parent}
{
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    sceneSet();
}

void AStarGraphicsView::setSceneOverride(QGraphicsScene *scene)
{
    setScene(scene);
    sceneSet();
}

void AStarGraphicsView::sceneSet()
{
    if (scene())
        connect(scene(), &QGraphicsScene::sceneRectChanged, this, &AStarGraphicsView::fitSceneToView, Qt::UniqueConnection);
}

void AStarGraphicsView::fitSceneToView()
{
    fitInView(sceneRect(), Qt::KeepAspectRatio);;
}

void AStarGraphicsView::resizeEvent(QResizeEvent *event)
{
    Q_UNUSED(event);
    fitSceneToView();
}
