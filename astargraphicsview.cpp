#include "astargraphicsview.h"

AStarGraphicsView::AStarGraphicsView(QGraphicsScene *scene, QWidget *parent)
    : QGraphicsView{scene, parent}
{
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    connect(scene, &QGraphicsScene::sceneRectChanged, this, [this](){ fitInView(sceneRect(), Qt::KeepAspectRatio); });
}

void AStarGraphicsView::resizeEvent(QResizeEvent *event)
{
    fitInView(sceneRect(), Qt::KeepAspectRatio);
}
