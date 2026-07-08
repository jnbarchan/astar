#include <QDebug>
#include <QPainter>

#include "astargraphicsscene.h"

AStarGraphicsScene::AStarGraphicsScene(QObject *parent)
    : QGraphicsScene{parent}
{

}

void AStarGraphicsScene::drawBackground(QPainter *painter, const QRectF &rect)
{
    painter->save();
    painter->setPen(QPen(Qt::lightGray, 0.1));
    int w = width(), h = height();
    for (int x = 0; x <= w; x += 1)
        painter->drawLine(x, 0, x, h);
    for (int y = 0; y <= h; y += 1)
        painter->drawLine(0, y, w, y);
    painter->setPen(QPen(Qt::darkGray, 0.25));
    painter->drawRect(sceneRect());
    painter->restore();
}
