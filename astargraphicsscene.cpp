#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>

#include "astargraphicsscene.h"

AStarGraphicsScene::AStarGraphicsScene(QObject *parent)
    : QGraphicsScene{parent}
{
    _startDragItem = nullptr;
}

QList<QGraphicsItem *> AStarGraphicsScene::itemsOfType(int type)
{
    QList<QGraphicsItem *> _itemsOfType;
    for (QGraphicsItem *item : items())
        if (item->type() == type)
            _itemsOfType.append(item);
    return _itemsOfType;
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

void AStarGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPointF scenePosF = event->scenePos();
        QPoint scenePos(scenePosF.x(), scenePosF.y());
        if (sceneRect().contains(scenePos))
        {
            QGraphicsItem *item = itemAt(scenePos + QPointF{0.5, 0.5}, QTransform());
            _startDragItem = item;
            if (item == nullptr || !(item->flags() & QGraphicsItem::ItemIsMovable))
            {
                _startDragItem = nullptr;
                emit aStarMouseClicked(scenePos);
                return;
            }
        }
    }
    QGraphicsScene::mousePressEvent(event);
}

void AStarGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    if (_startDragItem)
    {
        emit aStarItemDragged(_startDragItem);
        _startDragItem = nullptr;
    }
    QGraphicsScene::mouseReleaseEvent(event);
}
