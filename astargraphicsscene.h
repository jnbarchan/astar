#ifndef ASTARGRAPHICSSCENE_H
#define ASTARGRAPHICSSCENE_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QObject>

class AStarGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit AStarGraphicsScene(QObject *parent = nullptr);

    // QGraphicsScene interface
protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
};


class AStarGraphicsNodeItem : public QGraphicsEllipseItem
{
public:
    enum { Type = UserType + 1 };

    AStarGraphicsNodeItem(qreal x, qreal y, qreal w, qreal h,
                          QGraphicsItem *parent = nullptr)
        : QGraphicsEllipseItem(x, y, w, h, parent)
    {
    }

    int type() const override { return Type; }
};

#endif // ASTARGRAPHICSSCENE_H
