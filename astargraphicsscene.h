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

    QList<QGraphicsItem *> itemsOfType(int type);

signals:
    void aStarMouseClicked(QPoint coords);
    void aStarItemDragged(QGraphicsItem *item);

    // QGraphicsScene interface
protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    void mousePressEvent(QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event) override;

private:
    QGraphicsItem *_startDragItem;
};


class AStarGraphicsStartItem : public QGraphicsEllipseItem
{
public:
    enum { Type = UserType + 1 };

    AStarGraphicsStartItem(qreal x, qreal y, qreal w, qreal h,
                          QGraphicsItem *parent = nullptr)
        : QGraphicsEllipseItem(x, y, w, h, parent)
    {
        setPen(Qt::NoPen);
        setBrush(Qt::green);
    }
    AStarGraphicsStartItem(QGraphicsItem *parent = nullptr)
        : AStarGraphicsStartItem(0.1, 0.1, 0.8, 0.8, parent)
    {
    }

    int type() const override { return Type; }
};


class AStarGraphicsGoalItem : public QGraphicsEllipseItem
{
public:
    enum { Type = UserType + 2 };

    AStarGraphicsGoalItem(qreal x, qreal y, qreal w, qreal h,
                          QGraphicsItem *parent = nullptr)
        : QGraphicsEllipseItem(x, y, w, h, parent)
    {
        setPen(Qt::NoPen);
        setBrush(Qt::red);
    }
    AStarGraphicsGoalItem(QGraphicsItem *parent = nullptr)
        : AStarGraphicsGoalItem(0.1, 0.1, 0.8, 0.8, parent)
    {
    }

    int type() const override { return Type; }
};


class AStarGraphicsBlockItem : public QGraphicsRectItem
{
public:
    enum { Type = UserType + 3 };

    AStarGraphicsBlockItem(qreal x, qreal y, qreal w, qreal h,
                          QGraphicsItem *parent = nullptr)
        : QGraphicsRectItem(x, y, w, h, parent)
    {
        setPen(Qt::NoPen);
        setBrush(Qt::black);
    }
    AStarGraphicsBlockItem(QGraphicsItem *parent = nullptr)
        : AStarGraphicsBlockItem(0.1, 0.1, 0.8, 0.8, parent)
    {
    }

    int type() const override { return Type; }
};


class AStarGraphicsNodeItem : public QGraphicsEllipseItem
{
public:
    enum { Type = UserType + 4 };

    AStarGraphicsNodeItem(qreal x, qreal y, qreal w, qreal h,
                          QGraphicsItem *parent = nullptr)
        : QGraphicsEllipseItem(x, y, w, h, parent)
    {
        setPen(Qt::NoPen);
    }
    AStarGraphicsNodeItem(QGraphicsItem *parent = nullptr)
        : AStarGraphicsNodeItem(0.1, 0.1, 0.8, 0.8, parent)
    {
    }

    int type() const override { return Type; }
};

#endif // ASTARGRAPHICSSCENE_H
