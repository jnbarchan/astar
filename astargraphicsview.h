#ifndef ASTARGRAPHICSVIEW_H
#define ASTARGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QObject>

class AStarGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit AStarGraphicsView(QGraphicsScene *scene, QWidget *parent = nullptr);

    // QGraphicsView interface
protected:
    void resizeEvent(QResizeEvent *event) override;
};

#endif // ASTARGRAPHICSVIEW_H
