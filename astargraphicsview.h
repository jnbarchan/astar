#ifndef ASTARGRAPHICSVIEW_H
#define ASTARGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QObject>

class AStarGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit AStarGraphicsView(QWidget *parent = nullptr);
    explicit AStarGraphicsView(QGraphicsScene *scene, QWidget *parent = nullptr);

    void setSceneOverride(QGraphicsScene *scene);

    // QGraphicsView interface
protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void fitSceneToView();

private:
    void sceneSet();
};

#endif // ASTARGRAPHICSVIEW_H
