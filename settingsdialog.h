#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QDialog>

#include "astar.h"

namespace Ui {
class SettingsDialog;
}

class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    struct Settings
    {
        int x_coord_size, y_coord_size;
        QPoint start_point, goal_point;
        int edge_length;
        AStar::NodeSelectorMethod node_selector_method;
        AStar::NodeSelectorHeuristicMethod node_selector_heuristic_method;
        int animation_delay;
    };

    explicit SettingsDialog(Settings &settings, QWidget *parent = nullptr);
    ~SettingsDialog();

protected:
    void accept() override;

private slots:
    void onAnimationDelayValueChanged(int value);
    void onNodeSelectorMethodChanged();

private:
    Ui::SettingsDialog *ui;

    Settings &settings;
};

#endif // SETTINGSDIALOG_H
