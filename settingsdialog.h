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
        AStar::NodeSelectorMethod node_selector_method;
    };

    explicit SettingsDialog(Settings &settings, QWidget *parent = nullptr);
    ~SettingsDialog();

protected:
    void accept() override;

private:
    Ui::SettingsDialog *ui;

    Settings &settings;
};

#endif // SETTINGSDIALOG_H
