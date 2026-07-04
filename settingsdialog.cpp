#include "settingsdialog.h"
#include "ui_settingsdialog.h"

SettingsDialog::SettingsDialog(Settings &settings, QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::SettingsDialog)
    , settings(settings)
{
    ui->setupUi(this);

    ui->x_coord_size->setValue(settings.x_coord_size);
    ui->y_coord_size->setValue(settings.y_coord_size);
    ui->start_point_x->setValue(settings.start_point.x());
    ui->start_point_y->setValue(settings.start_point.y());
    ui->goal_point_x->setValue(settings.goal_point.x());
    ui->goal_point_y->setValue(settings.goal_point.y());
    ui->node_selector_method->clear();
    ui->node_selector_method->addItem("node_first_f_score", AStar::NodeSelectorFirst);
    ui->node_selector_method->addItem("node_lowest_f_score", AStar::NodeSelectorLowest);
    int index = ui->node_selector_method->findData(settings.node_selector_method);
    if (index != -1)
        ui->node_selector_method->setCurrentIndex(index);
}

SettingsDialog::~SettingsDialog()
{
    delete ui;
}

void SettingsDialog::accept()
{
    settings.x_coord_size = ui->x_coord_size->value();
    settings.y_coord_size = ui->y_coord_size->value();
    settings.start_point.setX(ui->start_point_x->value());
    settings.start_point.setY(ui->start_point_y->value());
    settings.goal_point.setX(ui->goal_point_x->value());
    settings.goal_point.setY(ui->goal_point_y->value());
    settings.node_selector_method = static_cast<AStar::NodeSelectorMethod>(ui->node_selector_method->currentData().toInt());

    QDialog::accept();      // Closes dialog and emits accepted()
}
