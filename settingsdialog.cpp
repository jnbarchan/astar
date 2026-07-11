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

    ui->edge_length->setValue(settings.edge_length);
    ui->animation_delay->setValue(settings.animation_delay);
    connect(ui->animation_delay, &QSlider::valueChanged, this, &SettingsDialog::onAnimationDelayValueChanged);
    onAnimationDelayValueChanged(ui->animation_delay->value());

    ui->node_selector_method->clear();
    ui->node_selector_method->addItem("node_first_f_score", AStar::NodeSelectorFirst);
    ui->node_selector_method->addItem("node_lowest_sequential_f_score", AStar::NodeSelectorLowestSequential);
    ui->node_selector_method->addItem("node_lowest_priority_map_f_score", AStar::NodeSelectorLowestPriorityMap);
    int index = ui->node_selector_method->findData(settings.node_selector_method);
    if (index != -1)
        ui->node_selector_method->setCurrentIndex(index);

    ui->node_selector_heuristic_method->clear();
    ui->node_selector_heuristic_method->addItem("heuristic_dijkstra", AStar::NodeSelectorHeuristicDijkstra);
    ui->node_selector_heuristic_method->addItem("heuristic_manhattan", AStar::NodeSelectorHeuristicManhattan);
    ui->node_selector_heuristic_method->addItem("heuristic_euclidean", AStar::NodeSelectorHeuristicEuclidean);
    ui->node_selector_heuristic_method->addItem("heuristic_euclidean_weighted", AStar::NodeSelectorHeuristicEuclideanWeighted);
    index = ui->node_selector_heuristic_method->findData(settings.node_selector_heuristic_method);
    if (index != -1)
        ui->node_selector_heuristic_method->setCurrentIndex(index);

    connect(ui->x_coord_size, &QSpinBox::editingFinished, this, [this]() { int x = ui->x_coord_size->value() - 1; ui->start_point_x->setMaximum(x); ui->goal_point_x->setMaximum(x); } );
    connect(ui->y_coord_size, &QSpinBox::editingFinished, this, [this]() { int y = ui->y_coord_size->value() - 1; ui->start_point_y->setMaximum(y); ui->goal_point_y->setMaximum(y); } );
    connect(ui->node_selector_method, &QComboBox::currentIndexChanged, this, &SettingsDialog::onNodeSelectorMethodChanged);

    onNodeSelectorMethodChanged();
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
    settings.edge_length = ui->edge_length->value();
    settings.animation_delay = ui->animation_delay->value();
    settings.node_selector_method = static_cast<AStar::NodeSelectorMethod>(ui->node_selector_method->currentData().toInt());
    settings.node_selector_heuristic_method = static_cast<AStar::NodeSelectorHeuristicMethod>(ui->node_selector_heuristic_method->currentData().toInt());

    QDialog::accept();      // Closes dialog and emits accepted()
}

void SettingsDialog::onAnimationDelayValueChanged(int value)
{
    ui->animation_delay->setToolTip(QString::number(value));
}

void SettingsDialog::onNodeSelectorMethodChanged()
{
    ui->node_selector_heuristic_method->setEnabled(static_cast<AStar::NodeSelectorMethod>(ui->node_selector_method->currentData().toInt()) == AStar::NodeSelectorLowestPriorityMap);
}
