#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "settingsdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void actionSettings();
    void actionRun();
    void actionExit();

private:
    Ui::MainWindow *ui;

    SettingsDialog::Settings settings;

    void do_aStar();
};
#endif // MAINWINDOW_H
