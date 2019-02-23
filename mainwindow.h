#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "pid.h"
#include <QTimer>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QTimer *timer;

    double dt =0.01;
    double T=0.0;
    pid p;
    double old;


private slots:
    void on_horizontalSlider_sliderMoved(int position);
    void update();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
