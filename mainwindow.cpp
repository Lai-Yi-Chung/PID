#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "pid.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    ui->plot->xAxis->setLabel("t");
    ui->plot->yAxis->setLabel("position");
    ui->plot->legend->setVisible(true);
    ui->plot->xAxis->setRange(-1, 15);
    ui->plot->yAxis->setRange(-1, 4);


    p.nature_freq = 0.55;
    p.kusi = 0.7;
    p.input = 1;
    p.set(0.7);
    timer = new QTimer(this);//remember to add timer
    connect( timer,SIGNAL(timeout()) ,this ,SLOT(update()));
    timer->start(50);

    QVector<double> x(10001), y(10001);
    for (int i=0; i<10001; i++){
        T+=dt;
        x[i] = T;
        y[i] = p.run();
    }
    QPen pen;
    pen.setColor(Qt::black);

    ui->plot->addGraph();
    ui->plot->graph(0)->setData(x, y);
    ui->plot->graph(0)->setName("measurement");
    ui->plot->graph(0)->setPen(pen);
    //ui->plot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 5));
    //ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plot->replot();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::update(){
    //old = p.kusi;
    if(p.kusi != old){
        T = 0;
        p.reset();
        QVector<double> x(10001), y(10001);
        for (int i=0; i<10001; i++){
            T+=dt;
            x[i] = T;
            y[i] = p.run();
        }
        ui->plot->graph(0)->setData(x, y);
        ui->plot->replot();
        QString str;
        str = QString("%1").arg(p.kusi);
        ui->label->setText(str) ;
    }
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    old = p.kusi;//
    double pos = position;
    ui->horizontalSlider->setMinimum(0);
    ui->horizontalSlider->setMaximum(100);
    pos /= 25;
    //p.kusi = pos; const
    p.set(pos);

}
