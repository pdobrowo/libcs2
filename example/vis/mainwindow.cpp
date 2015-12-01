#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "renderview.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    RenderView *rv = new RenderView();
    rv->setCaption("cspace");
    ui->widgetView->layout()->addWidget(rv);
}

MainWindow::~MainWindow()
{
    delete ui;
}
