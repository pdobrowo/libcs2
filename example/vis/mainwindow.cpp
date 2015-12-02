#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "renderview.h"
#include "cs2/predg3f.h"
#include "cs2/spin3f.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    RenderView *rv = new RenderView();
    rv->setCaption("cspace");
    ui->widgetView->layout()->addWidget(rv);

    // triangle list
    TriangleListPtr triangles(new TriangleList());

    // add predicate g
    predg3f_t pred = { { 1, 2, 3 }, { -1, 0, 2 }, { 4, 2, -2 }, { 0, -2, 3 }, 1 };

    predgparam3f_t param;
    predg3f_param(&param, &pred);

    for (double pu = 0; pu < 1; pu += 0.01) for (double pv = 0; pv < 1; pv += 0.01)
    {
        spin3f_t sp00, sp01, sp10, sp11;

        predgparam3f_eval(&sp00, &param, pu, pv);
        predgparam3f_eval(&sp01, &param, pu, pv + 0.01);
        predgparam3f_eval(&sp10, &param, pu + 0.01, pv);
        predgparam3f_eval(&sp11, &param, pu + 0.01, pv + 0.01);

        triangles->push_back(Triangle(QVector3D(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0)),
                                      QVector3D(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0)),
                                      QVector3D(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0))));

        triangles->push_back(Triangle(QVector3D(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0)),
                                      QVector3D(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0)),
                                      QVector3D(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0))));
    }

    rv->addTriangleList(triangles, Qt::green);
}

MainWindow::~MainWindow()
{
    delete ui;
}
