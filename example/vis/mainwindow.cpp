#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "renderview.h"
#include "renderviewarcballcamera.h"
#include "renderviewflycamera.h"
#include "renderviewautocamera.h"
#include "cs2/predg3f.h"
#include "cs2/spin3f.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    m_rv = new RenderView();

    m_rv->setCaption("S^4->(st.proj.)->R^3");
    m_rv->setCullingEnabled(true);

    ui->widgetView->layout()->addWidget(m_rv);

    // triangle list
    TriangleListPtr trianglesFront(new TriangleList());
    TriangleListPtr trianglesBack(new TriangleList());

    // add predicate g
    predg3f_t pred = { { 1, 2, 3 }, { -1, 0, 2 }, { 4, 2, -2 }, { 0, -2, 3 }, 1 };

    predgparam3f_t param;
    predg3f_param(&param, &pred);

    const static double STEP = 0.02;

    for (double pu = 0; pu < 1; pu += STEP) for (double pv = 0; pv < 1; pv += STEP)
    {
        spin3f_t sp00, sp01, sp10, sp11;

        predgparam3f_eval(&sp00, &param, pu, pv);
        predgparam3f_eval(&sp01, &param, pu, pv + STEP);
        predgparam3f_eval(&sp10, &param, pu + STEP, pv);
        predgparam3f_eval(&sp11, &param, pu + STEP, pv + STEP);

        QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
        QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
        QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
        QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

        trianglesFront->push_back(Triangle(v00, v01, v11));
        trianglesFront->push_back(Triangle(v00, v11, v10));

        trianglesBack->push_back(Triangle(v00, v11, v01));
        trianglesBack->push_back(Triangle(v00, v10, v11));
    }

    m_rv->addTriangleList(trianglesFront, Qt::green);
    m_rv->addTriangleList(trianglesBack, Qt::red);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionArcballCamera_triggered()
{
    m_rv->setRenderViewCamera(new RenderViewArcBallCamera());
}

void MainWindow::on_actionFreeCamera_triggered()
{
    m_rv->setRenderViewCamera(new RenderViewFlyCamera());
}

void MainWindow::on_actionAutoCamera_triggered()
{
    m_rv->setRenderViewCamera(new RenderViewAutoCamera());
}
