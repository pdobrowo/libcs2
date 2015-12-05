#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "renderview.h"
#include "renderviewarcballcamera.h"
#include "renderviewflycamera.h"
#include "renderviewautocamera.h"
#include "cs2/predg3f.h"
#include "cs2/spin3f.h"
#include <QImage>
#include <cmath>

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

    // predicate
    updatePredicate();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updatePredicate()
{
    // note: tests
    //predg3f_t pred = { { 1, 2, 3 }, { -1, 0, 2 }, { 4, 2, -2 }, { 0, -2, 3 }, 1 };
    //predg3f_t pred = { { 1, 0, 0 }, { -1, 1, 1 }, { 0, 0, -1 }, { 1, 0, 1 }, 2 };
    //predg3f_t pred = { { 1, -3, 0 }, { -1, 0, 2 }, { 0, 0, -1 }, { 1, 0, 1 }, 0 };

    // predicate g
    predg3f_t pred = {
                        { sliderToValue(ui->verticalSliderKX), sliderToValue(ui->verticalSliderKY), sliderToValue(ui->verticalSliderKZ) },
                        { sliderToValue(ui->verticalSliderLX), sliderToValue(ui->verticalSliderLY), sliderToValue(ui->verticalSliderLZ) },
                        { sliderToValue(ui->verticalSliderAX), sliderToValue(ui->verticalSliderAY), sliderToValue(ui->verticalSliderAZ) },
                        { sliderToValue(ui->verticalSliderBX), sliderToValue(ui->verticalSliderBY), sliderToValue(ui->verticalSliderBZ) },
                        sliderToValue(ui->verticalSliderC) };

    ui->labelKval->setText(QString("(%1, %2, %3)").arg(pred.k.x).arg(pred.k.y).arg(pred.k.z));
    ui->labelLval->setText(QString("(%1, %2, %3)").arg(pred.l.x).arg(pred.l.y).arg(pred.l.z));
    ui->labelAval->setText(QString("(%1, %2, %3)").arg(pred.a.x).arg(pred.a.y).arg(pred.a.z));
    ui->labelBval->setText(QString("(%1, %2, %3)").arg(pred.b.x).arg(pred.b.y).arg(pred.b.z));
    ui->labelCval->setText(QString("%1").arg(pred.c));

    m_rv->removeAllObjects();

    predgparam3f_t param;
    predg3f_param(&param, &pred);

    QImage img(100, 100, QImage::Format_ARGB32);
    QImage img2(100, 100, QImage::Format_ARGB32);

    QImage *imgs[2] = { &img, &img2 };

    const static double STEP = 0.01;

    for (int sgni = 0; sgni < 2; ++sgni)
    {
        // triangle list
        TriangleListPtr trianglesFront(new TriangleList());
        TriangleListPtr trianglesBack(new TriangleList());

        // sgn
        double sgn;
        QColor frontColor, backColor;

        if (!sgni)
        {
            sgn = -1.0;

            frontColor = Qt::red;
            backColor = Qt::green;
        }
        else
        {
            sgn = 1.0;

            frontColor = Qt::green;
            backColor = Qt::red;
        }

        for (double pu = 0; pu < 1; pu += STEP) for (double pv = 0; pv < 1; pv += STEP)
        {
            spin3f_t sp00, sp01, sp10, sp11;

            predgparam3f_eval(&sp00, &param, pu, pv, sgn);
            predgparam3f_eval(&sp01, &param, pu, pv + STEP, sgn);
            predgparam3f_eval(&sp10, &param, pu + STEP, pv, sgn);
            predgparam3f_eval(&sp11, &param, pu + STEP, pv + STEP, sgn);

            QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
            QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
            QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
            QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

            trianglesFront->push_back(Triangle(v00, v01, v11));
            trianglesFront->push_back(Triangle(v00, v11, v10));

            trianglesBack->push_back(Triangle(v00, v11, v01));
            trianglesBack->push_back(Triangle(v00, v10, v11));

            /* param vis */
            imgs[sgni]->setPixel(pu * 100, pv * 100, isnan(sp00.s0) ? 0xFFFF0000 : 0xFF00FF00);
        }

        m_rv->addTriangleList(trianglesFront, frontColor);
        m_rv->addTriangleList(trianglesBack, backColor);
    }

    ui->labelParam->setPixmap(QPixmap::fromImage(*imgs[0]));
    ui->labelParam2->setPixmap(QPixmap::fromImage(*imgs[1]));
}

double MainWindow::sliderToValue(QSlider *slider)
{
    return 0.01 * (slider->value() - 300) / 3.0;
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

void MainWindow::on_verticalSliderKX_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderKY_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderKZ_valueChanged(int value) { (void)value; updatePredicate(); }

void MainWindow::on_verticalSliderLX_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderLY_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderLZ_valueChanged(int value) { (void)value; updatePredicate(); }

void MainWindow::on_verticalSliderAX_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderAY_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderAZ_valueChanged(int value) { (void)value; updatePredicate(); }

void MainWindow::on_verticalSliderBX_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderBY_valueChanged(int value) { (void)value; updatePredicate(); }
void MainWindow::on_verticalSliderBZ_valueChanged(int value) { (void)value; updatePredicate(); }

void MainWindow::on_verticalSliderC_valueChanged(int value) { (void)value; updatePredicate(); }
