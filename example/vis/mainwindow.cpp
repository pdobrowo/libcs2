/**
 * Copyright (c) 2015-2016 Przemysław Dobrowolski
 *
 * This file is part of the Configuration Space Library (libcs2), a library
 * for creating configuration spaces of various motion planning problems.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "renderview.h"
#include "renderviewarcballcamera.h"
#include "renderviewflycamera.h"
#include "renderviewautocamera.h"
#include <QImage>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    m_rv = new RenderView();

    m_rv->setCaption("cspace");
    m_rv->setCullingEnabled(true);

    ui->widgetView->layout()->addWidget(m_rv);

    // update slider information
    updateSliderInformation();

    // update predicate predicate
    updatePredicateInformation();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::updatePredicateInformation()
{
    // predicate g
    predg3f_t pred = {
                        { sliderToParamValue(ui->verticalSliderKX), sliderToParamValue(ui->verticalSliderKY), sliderToParamValue(ui->verticalSliderKZ) },
                        { sliderToParamValue(ui->verticalSliderLX), sliderToParamValue(ui->verticalSliderLY), sliderToParamValue(ui->verticalSliderLZ) },
                        { sliderToParamValue(ui->verticalSliderAX), sliderToParamValue(ui->verticalSliderAY), sliderToParamValue(ui->verticalSliderAZ) },
                        { sliderToParamValue(ui->verticalSliderBX), sliderToParamValue(ui->verticalSliderBY), sliderToParamValue(ui->verticalSliderBZ) },
                          sliderToParamValue(ui->verticalSliderC)
                     };

    // param g
    predgparam3f_t param;
    predg3f_param(&param, &pred);

    double a = param.a;
    double b = param.b;
    double c = param.c;

    double mi = std::min(a - b, b - a);
    double ma = std::max(a - b, b - a);

    // information -> parameters
    ui->labelKval->setText(formatVector(&pred.k));
    ui->labelLval->setText(formatVector(&pred.l));
    ui->labelAval->setText(formatVector(&pred.a));
    ui->labelBval->setText(formatVector(&pred.b));
    ui->labelCval->setText(QString::number(c, 'f', 2));

    // information -> base variables
    vec3f_t p, q, u, v;
    predg3f_pquv(&p, &q, &u, &v, &pred);

    ui->labelPval->setText(formatVector(&p));
    ui->labelQval->setText(formatVector(&q));
    ui->labelUval->setText(formatVector(&u));
    ui->labelVval->setText(formatVector(&v));
    ui->labelC2val->setText(QString::number(c, 'f', 2));

    // information -> type
    predgtype3f_t type = predg3f_type(&pred);

    ui->labelproperval->setText(type == predgtype3f_inproper ? "no" : "yes");
    ui->labeltypeval->setText(predgtype3f_str(type));

    // information -> parametrization variables
    ui->labelaval->setText(QString::number(a, 'f', 4));
    ui->labelbval->setText(QString::number(b, 'f', 4));
    ui->labelcval->setText(QString::number(c, 'f', 4));
    ui->labelminval->setText(QString::number(mi, 'f', 4));
    ui->labelmaxval->setText(QString::number(ma, 'f', 4));

    ui->labelcaseval->setText(predgparamtype3f_str(param.t));
    ui->labeldimval->setText(QString::number(predgparamtype3f_dim(param.t)));

    // information -> eigen decomposition
    mat44f_t eigenvec;
    vec4f_t eigenval;
    predg3f_eigen(&eigenvec, &eigenval, &pred);

    double l1 = eigenval.x;
    double l2 = eigenval.y;
    double l3 = eigenval.z;
    double l4 = eigenval.w;

    double det = l1 * l2 * l3 * l4;

    vec4f_t w1 = { eigenvec.m[0][0], eigenvec.m[1][0], eigenvec.m[2][0], eigenvec.m[3][0] };
    vec4f_t w2 = { eigenvec.m[0][1], eigenvec.m[1][1], eigenvec.m[2][1], eigenvec.m[3][1] };
    vec4f_t w3 = { eigenvec.m[0][2], eigenvec.m[1][2], eigenvec.m[2][2], eigenvec.m[3][2] };
    vec4f_t w4 = { eigenvec.m[0][3], eigenvec.m[1][3], eigenvec.m[2][3], eigenvec.m[3][3] };

    ui->labeldetval->setText(QString::number(det, 'f', 8));

    ui->labell1val->setText(QString::number(l1, 'f', 4));
    ui->labell2val->setText(QString::number(l2, 'f', 4));
    ui->labell3val->setText(QString::number(l3, 'f', 4));
    ui->labell4val->setText(QString::number(l4, 'f', 4));

    ui->labelw1val->setText(formatVector(&w1));
    ui->labelw2val->setText(formatVector(&w2));
    ui->labelw3val->setText(formatVector(&w3));
    ui->labelw4val->setText(formatVector(&w4));

    // visual
    m_rv->removeAllObjects();

    // only 2-dimensional
    if (predgparamtype3f_dim(param.t) == 2)
    {
        const static double STEP = 0.01;

        TriangleListPtr trianglesFront(new TriangleList());
        TriangleListPtr trianglesBack(new TriangleList());

        // each component
        int ncomps = predgparamtype3f_components(param.t);

        for (int c = 0; c < ncomps; ++c)
        {
            for (double pu = 0; pu < 1; pu += STEP) for (double pv = 0; pv < 1; pv += STEP)
            {
                spin3f_t sp00, sp01, sp10, sp11;

                predgparam3f_eval(&sp00, &param, pu, pv, c);
                predgparam3f_eval(&sp01, &param, pu, pv + STEP, c);
                predgparam3f_eval(&sp10, &param, pu + STEP, pv, c);
                predgparam3f_eval(&sp11, &param, pu + STEP, pv + STEP, c);

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
    }
}

double MainWindow::sliderToParamValue(QSlider *slider)
{
    return sliderValueToParamValue(slider->value());
}

double MainWindow::sliderValueToParamValue(double value)
{
    return 0.01 * (value - 300) / 3.0;
}

void MainWindow::formatSliderValue(QLabel *label, double value)
{
    label->setText(QString::number(sliderValueToParamValue(value), 'f', 2));
}

void MainWindow::updateSliderInformation()
{
    formatSliderValue(ui->labelSliderKX, ui->verticalSliderKX->value());
    formatSliderValue(ui->labelSliderKY, ui->verticalSliderKY->value());
    formatSliderValue(ui->labelSliderKZ, ui->verticalSliderKZ->value());

    formatSliderValue(ui->labelSliderLX, ui->verticalSliderLX->value());
    formatSliderValue(ui->labelSliderLY, ui->verticalSliderLY->value());
    formatSliderValue(ui->labelSliderLZ, ui->verticalSliderLZ->value());

    formatSliderValue(ui->labelSliderAX, ui->verticalSliderAX->value());
    formatSliderValue(ui->labelSliderAY, ui->verticalSliderAY->value());
    formatSliderValue(ui->labelSliderAZ, ui->verticalSliderAZ->value());

    formatSliderValue(ui->labelSliderBX, ui->verticalSliderBX->value());
    formatSliderValue(ui->labelSliderBY, ui->verticalSliderBY->value());
    formatSliderValue(ui->labelSliderBZ, ui->verticalSliderBZ->value());

    formatSliderValue(ui->labelSliderC, ui->verticalSliderC->value());
}

QString MainWindow::formatVector(const vec3f_t *v)
{
    return "[" + QString::number(v->x, 'f', 2) + ", " + QString::number(v->y, 'f', 2) + ", " + QString::number(v->z, 'f', 2) + "]";
}

QString MainWindow::formatVector(const vec4f_t *v)
{
    return "[" + QString::number(v->x, 'f', 2) + ", " + QString::number(v->y, 'f', 2) + ", " + QString::number(v->z, 'f', 2) + ", " + QString::number(v->w, 'f', 2) + "]";
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

void MainWindow::on_verticalSliderKX_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderKX, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderKY_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderKY, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderKZ_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderKZ, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderLX_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderLX, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderLY_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderLY, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderLZ_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderLZ, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderAX_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderAX, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderAY_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderAY, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderAZ_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderAZ, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderBX_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderBX, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderBY_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderBY, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderBZ_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderBZ, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderC_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderC, value);
    updatePredicateInformation();
}

void MainWindow::on_labelZeroK_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->verticalSliderKX->setValue(300);
    ui->verticalSliderKY->setValue(300);
    ui->verticalSliderKZ->setValue(300);
}

void MainWindow::on_labelZeroL_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->verticalSliderLX->setValue(300);
    ui->verticalSliderLY->setValue(300);
    ui->verticalSliderLZ->setValue(300);
}

void MainWindow::on_labelZeroA_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->verticalSliderAX->setValue(300);
    ui->verticalSliderAY->setValue(300);
    ui->verticalSliderAZ->setValue(300);
}

void MainWindow::on_labelZeroB_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->verticalSliderBX->setValue(300);
    ui->verticalSliderBY->setValue(300);
    ui->verticalSliderBZ->setValue(300);
}

void MainWindow::on_labelZeroC_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->verticalSliderC->setValue(300);
}

void MainWindow::on_actionWireframe_triggered()
{
    m_rv->setWireframe(ui->actionWireframe->isChecked());
}
