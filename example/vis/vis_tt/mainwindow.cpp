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
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QTextStream>
#include <QFile>
#include <QImage>
#include <cassert>
#include <cmath>

namespace // anonymous
{
static double CUTOFF_DISTANCE = 10;

double projectedDistance(const spin3f_t *a, const spin3f_t *b)
{
    vec3f_t pa, pb, pc;

    pa.x = a->s12 / (1.0 - a->s0);
    pa.y = a->s23 / (1.0 - a->s0);
    pa.z = a->s31 / (1.0 - a->s0);

    pb.x = b->s12 / (1.0 - b->s0);
    pb.y = b->s23 / (1.0 - b->s0);
    pb.z = b->s31 / (1.0 - b->s0);

    vec3f_sub(&pc, &pa, &pb);

    return vec3f_len(&pc);
}
} // namespace anonymous

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_currentChanged(false),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    m_rv = new RenderView();

    m_rv->setCaption("cspace");
    m_rv->setCullingEnabled(false);

    ui->widgetView->layout()->addWidget(m_rv);

    // update slider information
    updateSliderInformation();

    // update predicate predicate
    updatePredicateInformation();

    // window title
    updateWindowTitle();

    // force no-changes
    m_currentChanged = false;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updatePredicateInformation()
{
    // changed
    m_currentChanged = true;
    updateWindowTitle();

    // predicate tt
    predtt3f_t tt = { { sliderToParamValue(ui->verticalSliderKX), sliderToParamValue(ui->verticalSliderKY), sliderToParamValue(ui->verticalSliderKZ) },
                      { sliderToParamValue(ui->verticalSliderLX), sliderToParamValue(ui->verticalSliderLY), sliderToParamValue(ui->verticalSliderLZ) },
                      { sliderToParamValue(ui->verticalSliderMX), sliderToParamValue(ui->verticalSliderMY), sliderToParamValue(ui->verticalSliderMZ) },
                      { sliderToParamValue(ui->verticalSliderAX), sliderToParamValue(ui->verticalSliderAY), sliderToParamValue(ui->verticalSliderAZ) },
                      { sliderToParamValue(ui->verticalSliderBX), sliderToParamValue(ui->verticalSliderBY), sliderToParamValue(ui->verticalSliderBZ) },
                      { sliderToParamValue(ui->verticalSliderCX), sliderToParamValue(ui->verticalSliderCY), sliderToParamValue(ui->verticalSliderCZ) } };

    // decompose tt->s
    predttdecomp3f_t ttd;
    predtt3f_decomp(&ttd, &tt);

    // visual
    m_rv->removeAllObjects();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            preds3f_t *s = &ttd.s[i][j];
            predg3f_t g;

            predg3f_from_preds3f(&g, s);

            // param g
            predgparam3f_t param;
            predg3f_param(&param, &g);

            // only 2-dimensional
            if (predgparamtype3f_dim(param.t) == 2)
            {
                TriangleListPtr triangles(new TriangleList());

                if (ui->actionAutoMesh->isChecked())
                    autoMesh(triangles, &param, 0.1, 0.5, 10);
                else
                    simpleMesh(triangles, &param, 0.01);

                m_rv->addTriangleList(triangles, Qt::green);
            }
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

    formatSliderValue(ui->labelSliderMX, ui->verticalSliderMX->value());
    formatSliderValue(ui->labelSliderMY, ui->verticalSliderMY->value());
    formatSliderValue(ui->labelSliderMZ, ui->verticalSliderMZ->value());

    formatSliderValue(ui->labelSliderAX, ui->verticalSliderAX->value());
    formatSliderValue(ui->labelSliderAY, ui->verticalSliderAY->value());
    formatSliderValue(ui->labelSliderAZ, ui->verticalSliderAZ->value());

    formatSliderValue(ui->labelSliderBX, ui->verticalSliderBX->value());
    formatSliderValue(ui->labelSliderBY, ui->verticalSliderBY->value());
    formatSliderValue(ui->labelSliderBZ, ui->verticalSliderBZ->value());

    formatSliderValue(ui->labelSliderCX, ui->verticalSliderCX->value());
    formatSliderValue(ui->labelSliderCY, ui->verticalSliderCY->value());
    formatSliderValue(ui->labelSliderCZ, ui->verticalSliderCZ->value());
}

QString MainWindow::formatVector(const vec3f_t *v)
{
    return "[" + QString::number(v->x, 'f', 2) + ", " + QString::number(v->y, 'f', 2) + ", " + QString::number(v->z, 'f', 2) + "]";
}

QString MainWindow::formatVector(const vec4f_t *v)
{
    return "[" + QString::number(v->x, 'f', 2) + ", " + QString::number(v->y, 'f', 2) + ", " + QString::number(v->z, 'f', 2) + ", " + QString::number(v->w, 'f', 2) + "]";
}

bool MainWindow::handleUnsavedChanges()
{
    switch (QMessageBox::question(this, tr("Unsaved changes"), tr("Save changes to file?"), QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel))
    {
        case QMessageBox::Yes:
            if (!m_currentFile.isEmpty())
                return saveCurrentFile();
            else
                return saveNewFile();

        case QMessageBox::No:
            return true;

        case QMessageBox::Cancel:
            return false;

    default:
        assert(0);
    }
}

bool MainWindow::saveCurrentFile()
{
    // changes
    m_currentChanged = false;
    updateWindowTitle();

    // save
    QFile out(m_currentFile);

    if (out.open(QFile::WriteOnly))
    {
        //writer
        {
            QTextStream stream(&out);

            stream << ui->verticalSliderKX->value() << " ";
            stream << ui->verticalSliderKY->value() << " ";
            stream << ui->verticalSliderKZ->value();
            stream << "\r\n";
            stream << ui->verticalSliderLX->value() << " ";
            stream << ui->verticalSliderLY->value() << " ";
            stream << ui->verticalSliderLZ->value();
            stream << "\r\n";
            stream << ui->verticalSliderMX->value() << " ";
            stream << ui->verticalSliderMY->value() << " ";
            stream << ui->verticalSliderMZ->value();
            stream << "\r\n";
            stream << ui->verticalSliderAX->value() << " ";
            stream << ui->verticalSliderAY->value() << " ";
            stream << ui->verticalSliderAZ->value();
            stream << "\r\n";
            stream << ui->verticalSliderBX->value() << " ";
            stream << ui->verticalSliderBY->value() << " ";
            stream << ui->verticalSliderBZ->value();
            stream << "\r\n";
            stream << ui->verticalSliderCX->value() << " ";
            stream << ui->verticalSliderCY->value() << " ";
            stream << ui->verticalSliderCZ->value();
            stream << "\r\n";
        }

        out.close();
    }
    else
    {
        (void)QMessageBox::warning(this, tr("Save file"), tr("Failed to write: ") + m_currentFile, QMessageBox::Ok);
        return false;
    }

    return true;
}

bool MainWindow::saveNewFile()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save as"), QString(), QString("*.tri"));

    if (fileName.isEmpty())
        return false;

    if (!fileName.endsWith(".tri"))
        fileName += ".tri";

    m_currentFile = fileName;
    return saveCurrentFile();
}

void MainWindow::updateWindowTitle()
{
    QString fileName;

    if (m_currentFile.isEmpty())
        fileName = "unnamed.tri";
    else
    {
        QFileInfo fileInfo(m_currentFile);
        fileName = fileInfo.fileName();
    }

    setWindowTitle(fileName + ": vis" + (m_currentChanged ? " *" : ""));
}

void MainWindow::autoMesh(TriangleListPtr triangles, predgparam3f_t *param, double initialRadius, double targetRadius, int maxSubdivisions)
{
    int number_of_components = predgparamtype3f_components(param->t);

    for (int component = 0; component < number_of_components; ++component)
        for (double u = 0; u <= 1 - initialRadius; u += initialRadius)
            for (double v = 0; v <= 1 - initialRadius; v += initialRadius)
                autoMeshInternal(triangles, param, targetRadius, component, u, u + initialRadius, v, v + initialRadius, maxSubdivisions, 0);
}

void MainWindow::autoMeshInternal(TriangleListPtr triangles, predgparam3f_t *param, double targetRadius, int component, double minU, double maxU, double minV, double maxV, int maxSubdivisions, int subdivision)
{
    spin3f_t sp00, sp01, sp10, sp11;

    predgparam3f_eval(&sp00, param, minU, minV, component);
    predgparam3f_eval(&sp01, param, minU, maxV, component);
    predgparam3f_eval(&sp10, param, maxU, minV, component);
    predgparam3f_eval(&sp11, param, maxU, maxV, component);

    if (subdivision == maxSubdivisions || (projectedDistance(&sp00, &sp01) <= targetRadius && projectedDistance(&sp00, &sp10) <= targetRadius && projectedDistance(&sp00, &sp11) <= targetRadius &&
                                           projectedDistance(&sp01, &sp10) <= targetRadius && projectedDistance(&sp01, &sp11) <= targetRadius && projectedDistance(&sp10, &sp11) <= targetRadius))
    {
        QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
        QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
        QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
        QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

        addTriangle(triangles, Triangle(v00, v01, v11));
        addTriangle(triangles, Triangle(v00, v11, v10));
    }
    else
    {
        autoMeshInternal(triangles, param, targetRadius, component, minU, minU + 0.5 * (maxU - minU), minV, minV + 0.5 * (maxV - minV), maxSubdivisions, subdivision + 1);
        autoMeshInternal(triangles, param, targetRadius, component, minU, minU + 0.5 * (maxU - minU), minV + 0.5 * (maxV - minV), maxV, maxSubdivisions, subdivision + 1);
        autoMeshInternal(triangles, param, targetRadius, component, minU + 0.5 * (maxU - minU), maxU, minV, minV + 0.5 * (maxV - minV), maxSubdivisions, subdivision + 1);
        autoMeshInternal(triangles, param, targetRadius, component, minU + 0.5 * (maxU - minU), maxU, minV + 0.5 * (maxV - minV), maxV, maxSubdivisions, subdivision + 1);
    }
}

void MainWindow::simpleMesh(TriangleListPtr triangles, predgparam3f_t *param, double radius)
{
    int number_of_components = predgparamtype3f_components(param->t);

    for (int c = 0; c < number_of_components; ++c)
    {
        for (double pu = 0; pu < 1 - radius; pu += radius) for (double pv = 0; pv < 1 - radius; pv += radius)
        {
            spin3f_t sp00, sp01, sp10, sp11;

            predgparam3f_eval(&sp00, param, pu, pv, c);
            predgparam3f_eval(&sp01, param, pu, pv + radius, c);
            predgparam3f_eval(&sp10, param, pu + radius, pv, c);
            predgparam3f_eval(&sp11, param, pu + radius, pv + radius, c);

            QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
            QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
            QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
            QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

            addTriangle(triangles, Triangle(v00, v01, v11));
            addTriangle(triangles, Triangle(v00, v11, v10));
        }

        // last u
        for (double pu = 0; pu < 1 - radius; pu += radius)
        {
            spin3f_t sp00, sp01, sp10, sp11;

            predgparam3f_eval(&sp00, param, pu, 1.0 - radius, c);
            predgparam3f_eval(&sp01, param, pu, 1.0, c);
            predgparam3f_eval(&sp10, param, pu + radius, 1.0 - radius, c);
            predgparam3f_eval(&sp11, param, pu + radius, 1.0, c);

            QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
            QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
            QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
            QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

            addTriangle(triangles, Triangle(v00, v01, v11));
            addTriangle(triangles, Triangle(v00, v11, v10));
        }

        // last v
        for (double pv = 0; pv < 1 - radius; pv += radius)
        {
            spin3f_t sp00, sp01, sp10, sp11;

            predgparam3f_eval(&sp00, param, 1.0 - radius, pv, c);
            predgparam3f_eval(&sp01, param, 1.0 - radius, pv + radius, c);
            predgparam3f_eval(&sp10, param, 1.0, pv, c);
            predgparam3f_eval(&sp11, param, 1.0, pv + radius, c);

            QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
            QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
            QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
            QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

            addTriangle(triangles, Triangle(v00, v01, v11));
            addTriangle(triangles, Triangle(v00, v11, v10));
        }

        // last uv
        {
            spin3f_t sp00, sp01, sp10, sp11;

            predgparam3f_eval(&sp00, param, 1.0 - radius, 1.0 - radius, c);
            predgparam3f_eval(&sp01, param, 1.0 - radius, 1.0, c);
            predgparam3f_eval(&sp10, param, 1.0, 1.0 - radius, c);
            predgparam3f_eval(&sp11, param, 1.0, 1.0, c);

            QVector3D v00(sp00.s12 / (1 - sp00.s0), sp00.s23 / (1 - sp00.s0), sp00.s31 / (1 - sp00.s0));
            QVector3D v01(sp01.s12 / (1 - sp01.s0), sp01.s23 / (1 - sp01.s0), sp01.s31 / (1 - sp01.s0));
            QVector3D v10(sp10.s12 / (1 - sp10.s0), sp10.s23 / (1 - sp10.s0), sp10.s31 / (1 - sp10.s0));
            QVector3D v11(sp11.s12 / (1 - sp11.s0), sp11.s23 / (1 - sp11.s0), sp11.s31 / (1 - sp11.s0));

            addTriangle(triangles, Triangle(v00, v01, v11));
            addTriangle(triangles, Triangle(v00, v11, v10));
        }
    }
}

void MainWindow::addTriangle(TriangleListPtr triangles, const Triangle &triangle)
{
    if (triangle.vertex(0).length() < CUTOFF_DISTANCE || triangle.vertex(1).length() < CUTOFF_DISTANCE || triangle.vertex(2).length() < CUTOFF_DISTANCE)
        triangles->push_back(triangle);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (!m_currentChanged)
        return;

    if (!handleUnsavedChanges())
    {
        event->ignore();
        return;
    }
}

void MainWindow::on_actionArcballCamera_triggered()
{
    m_rv->setRenderViewCamera(new RenderViewArcBallCamera());

    ui->actionArcballCamera->setChecked(true);
    ui->actionFreeCamera->setChecked(false);
    ui->actionAutoCamera->setChecked(false);
}

void MainWindow::on_actionFreeCamera_triggered()
{
    m_rv->setRenderViewCamera(new RenderViewFlyCamera());

    ui->actionArcballCamera->setChecked(false);
    ui->actionFreeCamera->setChecked(true);
    ui->actionAutoCamera->setChecked(false);
}

void MainWindow::on_actionAutoCamera_triggered()
{
    m_rv->setRenderViewCamera(new RenderViewAutoCamera());

    ui->actionArcballCamera->setChecked(false);
    ui->actionFreeCamera->setChecked(false);
    ui->actionAutoCamera->setChecked(true);
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

void MainWindow::on_verticalSliderMX_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderMX, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderMY_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderMY, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderMZ_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderMZ, value);
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

void MainWindow::on_verticalSliderCX_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderCX, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderCY_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderCY, value);
    updatePredicateInformation();
}

void MainWindow::on_verticalSliderCZ_valueChanged(int value)
{
    formatSliderValue(ui->labelSliderCZ, value);
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

void MainWindow::on_labelZeroM_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->verticalSliderMX->setValue(300);
    ui->verticalSliderMY->setValue(300);
    ui->verticalSliderMZ->setValue(300);
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

    ui->verticalSliderCX->setValue(300);
    ui->verticalSliderCY->setValue(300);
    ui->verticalSliderCZ->setValue(300);
}

void MainWindow::on_actionWireframe_triggered()
{
    m_rv->setWireframe(ui->actionWireframe->isChecked());
}

void MainWindow::on_actionQuit_triggered()
{
    if (!m_currentChanged)
    {
        emit close();
        return;
    }

    if (!handleUnsavedChanges())
        return;

    emit close();
}

void MainWindow::on_actionOpen_triggered()
{
    if (m_currentChanged && !handleUnsavedChanges())
        return;

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open file"), QString(), tr("*.tri"));

    if (fileName.isEmpty())
        return;

    QFile file(fileName);

    if (file.open(QFile::ReadOnly))
    {
        // reader
        {
            QTextStream stream(&file);

            int kx, ky, kz;
            stream >> kx >> ky >> kz;

            int lx, ly, lz;
            stream >> lx >> ly >> lz;

            int mx, my, mz;
            stream >> mx >> my >> mz;

            int ax, ay, az;
            stream >> ax >> ay >> az;

            int bx, by, bz;
            stream >> bx >> by >> bz;

            int cx, cy, cz;
            stream >> cx >> cy >> cz;

            if (stream.status() != QTextStream::Ok)
            {
                (void)QMessageBox::warning(this, tr("Open file"), tr("Failed to parse: ") + m_currentFile, QMessageBox::Ok);
                return;
            }

            ui->verticalSliderKX->setValue(kx);
            ui->verticalSliderKY->setValue(ky);
            ui->verticalSliderKZ->setValue(kz);

            ui->verticalSliderLX->setValue(lx);
            ui->verticalSliderLY->setValue(ly);
            ui->verticalSliderLZ->setValue(lz);

            ui->verticalSliderMX->setValue(mx);
            ui->verticalSliderMY->setValue(my);
            ui->verticalSliderMZ->setValue(mz);

            ui->verticalSliderAX->setValue(ax);
            ui->verticalSliderAY->setValue(ay);
            ui->verticalSliderAZ->setValue(az);

            ui->verticalSliderBX->setValue(bx);
            ui->verticalSliderBY->setValue(by);
            ui->verticalSliderBZ->setValue(bz);

            ui->verticalSliderCX->setValue(cx);
            ui->verticalSliderCY->setValue(cy);
            ui->verticalSliderCZ->setValue(cz);
        }

        file.close();
    }
    else
    {
        (void)QMessageBox::warning(this, tr("Open file"), tr("Failed to open: ") + m_currentFile, QMessageBox::Ok);
        return;
    }

    m_currentFile = fileName;
    m_currentChanged = false;
    updateWindowTitle();
}

void MainWindow::on_actionSave_triggered()
{
    if (m_currentChanged)
    {
        if (!m_currentFile.isEmpty())
            (void)saveCurrentFile();
        else
            (void)saveNewFile();
    }
}

void MainWindow::on_actionSaveAs_triggered()
{
    (void)saveNewFile();
}

void MainWindow::on_actionAutoMesh_triggered()
{
    updatePredicateInformation();
}

void MainWindow::on_actionTakeScreenshot_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save current frame buffer as"), QString(), QString("*.png"));

    if (fileName.isEmpty())
        return;

    if (!fileName.endsWith(".png"))
        fileName += ".png";

    QImage img = m_rv->grabFrameBuffer();

    if (!img.save(fileName, 0, 100))
    {
        (void)QMessageBox::warning(this, tr("Save current frame buffer file"), tr("Failed to save frame buffer"), QMessageBox::Ok);
        return;
    }
}

void MainWindow::on_actionModelOnlyView_triggered()
{
    m_rv->setModelOnlyView(ui->actionModelOnlyView->isChecked());
}
