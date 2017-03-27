/**
 * Copyright (c) 2015-2017 Przemysław Dobrowolski
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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
#include <boost/scoped_array.hpp>
#include <cassert>
#include <cmath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_currentChanged(false),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    m_rv = new RenderView();

    m_rv->setCaption("cspace");
    m_rv->enableViewModeOption(ViewMode::Culling);

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
    struct cs2_predtt3f_s tt =
        {
            { sliderToParamValue(ui->verticalSliderKX), sliderToParamValue(ui->verticalSliderKY), sliderToParamValue(ui->verticalSliderKZ) },
            { sliderToParamValue(ui->verticalSliderLX), sliderToParamValue(ui->verticalSliderLY), sliderToParamValue(ui->verticalSliderLZ) },
            { sliderToParamValue(ui->verticalSliderMX), sliderToParamValue(ui->verticalSliderMY), sliderToParamValue(ui->verticalSliderMZ) },
            { sliderToParamValue(ui->verticalSliderAX), sliderToParamValue(ui->verticalSliderAY), sliderToParamValue(ui->verticalSliderAZ) },
            { sliderToParamValue(ui->verticalSliderBX), sliderToParamValue(ui->verticalSliderBY), sliderToParamValue(ui->verticalSliderBZ) },
            { sliderToParamValue(ui->verticalSliderCX), sliderToParamValue(ui->verticalSliderCY), sliderToParamValue(ui->verticalSliderCZ) }
        };

    // decompose tt->s
    struct cs2_predttdecomp3f_s ttd;
    cs2_predtt3f_decomp(&ttd, &tt);

    // visual
    m_rv->removeAllObjects();

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            struct cs2_preds3f_s *s = &ttd.s[i][j];
            struct cs2_predg3f_s g;

            cs2_predg3f_from_preds3f(&g, s);

            // param g
            struct cs2_predgparam3f_s param;
            cs2_predg3f_param(&param, &g);

            // only 2-dimensional
            if (cs2_predgparamtype3f_dim(param.t) == 2)
            {
                TriangleListPtr trianglesFront(new TriangleList());
                TriangleListPtr trianglesBack(new TriangleList());

                if (ui->actionAutoMesh->isChecked())
                    meshing::autoMesh(trianglesFront, trianglesBack, &param, 0.1, 0.5, 10);
                else
                    meshing::simpleMesh(trianglesFront, trianglesBack, &param, 0.01);

                filterTriangles(trianglesFront);
                filterTriangles(trianglesBack);

                const QColor nepalColor(142, 171, 193);

                m_rv->addTriangleList(trianglesFront, nepalColor);
                m_rv->addTriangleList(trianglesBack, nepalColor);
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

QString MainWindow::formatVector(const struct cs2_vec3f_s *v)
{
    return "[" + QString::number(v->x, 'f', 2) + ", " + QString::number(v->y, 'f', 2) + ", " + QString::number(v->z, 'f', 2) + "]";
}

QString MainWindow::formatVector(const struct cs2_vec4f_s *v)
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

void MainWindow::filterTriangles(TriangleListPtr triangles)
{
    TriangleListPtr filteredTriangles(new TriangleList());

    double cutoffDistance = ui->doubleSpinBoxCutoffDistance->value();

    for (TriangleList::const_iterator it = triangles->begin(); it != triangles->end(); ++it)
    {
        const Triangle &triangle = *it;

        if (triangle.vertex(0).length() < cutoffDistance || triangle.vertex(1).length() < cutoffDistance || triangle.vertex(2).length() < cutoffDistance)
            filteredTriangles->push_back(triangle);
    }

    triangles->swap(*filteredTriangles);
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
    m_rv->setViewModeOption(ViewMode::Wireframe, ui->actionWireframe->isChecked());
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
    m_rv->setViewModeOption(ViewMode::ModelOnly, ui->actionModelOnlyView->isChecked());
}

void MainWindow::on_actionOutlines_triggered()
{
    m_rv->setViewModeOption(ViewMode::Outlines, ui->actionOutlines->isChecked());
}

void MainWindow::on_actionNormals_triggered()
{
    m_rv->setViewModeOption(ViewMode::Normals, ui->actionNormals->isChecked());
}

void MainWindow::on_doubleSpinBoxCutoffDistance_valueChanged(double value)
{
    Q_UNUSED(value);
    updatePredicateInformation();
}
