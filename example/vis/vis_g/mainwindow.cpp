/**
 * Copyright (c) 2015-2019 Przemysław Dobrowolski
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
#include "test/testpredg3f.h"
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
    m_blockPredicateUpdate(false),
    m_currentChanged(false),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // setup
    if (true)
    {
        QGLFormat fmt;
        fmt.setSamples(4);
        fmt.setSampleBuffers(true);
        QGLFormat::setDefaultFormat(fmt);
    }

    m_rv = new RenderView();

    m_rv->setCaption("cspace");
    m_rv->enableViewModeOption(ViewMode::Culling);

    ui->widgetView->layout()->addWidget(m_rv);

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
    if (m_blockPredicateUpdate)
        return;

    // changed
    m_currentChanged = true;
    updateWindowTitle();

    // predicate g
    struct cs2_predg3f_s pred =
        {
            { ui->doubleSpinBoxKX->value(), ui->doubleSpinBoxKY->value(), ui->doubleSpinBoxKZ->value() },
            { ui->doubleSpinBoxLX->value(), ui->doubleSpinBoxLY->value(), ui->doubleSpinBoxLZ->value() },
            { ui->doubleSpinBoxAX->value(), ui->doubleSpinBoxAY->value(), ui->doubleSpinBoxAZ->value() },
            { ui->doubleSpinBoxBX->value(), ui->doubleSpinBoxBY->value(), ui->doubleSpinBoxBZ->value() },
            ui->doubleSpinBoxC->value()
        };

    // param g
    struct cs2_predgparam3f_s param;
    cs2_predg3f_param(&param, &pred);

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
    struct cs2_vec3f_s p, q, u, v;
    cs2_predg3f_pquv(&p, &q, &u, &v, &pred);

    ui->labelPval->setText(formatVector(&p));
    ui->labelQval->setText(formatVector(&q));
    ui->labelUval->setText(formatVector(&u));
    ui->labelVval->setText(formatVector(&v));
    ui->labelC2val->setText(QString::number(c, 'f', 2));

    // information -> type
    enum cs2_predgtype3f_e type = cs2_predg3f_type(&pred);

    ui->labelproperval->setText(type == cs2_predgtype3f_improper ? "no" : "yes");
    ui->labeltypeval->setText(cs2_predgtype3f_str(type));

    // information -> parametrization variables
    ui->labelaval->setText(QString::number(a, 'f', 4));
    ui->labelbval->setText(QString::number(b, 'f', 4));
    ui->labelcval->setText(QString::number(c, 'f', 4));
    ui->labelminval->setText(QString::number(mi, 'f', 4));
    ui->labelmaxval->setText(QString::number(ma, 'f', 4));

    ui->labelcaseval->setText(cs2_predgparamtype3f_str(param.t));
    ui->labeldimval->setText(QString::number(cs2_predgparamtype3f_dim(param.t)));
    ui->labelcompval->setText(QString::number(cs2_predgparamtype3f_domain_components(param.t)));
    ui->labelismanival->setText(cs2_predgparamtype3f_is_manifold(param.t) ? "yes" : "no");
    ui->labelhasholeval->setText(cs2_predgparamtype3f_has_domain_hole(param.t) ? "yes" : "no");
    ui->labelisconnval->setText(cs2_predgparamtype3f_is_connected(param.t) ? "yes" : "no");

    // information -> eigen decomposition
    ui->labeldetval->setText(QString::number(param.e[0] * param.e[1] * param.e[2] * param.e[3], 'f', 8));

    ui->labell1val->setText(QString::number(param.e[0], 'f', 4));
    ui->labell2val->setText(QString::number(param.e[1], 'f', 4));
    ui->labell3val->setText(QString::number(param.e[2], 'f', 4));
    ui->labell4val->setText(QString::number(param.e[3], 'f', 4));

    ui->labelw1val->setText(formatVector(&param.ev[0]));
    ui->labelw2val->setText(formatVector(&param.ev[1]));
    ui->labelw3val->setText(formatVector(&param.ev[2]));
    ui->labelw4val->setText(formatVector(&param.ev[3]));

    // visual
    m_rv->removeAllObjects();

    // only 2-dimensional
    if (cs2_predgparamtype3f_dim(param.t) == 2)
    {
        TriangleListPtr trianglesFront(new TriangleList());
        TriangleListPtr trianglesBack(new TriangleList());

        if (ui->actionAutoMesh->isChecked())
            meshing::autoMesh(trianglesFront, trianglesBack, &param, 0.1, 0.5, 10);
        else
            meshing::simpleMesh(trianglesFront, trianglesBack, &param, 0.01);

        const QColor salemColor(9, 127, 75);
        const QColor redBerryColor(142, 0, 0);

        m_rv->addTriangleList(trianglesFront, salemColor);
        m_rv->addTriangleList(trianglesBack, redBerryColor);
    }
}

void MainWindow::setPredicate(double kx, double ky, double kz, double lx, double ly, double lz, double ax, double ay, double az, double bx, double by, double bz, double c)
{
    m_blockPredicateUpdate = true;

    ui->doubleSpinBoxKX->setValue(kx);
    ui->doubleSpinBoxKY->setValue(ky);
    ui->doubleSpinBoxKZ->setValue(kz);

    ui->doubleSpinBoxLX->setValue(lx);
    ui->doubleSpinBoxLY->setValue(ly);
    ui->doubleSpinBoxLZ->setValue(lz);

    ui->doubleSpinBoxAX->setValue(ax);
    ui->doubleSpinBoxAY->setValue(ay);
    ui->doubleSpinBoxAZ->setValue(az);

    ui->doubleSpinBoxBX->setValue(bx);
    ui->doubleSpinBoxBY->setValue(by);
    ui->doubleSpinBoxBZ->setValue(bz);

    ui->doubleSpinBoxC->setValue(c);

    m_blockPredicateUpdate = false;

    updatePredicateInformation();
}

void MainWindow::setPredicate(const cs2_predg3f_s *p)
{
    setPredicate(p->k.x, p->k.y, p->k.z,
                 p->l.x, p->l.y, p->l.z,
                 p->a.x, p->a.y, p->a.z,
                 p->b.x, p->b.y, p->b.z,
                 p->c);
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

            stream << ui->doubleSpinBoxKX->value() << " ";
            stream << ui->doubleSpinBoxKY->value() << " ";
            stream << ui->doubleSpinBoxKZ->value();
            stream << "\r\n";
            stream << ui->doubleSpinBoxLX->value() << " ";
            stream << ui->doubleSpinBoxLY->value() << " ";
            stream << ui->doubleSpinBoxLZ->value();
            stream << "\r\n";
            stream << ui->doubleSpinBoxAX->value() << " ";
            stream << ui->doubleSpinBoxAY->value() << " ";
            stream << ui->doubleSpinBoxAZ->value();
            stream << "\r\n";
            stream << ui->doubleSpinBoxBX->value() << " ";
            stream << ui->doubleSpinBoxBY->value() << " ";
            stream << ui->doubleSpinBoxBZ->value();
            stream << "\r\n";
            stream << ui->doubleSpinBoxC->value();
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
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save as"), QString(), QString("*.vis"));

    if (fileName.isEmpty())
        return false;

    if (!fileName.endsWith(".vis"))
        fileName += ".vis";

    m_currentFile = fileName;
    return saveCurrentFile();
}

void MainWindow::updateWindowTitle()
{
    QString fileName;

    if (m_currentFile.isEmpty())
        fileName = "unnamed.vis";
    else
    {
        QFileInfo fileInfo(m_currentFile);
        fileName = fileInfo.fileName();
    }

    setWindowTitle(fileName + ": vis" + (m_currentChanged ? " *" : ""));
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

void MainWindow::on_doubleSpinBoxKX_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxKY_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxKZ_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxLX_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxLY_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxLZ_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxAX_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxAY_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxAZ_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxBX_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxBY_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxBZ_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_doubleSpinBoxC_valueChanged(double)
{
    updatePredicateInformation();
}

void MainWindow::on_labelZeroK_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->doubleSpinBoxKX->setValue(0);
    ui->doubleSpinBoxKY->setValue(0);
    ui->doubleSpinBoxKZ->setValue(0);
}

void MainWindow::on_labelZeroL_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->doubleSpinBoxLX->setValue(0);
    ui->doubleSpinBoxLY->setValue(0);
    ui->doubleSpinBoxLZ->setValue(0);
}

void MainWindow::on_labelZeroA_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->doubleSpinBoxAX->setValue(0);
    ui->doubleSpinBoxAY->setValue(0);
    ui->doubleSpinBoxAZ->setValue(0);
}

void MainWindow::on_labelZeroB_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->doubleSpinBoxBX->setValue(0);
    ui->doubleSpinBoxBY->setValue(0);
    ui->doubleSpinBoxBZ->setValue(0);
}

void MainWindow::on_labelZeroC_linkActivated(const QString &link)
{
    Q_UNUSED(link);

    ui->doubleSpinBoxC->setValue(0);
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

    QString fileName = QFileDialog::getOpenFileName(this, tr("Open file"), QString(), tr("*.vis"));

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

            int ax, ay, az;
            stream >> ax >> ay >> az;

            int bx, by, bz;
            stream >> bx >> by >> bz;

            int c;
            stream >> c;

            if (stream.status() != QTextStream::Ok)
            {
                (void)QMessageBox::warning(this, tr("Open file"), tr("Failed to parse: ") + m_currentFile, QMessageBox::Ok);
                return;
            }

            ui->doubleSpinBoxKX->setValue(kx);
            ui->doubleSpinBoxKY->setValue(ky);
            ui->doubleSpinBoxKZ->setValue(kz);

            ui->doubleSpinBoxLX->setValue(lx);
            ui->doubleSpinBoxLY->setValue(ly);
            ui->doubleSpinBoxLZ->setValue(lz);

            ui->doubleSpinBoxAX->setValue(ax);
            ui->doubleSpinBoxAY->setValue(ay);
            ui->doubleSpinBoxAZ->setValue(az);

            ui->doubleSpinBoxBX->setValue(bx);
            ui->doubleSpinBoxBY->setValue(by);
            ui->doubleSpinBoxBZ->setValue(bz);

            ui->doubleSpinBoxC->setValue(c);
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

    if (!img.save(fileName, nullptr, 100))
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

void MainWindow::on_actionA_pair_of_points_triggered()
{
    setPredicate(&test_predg3f_a_pair_of_points);
}

void MainWindow::on_actionA_pair_of_separate_ellipsoids_triggered()
{
    setPredicate(&test_predg3f_a_pair_of_separate_ellipsoids);
}

void MainWindow::on_actionA_pair_of_y_touching_ellipsoids_triggered()
{
    setPredicate(&test_predg3f_a_pair_of_y_touching_ellipsoids);
}

void MainWindow::on_actionA_pair_of_yz_crossed_ellipsoids_triggered()
{
    setPredicate(&test_predg3f_a_pair_of_yz_crossed_ellipsoids);
}

void MainWindow::on_actionA_pair_of_z_touching_ellipsoids_triggered()
{
    setPredicate(&test_predg3f_a_pair_of_z_touching_ellipsoids);
}

void MainWindow::on_actionA_y_barrel_triggered()
{
    setPredicate(&test_predg3f_a_y_barrel);
}

void MainWindow::on_actionA_z_barrel_triggered()
{
    setPredicate(&test_predg3f_a_z_barrel);
}

void MainWindow::on_actionA_notched_y_barrel_triggered()
{
    setPredicate(&test_predg3f_a_notched_y_barrel);
}

void MainWindow::on_actionA_notched_z_barrel_triggered()
{
    setPredicate(&test_predg3f_a_notched_z_barrel);
}

void MainWindow::on_actionA_pair_of_separate_yz_caps_triggered()
{
    setPredicate(&test_predg3f_a_pair_of_separate_yz_caps);
}

void MainWindow::on_actionA_xy_zw_torus_triggered()
{
    setPredicate(&test_predg3f_a_xy_zw_torus);
}

void MainWindow::on_actionA_xy_circle_triggered()
{
    setPredicate(&test_predg3f_a_xy_circle);
}

void MainWindow::on_actionA_zw_circle_triggered()
{
    setPredicate(&test_predg3f_a_zw_circle);
}

void MainWindow::on_actionA_xz_yw_torus_triggered()
{
    setPredicate(&test_predg3f_a_xz_yw_torus);
}

void MainWindow::on_actionA_xz_circle_triggered()
{
    setPredicate(&test_predg3f_a_xz_circle);
}

void MainWindow::on_actionA_yw_circle_triggered()
{
    setPredicate(&test_predg3f_a_yw_circle);
}
