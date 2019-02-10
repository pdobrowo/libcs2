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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "cs2/spin3f.h"
#include "cs2/vec3f.h"
#include "cs2/predg3f.h"
#include "meshing.h"
#include <QCloseEvent>
#include <QMainWindow>
#include <QSlider>
#include <QLabel>

namespace Ui {
class MainWindow;
}

class RenderView;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    RenderView *m_rv;

    bool m_blockPredicateUpdate;

    void updatePredicateInformation();

    void setPredicate(double kx, double ky, double kz,
                      double lx, double ly, double lz,
                      double ax, double ay, double az,
                      double bx, double by, double bz,
                      double c);

    void setPredicate(const struct cs2_predg3f_s *p);

    QString formatVector(const struct cs2_vec3f_s *v);
    QString formatVector(const struct cs2_vec4f_s *v);

    // file
    QString m_currentFile;
    bool m_currentChanged;

    bool handleUnsavedChanges();
    bool saveCurrentFile();
    bool saveNewFile();

    void updateWindowTitle();

    virtual void closeEvent(QCloseEvent *event);

private slots:
    void on_actionArcballCamera_triggered();
    void on_actionFreeCamera_triggered();
    void on_actionAutoCamera_triggered();
    void on_doubleSpinBoxKX_valueChanged(double);
    void on_doubleSpinBoxKY_valueChanged(double);
    void on_doubleSpinBoxKZ_valueChanged(double);
    void on_doubleSpinBoxLX_valueChanged(double);
    void on_doubleSpinBoxLY_valueChanged(double);
    void on_doubleSpinBoxLZ_valueChanged(double);
    void on_doubleSpinBoxAX_valueChanged(double);
    void on_doubleSpinBoxAY_valueChanged(double);
    void on_doubleSpinBoxAZ_valueChanged(double);
    void on_doubleSpinBoxBX_valueChanged(double);
    void on_doubleSpinBoxBY_valueChanged(double);
    void on_doubleSpinBoxBZ_valueChanged(double);
    void on_doubleSpinBoxC_valueChanged(double);
    void on_labelZeroK_linkActivated(const QString &link);
    void on_labelZeroL_linkActivated(const QString &link);
    void on_labelZeroA_linkActivated(const QString &link);
    void on_labelZeroB_linkActivated(const QString &link);
    void on_labelZeroC_linkActivated(const QString &link);
    void on_actionWireframe_triggered();
    void on_actionQuit_triggered();
    void on_actionOpen_triggered();
    void on_actionSave_triggered();
    void on_actionSaveAs_triggered();
    void on_actionAutoMesh_triggered();
    void on_actionTakeScreenshot_triggered();
    void on_actionModelOnlyView_triggered();
    void on_actionOutlines_triggered();
    void on_actionNormals_triggered();

    void on_actionA_pair_of_points_triggered();
    void on_actionA_pair_of_separate_ellipsoids_triggered();
    void on_actionA_pair_of_y_touching_ellipsoids_triggered();
    void on_actionA_pair_of_yz_crossed_ellipsoids_triggered();
    void on_actionA_pair_of_z_touching_ellipsoids_triggered();
    void on_actionA_y_barrel_triggered();
    void on_actionA_z_barrel_triggered();
    void on_actionA_notched_y_barrel_triggered();
    void on_actionA_notched_z_barrel_triggered();
    void on_actionA_pair_of_separate_yz_caps_triggered();
    void on_actionA_xy_zw_torus_triggered();
    void on_actionA_xy_circle_triggered();
    void on_actionA_zw_circle_triggered();
    void on_actionA_xz_yw_torus_triggered();
    void on_actionA_xz_circle_triggered();
    void on_actionA_yw_circle_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
