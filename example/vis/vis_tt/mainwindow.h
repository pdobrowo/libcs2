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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "cs2/predg3f.h"
#include "cs2/preds3f.h"
#include "cs2/predtt3f.h"
#include "cs2/spin3f.h"
#include "cs2/vec3f.h"
#include "trianglelistmesh.h"
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
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    RenderView *m_rv;

    void updatePredicateInformation();

    double sliderToParamValue(QSlider *slider);
    double sliderValueToParamValue(double value);

    void formatSliderValue(QLabel *label, double value);
    void updateSliderInformation();

    QString formatVector(const struct cs2_vec3f_s *v);
    QString formatVector(const struct cs2_vec4f_s *v);

    // file
    QString m_currentFile;
    bool m_currentChanged;

    bool handleUnsavedChanges();
    bool saveCurrentFile();
    bool saveNewFile();

    void updateWindowTitle();

    void autoMesh(TriangleListPtr triangles, struct cs2_predgparam3f_s *param, double initialRadius, double targetRadius, int maxSubdivisions);
    void autoMeshInternal(TriangleListPtr triangles, struct cs2_predgparam3f_s *param, double targetRadius, int component, double minU, double maxU, double minV, double maxV, int maxSubdivisions, int subdivision);

    void simpleMesh(TriangleListPtr triangles, struct cs2_predgparam3f_s *param, double radius);

    void addTriangle(TriangleListPtr triangles, const Triangle &triangle);

    TriangleListPtr backface(const TriangleListPtr triangles) const;

    virtual void closeEvent(QCloseEvent *event);

private slots:
    void on_actionArcballCamera_triggered();
    void on_actionFreeCamera_triggered();
    void on_actionAutoCamera_triggered();
    void on_verticalSliderKX_valueChanged(int value);
    void on_verticalSliderKY_valueChanged(int value);
    void on_verticalSliderKZ_valueChanged(int value);
    void on_verticalSliderLX_valueChanged(int value);
    void on_verticalSliderLY_valueChanged(int value);
    void on_verticalSliderLZ_valueChanged(int value);
    void on_verticalSliderMX_valueChanged(int value);
    void on_verticalSliderMY_valueChanged(int value);
    void on_verticalSliderMZ_valueChanged(int value);
    void on_verticalSliderAX_valueChanged(int value);
    void on_verticalSliderAY_valueChanged(int value);
    void on_verticalSliderAZ_valueChanged(int value);
    void on_verticalSliderBX_valueChanged(int value);
    void on_verticalSliderBY_valueChanged(int value);
    void on_verticalSliderBZ_valueChanged(int value);
    void on_verticalSliderCX_valueChanged(int value);
    void on_verticalSliderCY_valueChanged(int value);
    void on_verticalSliderCZ_valueChanged(int value);
    void on_labelZeroK_linkActivated(const QString &link);
    void on_labelZeroL_linkActivated(const QString &link);
    void on_labelZeroM_linkActivated(const QString &link);
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

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
