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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "cs2/predg3f.h"
#include "cs2/spin3f.h"
#include "cs2/vec3f.h"
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

    QString formatVector(const vec3f_t *v);
    QString formatVector(const vec4f_t *v);

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
    void on_verticalSliderAX_valueChanged(int value);
    void on_verticalSliderAY_valueChanged(int value);
    void on_verticalSliderAZ_valueChanged(int value);
    void on_verticalSliderBX_valueChanged(int value);
    void on_verticalSliderBY_valueChanged(int value);
    void on_verticalSliderBZ_valueChanged(int value);
    void on_verticalSliderC_valueChanged(int value);
    void on_labelZeroK_linkActivated(const QString &link);
    void on_labelZeroL_linkActivated(const QString &link);
    void on_labelZeroA_linkActivated(const QString &link);
    void on_labelZeroB_linkActivated(const QString &link);
    void on_labelZeroC_linkActivated(const QString &link);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
