#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>

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

    void updatePredicate();
    double sliderToValue(QSlider *slider);

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

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
