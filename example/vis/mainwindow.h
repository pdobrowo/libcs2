#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

private slots:
    void on_actionArcballCamera_triggered();

    void on_actionFreeCamera_triggered();

    void on_actionAutoCamera_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
