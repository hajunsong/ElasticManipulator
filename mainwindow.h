#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void quitBtnSlot();
    void runBtnSlot();
    void readyBtnSlot();
    void on_motionACb_stateChanged(int arg1);
    void on_motionBCb_stateChanged(int arg1);
    void on_collisionCb_stateChanged(int arg1);
    void on_setting1Btn_clicked();
    void on_setting2Btn_clicked();
    void on_setting3Btn_clicked();
    void on_setting4Btn_clicked();
    void run_timer_out();

private slots:
    void on_cutoffSpinBox_editingFinished();
    void on_threshSpinBox_editingFinished();

    void on_emergen_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *run_timer;
};

#endif // MAINWINDOW_H
