#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
#include <DxlControl.h>
#include <pthread.h>

#include <iostream>
#include <fstream>
#include <cstdio>
#include "dob.h"

using namespace std;

void high_pass_filter(QVector<double> *cur, QVector<double> *timeZone, QVector<double> *cur_filter, int index, double ts);

#define D2R 0.0174532925199433
#define R2D 57.2957795130823
#define D2P 728.175
#define P2D 0.00137329625433447

RT_TASK dxl_task;
RTIME interval = 5e6;
DxlControl dxlControl;
bool run = false, ready = false, quit = false;

QVector<double> sim_time;
QVector<double> J1_cur, J2_cur, J3_cur, J4_cur;
QVector<double> J1_pos, J2_pos, J3_pos, J4_pos;
QVector<double> J1_vel, J2_vel, J3_vel, J4_vel;
QVector<double> J1_filter, J2_filter, J3_filter, J4_filter;
QVector<double> J1_rhat, J2_rhat, J3_rhat, J4_rhat;
QVector<double> J1_enc1, J2_enc1, J3_enc1, J4_enc1;
QVector<double> J1_enc2, J2_enc2, J3_enc2, J4_enc2;
QVector<double> J1_tau, J2_tau, J3_tau, J4_tau;
double indx;

int J1offset, J2offset, J3offset, J4offset;
double J1threshP, J2threshP, J3threshP, J4threshP;
double J1threshN, J2threshN, J3threshN, J4threshN;
int J1CollPos, J2CollPos, J3CollPos, J4CollPos;
int CollIndex;
double threshOffset;

bool flag, collision;
bool motionA, motionB;
bool start_flag, stop_flag;

double f_cut, ts, w_cut, tau, tau_ts, a1, b0, b1, a2, b2, sum0;
QVector<double> timeZone1, timeZone2, timeZone3, timeZone4;
int filter_index;
bool collStartFlag;

double J1des, J2des, J3des, J4des;

DOB dob(4);
double q[4], q_dot[4], torque[4], rhat[4];
double K = 11000;

/* NOTE: error handling omitted. */
void MotorCtrlMain(void *arg)
{
    cout << "Zenomail Thread" << endl;
    RTIME now, previous;

    rt_task_set_periodic(NULL, TM_NOW, interval);
    previous = rt_timer_read();
    while(1){
        rt_task_wait_period(NULL);
        now = rt_timer_read();

        ///////////////////////////////
        if (run){
            if (collision == false){
                if (motionA && !motionB){
                    if (flag == true){
                        J1des = 60*D2P;
                        J2des = 30*D2P;
                        J3des = 40*D2P;
                        J4des = 60*D2P;
                        dxlControl.setPosition(J1offset + static_cast<int32_t>(J1des), 1);
                        dxlControl.setPosition(J4offset + static_cast<int32_t>(J4des), 4);
                    }
                    else {
                        J1des = -60*D2P;
                        J2des = 30*D2P;
                        J3des = 40*D2P;
                        J4des = -60*D2P;
                        dxlControl.setPosition(J1offset + static_cast<int32_t>(J1des), 1);
                        dxlControl.setPosition(J4offset + static_cast<int32_t>(J4des), 4);
                    }
                }
                if (!motionA && motionB){
                    if (flag == true){
                        J1des = 0*D2P;
                        J2des = 0*D2P;
                        J3des = 30*D2P;
                        J4des = 0*D2P;
                        dxlControl.setPosition(J1offset + static_cast<int32_t>(J1des), 1);
                        dxlControl.setPosition(J2offset + static_cast<int32_t>(J2des), 2);
                        dxlControl.setPosition(J3offset + static_cast<int32_t>(J3des), 3);
                        dxlControl.setPosition(J4offset + static_cast<int32_t>(J4des), 4);
                    }
                    else {
                        J1des = 0*D2P;
                        J2des = 30*D2P;
                        J3des = 30*D2P;
                        J4des = 0*D2P;
                        dxlControl.setPosition(J1offset + static_cast<int32_t>(J1des), 1);
                        dxlControl.setPosition(J2offset + static_cast<int32_t>(J2des), 2);
                        dxlControl.setPosition(J3offset + static_cast<int32_t>(J3des), 3);
                        dxlControl.setPosition(J4offset + static_cast<int32_t>(J4des), 4);
                    }
                }
            }
            else{
                dxlControl.setPosition(J1CollPos, 1);
                dxlControl.setPosition(J2CollPos, 2);
                dxlControl.setPosition(J3CollPos, 3);
                dxlControl.setPosition(J4CollPos, 4);
            }

            sim_time.push_back(indx);
            J1_cur.push_back(dxlControl.getPresentCurrent(1));
            J2_cur.push_back(dxlControl.getPresentCurrent(2));
            J3_cur.push_back(dxlControl.getPresentCurrent(3));
            J4_cur.push_back(dxlControl.getPresentCurrent(4));

            J1_pos.push_back(dxlControl.getPresentPosition(1));
            J2_pos.push_back(dxlControl.getPresentPosition(2));
            J3_pos.push_back(dxlControl.getPresentPosition(3));
            J4_pos.push_back(dxlControl.getPresentPosition(4));

            J1_vel.push_back(dxlControl.getPresentVelocity(1));
            J2_vel.push_back(dxlControl.getPresentVelocity(2));
            J3_vel.push_back(dxlControl.getPresentVelocity(3));
            J4_vel.push_back(dxlControl.getPresentVelocity(4));

            J1_enc1.push_back(dxlControl.getPresentEncoder1(1));
            J2_enc1.push_back(dxlControl.getPresentEncoder1(2));
            J3_enc1.push_back(dxlControl.getPresentEncoder1(3));
            J4_enc1.push_back(dxlControl.getPresentEncoder1(4));

            J1_enc2.push_back(dxlControl.getPresentEncoder2(1));
            J2_enc2.push_back(dxlControl.getPresentEncoder2(2));
            J3_enc2.push_back(dxlControl.getPresentEncoder2(3));
            J4_enc2.push_back(dxlControl.getPresentEncoder2(4));

//            cout << J1_cur.back() << ", "<< J2_cur.back() << ", "<< J3_cur.back() << ", "<< J4_cur.back() << endl;

            ///////////////////////////////
            // DOB
            q[0] = J1_pos.back()*P2D*D2R;
            q[1] = J2_pos.back()*P2D*D2R;
            q[2] = J3_pos.back()*P2D*D2R;
            q[3] = J4_pos.back()*P2D*D2R;
            q_dot[0] = J1_vel.back()*0.01*6*D2R;
            q_dot[1] = J2_vel.back()*0.01*6*D2R;
            q_dot[2] = J3_vel.back()*0.01*6*D2R;
            q_dot[3] = J4_vel.back()*0.01*6*D2R;
            torque[0] = K*((J1_enc1.back() - J1_enc2.back())*P2D*D2R);
            torque[1] = K*((J2_enc1.back() - J2_enc2.back())*P2D*D2R);
            torque[2] = K*((J3_enc1.back() - J3_enc2.back())*P2D*D2R);
            torque[3] = K*((J4_enc1.back() - J4_enc2.back())*P2D*D2R);

            J1_tau.push_back(torque[0]);
            J2_tau.push_back(torque[1]);
            J3_tau.push_back(torque[2]);
            J4_tau.push_back(torque[3]);

            dob.run(q, q_dot, torque, rhat);

            J1_rhat.push_back(rhat[0]);
            J2_rhat.push_back(rhat[1]);
            J3_rhat.push_back(rhat[2]);
            J4_rhat.push_back(rhat[3]);
            ///////////////////////////////

            double h_save = (now - previous)/1000000000.0;
            high_pass_filter(&J1_rhat, &timeZone1, &J1_filter, 1, h_save);
            high_pass_filter(&J2_rhat, &timeZone2, &J2_filter, 2, h_save);
            high_pass_filter(&J3_rhat, &timeZone3, &J3_filter, 3, h_save);
            high_pass_filter(&J4_rhat, &timeZone4, &J4_filter, 4, h_save);
            filter_index++;
            indx += h_save;
            previous = now;

//            cout << J1_cur_filter.back() << ", "<< J2_cur_filter.back() << ", "<< J3_cur_filter.back() << ", "<< J4_cur_filter.back() << endl;

            if (!collStartFlag && sim_time.length() > 2){
                if (J1_filter.back() > J1threshP) J1threshP = J1_filter.back();
                if (J1_filter.back() < J1threshN) J1threshN = J1_filter.back();
                if (J2_filter.back() > J2threshP) J2threshP = J2_filter.back();
                if (J2_filter.back() < J2threshN) J2threshN = J2_filter.back();
                if (J3_filter.back() > J3threshP) J3threshP = J3_filter.back();
                if (J3_filter.back() < J3threshN) J3threshN = J3_filter.back();
                if (J4_filter.back() > J4threshP) J4threshP = J4_filter.back();
                if (J4_filter.back() < J4threshN) J4threshN = J4_filter.back();
            }

            if (sim_time.length() > 2){
                if (collision == false && collStartFlag){
                    if (J1threshP < J1_filter.back() || J1threshN > J1_filter.back()){
                        collision = true;
                        CollIndex = 1;
                    }
                    if (J2threshP < J2_filter.back() || J2threshN > J2_filter.back()){
                        collision = true;
                        CollIndex = 2;
                    }
                    if (J3threshP < J3_filter.back() || J3threshN > J3_filter.back()){
                        collision = true;
                        CollIndex = 3;
                    }
//                    if (J4threshP < J4_filter.back() || J4threshN > J4_filter.back()){
//                        collision = true;
//                        CollIndex = 4;
//                    }
                    if (collision || stop_flag == true){
                        dxlControl.reset();
                        J1CollPos = dxlControl.getPresentPosition(1);
                        J2CollPos = dxlControl.getPresentPosition(2);
                        J3CollPos = dxlControl.getPresentPosition(3);
                        J4CollPos = dxlControl.getPresentPosition(4);

                        cout << "Collision Detect.... " << CollIndex << endl;
                        collision = true;
                    }
                }
            }
        }
    }
}

void catch_signal(int sig){}

void high_pass_filter(QVector<double> *cur, QVector<double> *timeZone, QVector<double> *cur_filter, int index, double ts){
//    switch(index){
    //    case 1:
    //        f_cut = 20000;
    //        break;
    //    case 2:
    //        f_cut = 20000;
    //        break;
    //    case 3:
    //        f_cut = 20000;
    //        break;
    //    case 4:
    //        f_cut = 20000;
    //        break;
    //    }
    w_cut = 2*M_PI*f_cut;
    tau = 1 / w_cut;
    tau_ts = 1/(tau + ts);
    a1 = -tau*tau_ts;
    b0 = tau*tau_ts;
    b1 = -tau*tau_ts;
    a2 = 0;
    b2 = 0;
    sum0 = 0;

    if (cur->length() >= 3){
        sum0 = -a1*timeZone->at(filter_index - 1) - a2*timeZone->at(filter_index - 2);
        timeZone->push_back(cur->back() + sum0);
        cur_filter->push_back(b0*timeZone->back() + b1*timeZone->at(filter_index - 1) + b2*timeZone->at(filter_index - 2));
    }
    else{
        timeZone->push_back(cur->at(filter_index));
        cur_filter->push_back(cur->at(filter_index));
    }
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    dxlControl.init();
    dxlControl.dxl_init(1);
    dxlControl.dxl_init(2);
    dxlControl.dxl_init(3);
    dxlControl.dxl_init(4);

    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);
    /* Avoids memory swapping for this program */
    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_task_create(&dxl_task, "CollisionDetection", 0, 99, 0);
    rt_task_start(&dxl_task, &MotorCtrlMain, NULL);
//    rt_task_join(&dxl_task);

    connect(ui->runBtn, SIGNAL(clicked()), this, SLOT(runBtnSlot()));
    connect(ui->readyBtn, SIGNAL(clicked()), this, SLOT(readyBtnSlot()));
    connect(ui->quitBtn, SIGNAL(clicked()), this, SLOT(quitBtnSlot()));

    ui->pos1SpinBox->setMaximum(100000000);
    ui->pos2SpinBox->setMaximum(100000000);
    ui->pos3SpinBox->setMaximum(100000000);
    ui->pos4SpinBox->setMaximum(100000000);
    ui->pos1SpinBox->setMinimum(-100000000);
    ui->pos2SpinBox->setMinimum(-100000000);
    ui->pos3SpinBox->setMinimum(-100000000);
    ui->pos4SpinBox->setMinimum(-100000000);

    ui->pos1SpinBox->setSingleStep(100);
    ui->pos2SpinBox->setSingleStep(100);
    ui->pos3SpinBox->setSingleStep(100);
    ui->pos4SpinBox->setSingleStep(100);

    ui->pos1SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_1));
    ui->pos2SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_2));
    ui->pos3SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_3));
    ui->pos4SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_4));

    f_cut = 100;
    threshOffset = 1;
    ui->cutoffSpinBox->setSingleStep(100);
    ui->cutoffSpinBox->setRange(0,20000);
    ui->cutoffSpinBox->setValue(f_cut);
    ui->threshSpinBox->setSingleStep(0.01);
    ui->threshSpinBox->setRange(0, 100);
    ui->threshSpinBox->setValue(threshOffset);


    J1_cur.clear();
    J2_cur.clear();
    J3_cur.clear();
    J4_cur.clear();
    sim_time.clear();

    indx = 0;

    J1offset = 0;
    J2offset = -22500;
    J3offset = -86500;
    J4offset = 53950;

    flag = true;
    collision = false;
    stop_flag = false;

    J1threshP = -10000;
    J2threshP = -10000;
    J3threshP = -10000;
    J4threshP = -10000;

    J1threshN = 10000;
    J2threshN = 10000;
    J3threshN = 10000;
    J4threshN = 10000;

    motionA = true;
    motionB = false;

    ui->motionACb->setChecked(motionA);
    ui->motionBCb->setChecked(motionB);

    start_flag = false;
    filter_index = 0;
    collStartFlag = false;

    run_timer = new QTimer(this);
    run_timer->setInterval(3000);
    connect(run_timer, SIGNAL(timeout()), this, SLOT(run_timer_out()));
}

MainWindow::~MainWindow()
{
    delete ui;
    J1_cur.clear(); J2_cur.clear(); J3_cur.clear(); J4_cur.clear();
    J1_filter.clear(); J2_filter.clear(); J3_filter.clear(); J4_filter.clear();
    J1_pos.clear(); J2_pos.clear(); J3_pos.clear(); J4_pos.clear();
    J1_vel.clear(); J2_vel.clear(); J3_vel.clear(); J4_vel.clear();
    J1_rhat.clear(); J2_rhat.clear(); J3_rhat.clear(); J4_rhat.clear();
    J1_enc1.clear(); J2_enc1.clear(); J3_enc1.clear(); J4_enc1.clear();
    J1_enc2.clear(); J2_enc2.clear(); J3_enc2.clear(); J4_enc2.clear();
    sim_time.clear();
    dxlControl.dxl_deinit();
}

void MainWindow::quitBtnSlot()
{
    rt_task_delete(&dxl_task);
    dxlControl.reset();
//    FILE *fp1, *fp2, *fp3, *fp4;
//    fp1 = fopen("../SaveData/Axis1Data.csv", "w+");
//    fp2 = fopen("../SaveData/Axis2Data.csv", "w+");
//    fp3 = fopen("../SaveData/Axis3Data.csv", "w+");
//    fp4 = fopen("../SaveData/Axis4Data.csv", "w+");
//    for(int i = 2; i < sim_time.length(); i++){
//        fprintf(fp1, "%5.5f,%5.5f,%5.5f,%5.5f,%5.5f\n", sim_time[i], J1_pos[i], J1_vel[i], J1_cur[i], J1_cur_filter[i]);
//        fprintf(fp2, "%5.5f,%5.5f,%5.5f,%5.5f,%5.5f\n", sim_time[i], J2_pos[i], J2_vel[i], J2_cur[i], J2_cur_filter[i]);
//        fprintf(fp3, "%5.5f,%5.5f,%5.5f,%5.5f,%5.5f\n", sim_time[i], J3_pos[i], J3_vel[i], J3_cur[i], J3_cur_filter[i]);
//        fprintf(fp4, "%5.5f,%5.5f,%5.5f,%5.5f,%5.5f\n", sim_time[i], J4_pos[i], J4_vel[i], J4_cur[i], J4_cur_filter[i]);
//    }
//    fclose(fp1);
//    fclose(fp2);
//    fclose(fp3);
//    fclose(fp4);

    this->close();
}

void MainWindow::runBtnSlot(){
    if (ui->runBtn->text().compare("RUN") == 0){
        ui->runBtn->setText("STOP");
        run_timer->start();
        run = true;

        rt_task_start(&dxl_task, &MotorCtrlMain, NULL);
    }
    else{
        ui->runBtn->setText("RUN");
//        rt_task_suspend(&dxl_task);

        collision = false;
        collStartFlag = false;
        run_timer->stop();
        run = false;

        ofstream fp1, fp2, fp3, fp4;
        fp1.open("../SaveData/Axis1Data.csv");
        fp2.open("../SaveData/Axis2Data.csv");
        fp3.open("../SaveData/Axis3Data.csv");
        fp4.open("../SaveData/Axis4Data.csv");
//        printf("%d\n", sim_time.length());
        for(int i = 2; i < sim_time.length(); i++){
            fp1<<sim_time[i]<<","<<J1_pos[i]*P2D<<","<<J1_vel[i]*0.01*6*D2R<<","<<J1_cur[i]*0.001<<","<<J1_filter[i]<<","<<J1_enc1[i]<<","<<J1_enc2[i]<<","<<J1_tau[i]<<","<<J1_rhat[i]<<"\n";
            fp2<<sim_time[i]<<","<<J2_pos[i]*P2D<<","<<J2_vel[i]*0.01*6*D2R<<","<<J2_cur[i]*0.001<<","<<J2_filter[i]<<","<<J2_enc1[i]<<","<<J2_enc2[i]<<","<<J2_tau[i]<<","<<J2_rhat[i]<<"\n";
            fp3<<sim_time[i]<<","<<J3_pos[i]*P2D<<","<<J3_vel[i]*0.01*6*D2R<<","<<J3_cur[i]*0.001<<","<<J3_filter[i]<<","<<J3_enc1[i]<<","<<J3_enc2[i]<<","<<J3_tau[i]<<","<<J3_rhat[i]<<"\n";
            fp4<<sim_time[i]<<","<<J4_pos[i]*P2D<<","<<J4_vel[i]*0.01*6*D2R<<","<<J4_cur[i]*0.001<<","<<J4_filter[i]<<","<<J4_enc1[i]<<","<<J4_enc2[i]<<","<<J4_tau[i]<<","<<J4_rhat[i]<<"\n";
        }
        fp1.close();
        fp2.close();
        fp3.close();
        fp4.close();
    }
}

void MainWindow::readyBtnSlot(){
    ready = true;
    dxlControl.reset();
    if (motionA && !motionB){
        double J1des = -60*D2P;
        double J2des = 30*D2P;
        double J3des = 40*D2P;
        double J4des = -60*D2P;
        dxlControl.setPosition(J1offset + static_cast<int32_t>(J1des), 1);
        dxlControl.setPosition(-2988, 2);
        dxlControl.setPosition(-126028, 3);
        dxlControl.setPosition(J4offset + static_cast<int32_t>(J4des), 4);
    }
    if (!motionA && motionB) {
        double J1des = 0*D2P;
        double J2des = 30*D2P;
        double J3des = 30*D2P;
        double J4des = 0*D2P;
        dxlControl.setPosition(J1offset + (int32_t)J1des, 1);
        dxlControl.setPosition(J2offset + (int32_t)J2des, 2);
        dxlControl.setPosition(J3offset + (int32_t)J3des, 3);
        dxlControl.setPosition(J4offset + (int32_t)J4des, 4);
    }

    collision = false;
    J1CollPos = 0;
    J2CollPos = 0;
    J3CollPos = 0;
    J4CollPos = 0;

    J1_cur.clear(); J2_cur.clear(); J3_cur.clear(); J4_cur.clear();
    J1_filter.clear(); J2_filter.clear(); J3_filter.clear(); J4_filter.clear();
    J1_pos.clear(); J2_pos.clear(); J3_pos.clear(); J4_pos.clear();
    J1_vel.clear(); J2_vel.clear(); J3_vel.clear(); J4_vel.clear();
    J1_rhat.clear(); J2_rhat.clear(); J3_rhat.clear(); J4_rhat.clear();
    J1_enc1.clear(); J2_enc1.clear(); J3_enc1.clear(); J4_enc1.clear();
    J1_enc2.clear(); J2_enc2.clear(); J3_enc2.clear(); J4_enc2.clear();
    sim_time.clear();

    indx = 0;

    stop_flag = false;
}

void MainWindow::on_motionACb_stateChanged(int arg1)
{
    motionA = arg1;
    ui->motionBCb->setChecked(!motionA);
    motionB = !arg1;
}

void MainWindow::on_motionBCb_stateChanged(int arg1)
{
    motionB = arg1;
    ui->motionACb->setChecked(!motionB);
    motionA = !arg1;
}

void MainWindow::on_collisionCb_stateChanged(int arg1)
{
    collStartFlag = arg1;
//    cout << collStartFlag << endl;
    if (collStartFlag == 1){
        J1threshP += threshOffset;
        J2threshP += threshOffset;
        J3threshP += threshOffset;
        J4threshP += threshOffset;
        J1threshN -= threshOffset;
        J2threshN -= threshOffset;
        J3threshN -= threshOffset;
        J4threshN -= threshOffset;
        cout << "J1 : " << J1threshP << ", " << J1threshN << endl;
        cout << "J2 : " << J2threshP << ", " << J2threshN << endl;
        cout << "J3 : " << J3threshP << ", " << J3threshN << endl;
        cout << "J4 : " << J4threshP << ", " << J4threshN << endl;
    }
    else {
        J1threshP = -10000;
        J2threshP = -10000;
        J3threshP = -10000;
        J4threshP = -10000;

        J1threshN = 10000;
        J2threshN = 10000;
        J3threshN = 10000;
        J4threshN = 10000;
    }
}

void MainWindow::on_setting1Btn_clicked()
{
    int state = 0;
    state = dxlControl.dxl_init(DXL_ID_1);
    if (state == 1) {
        cout << "Axis 1 successfully connected!!" << endl;
        ui->setting1Btn->setDisabled(true);
        dxlControl.setPosition(J1offset, DXL_ID_1);
        ui->pos1SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_1));
    }
    else {
        cout << "Axis 1 fail connect..." << endl;
    }
}

void MainWindow::on_setting2Btn_clicked()
{
    int state = 0;
    state = dxlControl.dxl_init(DXL_ID_2);
    if (state == 1) {
        cout << "Axis 2 successfully connected!!" << endl;
        ui->setting2Btn->setDisabled(true);
        dxlControl.setPosition(J2offset, DXL_ID_2);
        ui->pos2SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_2));
    }
    else {
        cout << "Axis 2 fail connect..." << endl;
    }
}

void MainWindow::on_setting3Btn_clicked()
{
    int state = 0;
    state = dxlControl.dxl_init(DXL_ID_3);
    if (state == 1) {
        cout << "Axis 3 successfully connected!!" << endl;
        ui->setting3Btn->setDisabled(true);
        dxlControl.setPosition(J3offset, DXL_ID_3);
        ui->pos3SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_3));
    }
    else {
        cout << "Axis 3 fail connect..." << endl;
    }
}

void MainWindow::on_setting4Btn_clicked()
{
    int state = 0;
    state = dxlControl.dxl_init(DXL_ID_4);
    if (state == 1) {
        cout << "Axis 4 successfully connected!!" << endl;
        ui->setting4Btn->setDisabled(true);
        dxlControl.setPosition(J4offset, DXL_ID_4);
        ui->pos4SpinBox->setValue(dxlControl.getPresentPosition(DXL_ID_4));
    }
    else {
        cout << "Axis 4 fail connect..." << endl;
    }
}

void MainWindow::run_timer_out(){
    flag ^= true;
    run_timer->start();
}

void MainWindow::on_cutoffSpinBox_editingFinished()
{
    f_cut = ui->cutoffSpinBox->value();
}

void MainWindow::on_threshSpinBox_editingFinished()
{
    threshOffset = ui->threshSpinBox->value();
}

void MainWindow::on_emergen_clicked()
{
//    int motorState = 0;
//    dxlControl.reset();
//    do{
//        motorState = dxlControl.reset2();
//        cout << "Motor State : " << motorState << endl;
//    } while(motorState == 0);

//    collision = true;
//    collStartFlag = true;
    CollIndex = 5;
    stop_flag = true;
//    dxlControl.reset();
//    J1CollPos = dxlControl.getPresentPosition(1);
//    J2CollPos = dxlControl.getPresentPosition(2);
//    J3CollPos = dxlControl.getPresentPosition(3);
//    J4CollPos = dxlControl.getPresentPosition(4);
}
