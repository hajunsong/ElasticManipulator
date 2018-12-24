#ifndef DOB_H
#define DOB_H

#if !defined(_WIN32) && !defined(_WIN64)
    #include <QElapsedTimer>
#else
    #include <time.h>
#endif
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace std;

typedef unsigned int uint;
inline void tilde(double *a, double *b) {
    *(b++) = 0;	*(b++) = -a[2];	*(b++) = a[1];
    *(b++) = a[2];	*(b++) = 0;	*(b++) = -a[0];
    *(b++) = -a[1];	*(b++) = a[0];	*(b++) = 0;
}

class Body
{
public:
    Body();
    ~Body();
    // base body information
    double A0[9], C01[9], s01p[3], r0[3];
    // body initial data
    double qi, qi_dot, mi;
    double ri[3], ri_dot[3], wi[3], rhoip[3], sijp[3], Jip[9], Cii[9], Cij[9];
    // Orientation
    double Aijpp[9], Ai[9], Hi[3], u_vec[3];
    // Position
    double sij[3], rhoi[3], ric[3], rit[9];
    // Velocity State
    double Bi[6], Yih[6];
    // Cartesian velocity state
    double Ti[36], wit[9], Yib[6], ric_dot[3];
    // Mass & Force
    double Jic[9], rict[9], rict_dot[9], Mih[36], Fic[3], Tic[3], Qih[6], Qih_g[6], Qih_c[6];
    // Velocity Coupling
    double rit_dot[9], dHi[3], Di[6];
    // System EQM
    double Ki[36], Li[6], Li_g[6], Li_c[6];
    // Acceleration
    double qi_ddot;
    // Control
    double des_vel, err_vel, err_vel_accum, err_vel_prev, T_control, T_control_vel, Kp_vel, Ki_vel, Kd_vel;
    // Residual
    double Tg, Tc, alpha;
    double r_hat, K, p, Ta, Td, yp;
    double tau;
};

inline Body::Body(){
    u_vec[0] = 0;
    u_vec[1] = 0;
    u_vec[2] = 1;

    qi = 0;
    qi_dot = 0;
    qi_ddot = 0;
    mi = 0;

    Tg = 0;
    Tc = 0;

    des_vel = 0;
    err_vel = 0;
    err_vel_accum = 0;
    err_vel_prev = 0;
    T_control = 0;
    T_control_vel = 0;
    Kp_vel = 0;
    Ki_vel = 0;
    Kd_vel = 0;

    r_hat = 0;
    K = 0;
    p = 0;
    Ta = 0;
    Td = 0;
    yp = 0;

    tau = 0;
}

inline Body::~Body(){}

class DOB
{
public:
    DOB(uint num_body);
    ~DOB();
    void run(double *q, double *q_dot, double *tau, double *rhat);
    double total_time, average_time;
private:
    uint num_body;
    Body* body;

    double start_time, end_time, h, g, t_current;

    double *Y, *Yp, *Y_old, *Yp_old;

    void analysis();
        void kinematics_analysis();
        void generalized_mass_force();
        void residual_analysis();
};

#endif // DOB_H
