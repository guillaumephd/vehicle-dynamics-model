#include <boost/python/def.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/module.hpp>
#include <boost/python/numpy.hpp>
#include <cmath>
#include <ctime>
#include <iostream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

#define EPS 0.01
#define STATE_DIM 16
#define HELPER_T_DIM 10
#define CTRL_DIM 5
#define mathpi 3.141592653589793238462643383279502884197169399375105820974944592307816406286

using namespace std;
using namespace boost::python;

using std::sin;
using std::cos;
using std::tan;
using std::atan;
using std::asin;
using std::acos;

/**
*   \namespace params
*   \brief     Every constant parameter of the vehicle
*/
namespace params
{
    double gravity = 9.81;
    double rho_air = 1.2;

    double lf = 1.304;
    double lr = 1.788;

    double lw = 1.608;

    double r_eff = 0.34;
    double h_CoG = 0.715;
    double r0 = 0.35;

    double mass_susp = 1738;
    double mass = 1878;

    double Iz = 4045.3;
    double Ir = 1.3;
    double Ix = 692.7;
    double Iy = 3961.4;

    double k_susp_f = 42000;
    double d_susp_f = 4000;

    double k_susp_r = (lf / lr) *(lf / lr) * k_susp_f;
    double d_susp_r = (lf / lr) *(lf / lr) * d_susp_f;

    double front_surf = 2.675;
    double drag_coef = 0.35;

    double mass_wheel = (mass - mass_susp) / 4;

    double f0 = mass_susp*gravity / (2 * (lf + lr));
    double Fsusp0[4] = { f0*lr, f0*lr, f0*lf, f0*lf };
    double fz0[4] = { Fsusp0[0] + mass_wheel*gravity, Fsusp0[1] + mass_wheel*gravity ,Fsusp0[2] + mass_wheel*gravity ,Fsusp0[3] + mass_wheel*gravity };

    double aero_coef = 0.5 * rho_air*front_surf*drag_coef;

    double V_max = 50;
    double OmegaMax = V_max / r_eff;
    double rollMax = 10 * mathpi / 180;
    double pitchMax = 10 * mathpi / 180;

    double C_mot_max = 2 * 750;
    double C_fr_min = 4 * (-1250);
    double A_max = 2.5;
    double D_max = -9.8;

    double ColDirection = 18.24;
    double delta_max = 500 * (mathpi / (180 * ColDirection));
    double delta_min = -500 * (mathpi / (180 * ColDirection));
    double delta_braq = 450 * (mathpi / (180 * ColDirection));

    /**
    *   \namespace params::tires
    *   \brief     Every constant parameter of the vehicle tires
    */
    namespace tires
    {
        // pTire
        double p_Cx1 = 1.65;
        double p_Dx1 = 1;
        double p_Dx2 = 0;
        double p_Ex1 = -0.5;
        double p_Ex2 = 0;
        double p_Ex3 = 0;
        double p_Ex4 = 0;
        double p_Kx1 = 12;
        double p_Kx2 = 10;
        double p_Kx3 = -0.6;
        double p_Hx1 = 0;
        double p_Hx2 = 0;
        double p_Vx1 = 0;
        double p_Vx2 = 0;

        double p_Cy1 = 1.3;
        double p_Dy1 = 1;
        double p_Dy2 = 0;
        double p_Dy3 = 0;
        double p_Ey1 = -1;
        double p_Ey2 = 0;
        double p_Ey3 = 0;
        double p_Ey4 = 0;
        double p_Ky1 = 10;
        double p_Ky2 = 1.5;
        double p_Ky3 = 0;
        double p_Hy1 = 0;
        double p_Hy2 = 0;
        double p_Hy3 = 0.25;
        double p_Vy1 = 0;
        double p_Vy2 = 0;
        double p_Vy3 = 0.15;
        double p_Vy4 = 0;

        double q_Bz1 = 6;
        double q_Bz2 = -4;
        double q_Bz3 = 0.6;
        double q_Bz4 = 0;
        double q_Bz5 = 0;
        double q_Bz10 = 0.7;
        double q_Cz1 = 1.05;
        double q_Dz1 = 0.12;
        double q_Dz2 = -0.03;
        double q_Dz3 = 0;
        double q_Dz4 = -1;
        double q_Dz6 = 0;
        double q_Dz7 = 0;
        double q_Dz8 = 0.6;
        double q_Dz9 = 0.2;
        double q_Ez1 = -10;
        double q_Ez2 = 0;
        double q_Ez3 = 0;
        double q_Ez4 = 0;
        double q_Ez5 = 0;
        double q_Hz1 = 0;
        double q_Hz2 = 0;
        double q_Hz3 = 0;
        double q_Hz4 = 0;

        double r_Bx1 = 5;
        double r_Bx2 = 8;
        double r_Cx1 = 1;
        double r_Ex1 = 0;
        double r_Ex2 = 0;
        double r_Hx1 = 0;

        double r_By1 = 7;
        double r_By2 = 2.5;
        double r_By3 = 0;
        double r_Cy1 = 1;
        double r_Ey1 = 0;
        double r_Ey2 = 0;
        double r_Hy1 = 0;
        double r_Hy2 = 0;
        double r_Vy1 = 0;
        double r_Vy2 = 0;
        double r_Vy3 = -0.2;
        double r_Vy4 = 14;
        double r_Vy5 = 1.9;
        double r_Vy6 = 10;

        double s_sz1 = 0;
        double s_sz2 = -0.1;
        double s_sz3 = -1.0;
        double s_sz4 = 0;
    }
}

/**
*   \struct    state
*   \brief     The vehicle internal state
    The state consist of position, rotation, position derivative, rotation derivative, and the wheels angular rotation speed
*/
struct state
{
    double* __data = new double[STATE_DIM];
    double* x = &(this->__data[0]);
    double* vx = &(this->__data[1]);
    double* y = &(this->__data[2]);
    double* vy = &(this->__data[3]);
    double* z = &(this->__data[4]);
    double* vz = &(this->__data[5]);

    double* r = &(this->__data[6]);
    double* vr = &(this->__data[7]);
    double* p = &(this->__data[8]);
    double* vp = &(this->__data[9]);
    double* yaw = &(this->__data[10]);
    double* vyaw = &(this->__data[11]);

    double* om_fl = &(this->__data[12]);
    double* om_fr = &(this->__data[13]);
    double* om_rl = &(this->__data[14]);
    double* om_rr = &(this->__data[15]);

    void init(double* dat)
    {
        for (int i = 0; i < STATE_DIM; ++i)
            this->__data[i] = dat[i];
    }
};

/**
*   \struct    ctrl
*   \brief     The control to be applied to the vehicle at a given timestep
    The control to apply consist of the couple T on each of the 4 wheels (each wheel can have a specific couple) and the steering \delta of the vehicle
*/
struct ctrl
{
    double* __data = new double[CTRL_DIM];
    double* T = &(this->__data[0]);
    double* steer = &(this->__data[4]);

    void init(double* dat)
    {
        for (int i = 0; i < CTRL_DIM; ++i)
            this->__data[i] = dat[i];
    }
};

/**
*   \struct    helper_t
*   \brief     Used to keep track of other state values: ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4
*/
struct helper_t {
    double* __data = new double[HELPER_T_DIM];
    double* ax = &(this->__data[0]);
    double* ay = &(this->__data[1]);
    double* sr1 = &(this->__data[2]);
    double* sr2 = &(this->__data[3]);
    double* sr3 = &(this->__data[4]);
    double* sr4 = &(this->__data[5]);
    double* sa1 = &(this->__data[6]);
    double* sa2 = &(this->__data[7]);
    double* sa3 = &(this->__data[8]);
    double* sa4 = &(this->__data[9]);
    void init(double* dat)
    {
        for (int i = 0; i < HELPER_T_DIM; ++i)
            this->__data[i] = dat[i];
    }
};

/**
*   \fn        sign
*   \brief     Returns the sign of the argument. (i.e. +1 if the value provided is positive, and -1 otherwise.)
*/
double sign(double x)
{
    return (x >= 0) - (x < 0);
}

/**
*   \fn        compute_slip
*   \brief     TODO
*/
void compute_slip(const double* X, double steer, double* sr, double* sa)
{
    double xxx = params::lw / 2 * X[11];
    double yyyf = params::lf * X[11];
    double yyyr = params::lr * X[11];

    double V_pneu_x[4] = { X[1] - xxx, X[1] + xxx, X[1] - xxx, X[1] + xxx };
    double V_pneu_y[4] = { X[3] + yyyf, X[3] + yyyf, X[3] - yyyr, X[3] - yyyr };

    double c = std::cos(steer);
    double s = std::sin(steer);

    double V_tire_t[4] = { c*V_pneu_x[0] - s*V_pneu_y[0], c*V_pneu_x[1] - s*V_pneu_y[1], V_pneu_x[2], V_pneu_x[3] };

    for (int k = 0; k < 4; ++k)
    {
        if (X[k + 12] * V_tire_t[k] >= 0)
        {
            if (std::abs(params::r_eff*X[k + 12]) >= std::abs(V_tire_t[k]))
            {
                sr[k] = (params::r_eff*X[k + 12] - V_tire_t[k]) / (params::r_eff*X[k + 12] + sign(X[k + 12])*EPS);
            }
            else {
                sr[k] = (params::r_eff*X[k + 12] - V_tire_t[k]) / (V_tire_t[k] + sign(V_tire_t[k])*EPS);
            }
        }
        else if (V_tire_t[k] < 0) {
            sr[k] = 1;
        }
        else {
            sr[k] = -1;
        }

        sa[k] = -std::atan2(V_pneu_y[k], V_pneu_x[k] + EPS*sign(V_pneu_x[k]));
        if (k < 2)
            sa[k] += steer;
    }
}

/**
*   \fn        tire_forces
*   \brief     TODO
*/
void tire_forces(const double* tau_x, const double* slip_angle, const double* fz, const double* gamma, const double* mu, double* Fxp, double* Fyp, double* dFz)
{
    double tau_shift[4];
    double slip_angle_shift[4];
    double Fxp0[4];
    double Fyp0[4];
    double G_xa[4];
    double G_yk[4];

    for (int k = 0; k < 4; ++k)
    {
        double dfz = (fz[k] - params::fz0[k]) / params::fz0[k];
        dFz[k] = dfz;

        double S_Hx = params::tires::p_Hx1;
        tau_shift[k] = tau_x[k] + S_Hx;
        double C_x = params::tires::p_Cx1;
        double mu_x = mu[k] * (params::tires::p_Dx1 + params::tires::p_Dx2*dfz);
        double D_x = mu_x*fz[k];
        double E_x = (params::tires::p_Ex1 + params::tires::p_Ex2*dfz + params::tires::p_Ex3*dfz*dfz)*(1 - params::tires::p_Ex4*sign(tau_shift[k]));
        double K_xk = fz[k] * (params::tires::p_Kx1 + params::tires::p_Kx2*dfz)*exp(params::tires::p_Kx3*dfz);
        double B_x = K_xk / (C_x*D_x);
        double S_Vx = fz[k] * (params::tires::p_Vx1 + params::tires::p_Vx2*dfz);

        Fxp0[k] = D_x*sin(C_x*atan(B_x*tau_shift[k] - E_x*(B_x*tau_shift[k] - atan(B_x*tau_shift[k])))) + S_Vx;

        double S_Hy = (params::tires::p_Hy1 + params::tires::p_Hy2*dfz) + params::tires::p_Hy3*gamma[k];
        slip_angle_shift[k] = slip_angle[k] + S_Hy;

        double C_y = params::tires::p_Cy1;
        double mu_y = mu[k] * (params::tires::p_Dy1 + params::tires::p_Dy2*dfz)*(1 - params::tires::p_Dy3*gamma[k] * gamma[k]);
        double D_y = mu_y*fz[k];
        double E_y = (params::tires::p_Ey1 + params::tires::p_Ey2*dfz)*(1 - (params::tires::p_Ey3 + params::tires::p_Ey4*gamma[k] * sign(slip_angle_shift[k])));
        double K_ya0 = params::tires::p_Ky1*params::fz0[k] * sin(2 * atan(fz[k] / (params::tires::p_Ky2*params::fz0[k])));
        double K_ya = K_ya0*(1 - params::tires::p_Ky3*gamma[k] * gamma[k]);
        double B_y = K_ya / (C_y*D_y);
        double S_Vy = fz[k] * ((params::tires::p_Vy1 + params::tires::p_Vy2*dfz) + (params::tires::p_Vy3 + params::tires::p_Vy4*dfz)*gamma[k]);

        Fyp0[k] = D_y*sin(C_y*atan(B_y*slip_angle_shift[k] - E_y*(B_y*slip_angle_shift[k] - atan(B_y*slip_angle_shift[k])))) + S_Vy;

        double B_xa = params::tires::r_Bx1*cos(atan(params::tires::r_Bx2*tau_x[k]));
        double C_xa = params::tires::r_Cx1;
        double E_xa = params::tires::r_Ex1 + params::tires::r_Ex2*dfz;
        double S_Hxa = params::tires::r_Hx1;

        slip_angle_shift[k] = slip_angle[k] + S_Hxa;

        G_xa[k] = (cos(C_xa*atan(B_xa*slip_angle_shift[k] - E_xa*(B_xa*slip_angle_shift[k] - atan(B_xa*slip_angle_shift[k]))))) / (cos(C_xa*atan(B_xa*S_Hxa - E_xa*(B_xa*S_Hxa - atan(B_xa*S_Hxa)))));

        Fxp[k] = G_xa[k] * Fxp0[k];

        double B_yk = params::tires::r_By1*cos(atan(params::tires::r_By2*(slip_angle[k] - params::tires::r_By3)));
        double C_yk = params::tires::r_Cy1;
        double E_yk = params::tires::r_Ey1 + params::tires::r_Ey2*dfz;
        double S_Hyk = params::tires::r_Hy1 + params::tires::r_Hy2*dfz;
        double D_Vyk = mu_y*fz[k] * (params::tires::r_Vy1 + params::tires::r_Vy2*dfz + params::tires::r_Vy3*gamma[k])*cos(atan(params::tires::r_Vy4*slip_angle[k]));
        double S_Vyk = D_Vyk*sin(params::tires::r_Vy5*atan(params::tires::r_Vy6*tau_x[k]));

        tau_shift[k] = tau_x[k] + S_Hyk;
        G_yk[k] = (cos(C_yk*atan(B_yk*tau_shift[k] - E_yk*(B_yk*tau_shift[k] - atan(B_yk*tau_shift[k]))))) / (cos(C_yk*atan(B_yk*S_Hyk - E_yk*(B_yk*S_Hyk - atan(B_yk*S_Hyk)))));
        Fyp[k] = G_yk[k] * Fyp0[k] + S_Vyk;
    }
}

/**
*   \fn        compute_susp_forces
*   \brief     TODO
*/
void compute_susp_forces(const double* X, double* F_susp, double* Delta_z_susp, double* d_z_susp)
{
    for (int k = 0; k < 2; ++k)
    {
        double sgn = (k % 2) ? -1. : 1.;
        Delta_z_susp[k] = sgn*params::lw / 2 * sin(X[6]) - params::lf*cos(X[6])*sin(X[8]);
        d_z_susp[k] = sgn*params::lw / 2 * X[7] * cos(X[6]) + params::lf*X[7] * sin(X[6])*sin(X[8]) - params::lf*X[9] * cos(X[6]) * cos(X[8]);
        F_susp[k] = -params::k_susp_f*Delta_z_susp[k] - params::d_susp_f*d_z_susp[k];
    }

    for (int k = 2; k < 4; ++k)
    {
        double sgn = (k % 2) ? -1. : 1.;
        Delta_z_susp[k] = sgn*params::lw / 2 * sin(X[6]) + params::lr*cos(X[6])*sin(X[8]);
        d_z_susp[k] = sgn*params::lw / 2 * X[7] * cos(X[6]) - params::lr*X[7] * sin(X[6])*sin(X[8]) + params::lr*X[9] * cos(X[6]) * cos(X[8]);
        F_susp[k] = -params::k_susp_f*Delta_z_susp[k] - params::d_susp_f*d_z_susp[k];
    }
}

/**
*   \fn        force_fame_change
*   \brief     TODO
*/
void force_fame_change(const double* Fxp, const double* Fyp, const double* Fz, const double* X, double steer, double* Fx, double* Fy)
{
    double delta_steer[4] = { steer, steer, 0, 0 };
    for (int k = 0; k < 4; ++k)
    {
        Fx[k] = (Fxp[k] * cos(delta_steer[k]) - Fyp[k] * sin(delta_steer[k]))*cos(X[8]) - Fz[k] * sin(X[8]);
        Fy[k] = (Fxp[k] * cos(delta_steer[k]) - Fyp[k] * sin(delta_steer[k]))*sin(X[6])*sin(X[8]) + (Fyp[k] * cos(delta_steer[k]) + Fxp[k] * sin(delta_steer[k]))*cos(X[6]) - Fz[k] * sin(X[6])*cos(X[8]);
    }
}

/**
*   \fn        comp_derivative
*   \brief     TODO
*/
void comp_derivative(const double* X, const double* u, const double* mu, double* dX)
{
    double tau_x[4];
    double slip_angle[4];

    double steer = u[4];

    compute_slip(X, steer, tau_x, slip_angle);

    double Delta_F_susp[4];
    double Delta_z_susp[4];
    double d_z_susp[4];
    compute_susp_forces(X, Delta_F_susp, Delta_z_susp, d_z_susp);

    double Fz[4];
    for (int k = 0; k < 4; ++k)
        Fz[k] = params::fz0[k] + Delta_F_susp[k];

    double gamma[4] = { 0,0,0,0 };
    double Fxp[4];
    double Fyp[4];
    double dFz[4];
    tire_forces(tau_x, slip_angle, Fz, gamma, mu, Fxp, Fyp, dFz);


    double Fx[4];
    double Fy[4];
    force_fame_change(Fxp, Fyp, Fz, X, steer, Fx, Fy);

    double F_aero = params::aero_coef * X[1] * X[1];

    double sFx = 0;
    double sFy = 0;
    for (int k = 0; k < 4; ++k)
    {
        sFx += Fx[k];
        sFy += Fy[k];
    }

    dX[0] = X[1] * cos(X[10]) - X[3] * sin(X[10]);
    dX[1] = X[11] * X[3] - X[9] * X[5] + (Fx[0] + Fx[1] + Fx[2] + Fx[3] - F_aero) / params::mass;
    dX[2] = X[1] * sin(X[10]) + X[3] * cos(X[10]);
    dX[3] = -X[11] * X[1] + X[7] * X[5] + (Fy[0] + Fy[1] + Fy[2] + Fy[3]) / params::mass;
    dX[4] = X[5];
    dX[5] = 0;
    dX[6] = X[7];
    dX[7] = 1. / (params::Ix)*(params::lw / 2 * (Delta_F_susp[0] + Delta_F_susp[2] - Delta_F_susp[1] - Delta_F_susp[3]) + X[4] * sFy);
    dX[8] = X[9];
    dX[9] = 1 / params::Iy *(-params::lf*(Delta_F_susp[0] + Delta_F_susp[1]) + params::lr*(Delta_F_susp[2] + Delta_F_susp[3]) - X[4] * sFx);
    dX[10] = X[11];
    dX[11] = 1. / params::Iz*(params::lf*(Fy[0] + Fy[1]) - params::lr*(Fy[2] + Fy[3]) + params::lw / 2.*(Fx[1] + Fx[3] - Fx[0] - Fx[2]));
    for (int k = 0; k < 4; ++k)
        dX[k + 12] = (u[k] - params::r_eff*Fxp[k]) / params::Ir;
}

/**
*   \fn        propagate
*   \brief     TODO
*/
void propagate(const double* X, const double* dX, double h, double* Xnew)
{
    for (int i = 0; i < STATE_DIM; ++i)
        Xnew[i] = X[i] + dX[i] * h;

    for (int k = 0; k < 4; ++k)
        Xnew[k + 12] = std::fmax(std::fmin(Xnew[k + 12], params::V_max / params::r_eff), 0.);
    Xnew[1] = std::fmax(std::fmin(Xnew[1], params::V_max), 0.);
}

/**
*   \fn        rk4
*   \brief     Applies a fourth-order Runge-Kutta approach to approximate the solution of the differential equations.
*/
state rk4(const state &X, const ctrl &u, const double* mu, double h, helper_t& helper)
{
    state Xnew;
    double* Xtmp = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX1 = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX2 = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX3 = (double*)malloc(sizeof(double)*STATE_DIM);
    double* dX4 = (double*)malloc(sizeof(double)*STATE_DIM);

    comp_derivative(X.__data, u.__data, mu, dX1); // k1

    propagate(X.__data, dX1, h / 2, Xtmp); // Xtmp = X + h/2 dX1

    comp_derivative(Xtmp, u.__data, mu, dX2); // k2

    propagate(X.__data, dX2, h / 2, Xtmp); // Xtmp = X + h/2 dX2
    comp_derivative(Xtmp, u.__data, mu, dX3); // k3

    propagate(X.__data, dX3, h, Xtmp);
    comp_derivative(Xtmp, u.__data, mu, dX4); // k4

    double tau_x[4];
    double slip_angle[4];

    for (int i = 0; i < STATE_DIM; ++i)
    {
        Xnew.__data[i] = X.__data[i] + h / 6. * (dX1[i] + 2 * dX2[i] + 2 * dX3[i] + dX4[i]);
        if (i == 1 || i == 3)
        {
            helper.__data[(int)(i-1)/2] = 1./6.*(dX1[i] + 2 * dX2[i] + 2 * dX3[i] + dX4[i]);
        }
    }

    compute_slip(Xnew.__data, *u.steer, tau_x, slip_angle);

    for (int i = 0; i < 4; ++i)
    {
        helper.__data[2+i] = tau_x[i];
        helper.__data[2+4+i] = slip_angle[i];
    }

    free(Xtmp);
    free(dX1);
    free(dX2);
    free(dX3);
    free(dX4);

    return Xnew;
}

/**
*   \fn        simulate
*   \brief     Compute the state of the vehicle at each timestep under the controls provided.
*/
std::vector<state> simulate(state X0, std::vector<ctrl> controls, double* mu, double dt, std::vector<helper_t>& helper)
{
    helper_t _helper;
    std::vector<state> out;
    out.reserve(controls.size());
    out.push_back(X0);
    state curState = X0;
    for (int i = 0; i < controls.size(); ++i)
    {
        curState = rk4(curState, controls[i], mu, dt, _helper);
        out.push_back(curState);
        helper.push_back(_helper);
    }
    return out;
}

/**
*   \fn        run
*   \brief     Run a simulation from python and return the vehicle simulated states back to python
*/
boost::python::numpy::ndarray run(int duration, double dt, boost::python::numpy::ndarray initial_conditions_ndarray,
    boost::python::numpy::ndarray controls_ndarray, boost::python::numpy::ndarray mu_ndarray)
{
    // mu = 1 (for each wheel) when it's sunny
    double mu[4] = {
        boost::python::extract<double>(mu_ndarray[0]),
        boost::python::extract<double>(mu_ndarray[1]),
        boost::python::extract<double>(mu_ndarray[2]),
        boost::python::extract<double>(mu_ndarray[3]),
    };

    // Initial state of the vehicle
    state state_X_init;
    // Xg Vx Yg Vy Zg Vz roll droll pitch dpitch yaw dyaw omega1 omega2 omega3 omega4 ax ay sr1 sr2 sr3 sr4 sa1 sa2 sa3 sa4 u1 u2 u3 u4 delta
    double x = boost::python::extract<double>(initial_conditions_ndarray[0]);
    double vx = boost::python::extract<double>(initial_conditions_ndarray[1]);
    double y = boost::python::extract<double>(initial_conditions_ndarray[2]);
    double vy = boost::python::extract<double>(initial_conditions_ndarray[3]);
    double z = boost::python::extract<double>(initial_conditions_ndarray[4]);
    double vz = boost::python::extract<double>(initial_conditions_ndarray[5]);
    double r = boost::python::extract<double>(initial_conditions_ndarray[6]);
    double vr = boost::python::extract<double>(initial_conditions_ndarray[7]);
    double p = boost::python::extract<double>(initial_conditions_ndarray[8]);
    double vp = boost::python::extract<double>(initial_conditions_ndarray[9]);
    double yaw = boost::python::extract<double>(initial_conditions_ndarray[10]);
    double vyaw = boost::python::extract<double>(initial_conditions_ndarray[11]);
    *state_X_init.x = x;
    *state_X_init.vx = vx;
    *state_X_init.y = y;
    *state_X_init.vy = vy;
    *state_X_init.z = z;
    *state_X_init.vz = vz;
    *state_X_init.r = r;
    *state_X_init.vr = vr;
    *state_X_init.p = p;
    *state_X_init.vp = vp;
    *state_X_init.yaw = yaw;
    *state_X_init.vyaw = vyaw;
    *state_X_init.om_fl = *state_X_init.vx / params::r_eff;
    *state_X_init.om_fr = *state_X_init.vx / params::r_eff;
    *state_X_init.om_rl = *state_X_init.vx / params::r_eff;
    *state_X_init.om_rr = *state_X_init.vx / params::r_eff;

    // Controls to apply at each timestep
    std::vector<ctrl> my_ctrls;
    std::vector<helper_t> my_helpers;
    for (int i = 0; i < duration; ++i)
    {
        if (controls_ndarray.get_nd() == 1 && controls_ndarray.shape(0) == 5)
        {
            // constant control to apply
            ctrl m_ctrl;
            double T_front_left = boost::python::extract<double>(controls_ndarray[0]);
            double T_front_right = boost::python::extract<double>(controls_ndarray[1]);
            double T_back_left = boost::python::extract<double>(controls_ndarray[2]);
            double T_back_right = boost::python::extract<double>(controls_ndarray[3]);
            double steer = boost::python::extract<double>(controls_ndarray[4]);
            m_ctrl.T[0] = T_front_left;
            m_ctrl.T[1] = T_front_right;
            m_ctrl.T[2] = T_back_left;
            m_ctrl.T[3] = T_back_right;
            *m_ctrl.steer = steer;
            my_ctrls.push_back(m_ctrl);
        }
        else if (controls_ndarray.get_nd() == 2 && controls_ndarray.shape(0) == duration && controls_ndarray.shape(1) == 5)
        {
            // this timestep's control to apply
            double T_front_left = boost::python::extract<double>(controls_ndarray[i][0]);
            double T_front_right = boost::python::extract<double>(controls_ndarray[i][1]);
            double T_back_left = boost::python::extract<double>(controls_ndarray[i][2]);
            double T_back_right = boost::python::extract<double>(controls_ndarray[i][3]);
            double steer = boost::python::extract<double>(controls_ndarray[i][4]);
            ctrl m_ctrl;
            m_ctrl.T[0] = T_front_left;
            m_ctrl.T[1] = T_front_right;
            m_ctrl.T[2] = T_back_left;
            m_ctrl.T[3] = T_back_right;
            *m_ctrl.steer = steer;
            my_ctrls.push_back(m_ctrl);
        }
        else {
            cerr << "The controls you provide are not formatted as they should." << endl;
            exit(EXIT_FAILURE);
        }
    }
    
    // Run the vehicle simulation
    std::vector<state> res_states = simulate(state_X_init, my_ctrls, mu, dt, my_helpers);

    // Return values in a python numpy array
    boost::python::tuple shape = boost::python::make_tuple(duration, 1 + STATE_DIM + HELPER_T_DIM);
    boost::python::numpy::dtype dtype = boost::python::numpy::dtype::get_builtin<float>();
    boost::python::numpy::ndarray res_numpy_array = boost::python::numpy::zeros(shape, dtype);
    for (int i = 0; i < duration; i += 1)
    {
        // elapsed time
        res_numpy_array[i][0] = i * dt;
        // state
        res_numpy_array[i][1] = *res_states[i].x;
        res_numpy_array[i][2] = *res_states[i].vx;
        res_numpy_array[i][3] = *res_states[i].y;
        res_numpy_array[i][4] = *res_states[i].vy;
        res_numpy_array[i][5] = *res_states[i].z;
        res_numpy_array[i][6] = *res_states[i].vz;
        res_numpy_array[i][7] = *res_states[i].r;
        res_numpy_array[i][8] = *res_states[i].vr;
        res_numpy_array[i][9] = *res_states[i].p;
        res_numpy_array[i][10] = *res_states[i].vp;
        res_numpy_array[i][11] = *res_states[i].yaw;
        res_numpy_array[i][12] = *res_states[i].vyaw;
        res_numpy_array[i][13] = *res_states[i].om_fl;
        res_numpy_array[i][14] = *res_states[i].om_fr;
        res_numpy_array[i][15] = *res_states[i].om_rl;
        res_numpy_array[i][16] = *res_states[i].om_rr;
        // additional state
        res_numpy_array[i][17] = *my_helpers[i].ax;
        res_numpy_array[i][18] = *my_helpers[i].ay;
        res_numpy_array[i][19] = *my_helpers[i].sr1;
        res_numpy_array[i][20] = *my_helpers[i].sr2;
        res_numpy_array[i][21] = *my_helpers[i].sr3;
        res_numpy_array[i][22] = *my_helpers[i].sr4;
        res_numpy_array[i][23] = *my_helpers[i].sa1;
        res_numpy_array[i][24] = *my_helpers[i].sa2;
        res_numpy_array[i][25] = *my_helpers[i].sa3;
        res_numpy_array[i][26] = *my_helpers[i].sa4;
    }
    return res_numpy_array;
}

/**
*   \fn        cpp_version
*   \brief     Get the C++ code compilation date and time as a python string
*/
boost::python::object cpp_version()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);
    std::string str(buffer);

    return boost::python::str("Compiled on " + str);
}

/**
*   BOOST_PYTHON_MODULE
*   \brief     Expose the c++ functions to python
*   Important: the name provided to the macro (i.e. vehiclesim) must match the name of the .so generated file
*   Important: initialize boost before defining the functions exposed
*/
BOOST_PYTHON_MODULE(vehiclesim)
{
    Py_Initialize();
    boost::python::numpy::initialize();
    def("run", run);
    def("cpp_version", cpp_version);
}
