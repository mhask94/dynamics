#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <Eigen/Core>
#include "types.hpp"

namespace quadcopter
{

struct Params
{
    const double center_mass{2.0};
    const double prop_mass{0.25};
    const double mass{center_mass+4*prop_mass};
    const double arm_len{0.3};
    const double est_radius{0.1};
    const double inertia_x{0.4*center_mass*est_radius*est_radius+2*prop_mass*arm_len*arm_len};
    const double inertia_y{inertia_x};
    const double inertia_z{0.4*center_mass*est_radius*est_radius+4*prop_mass*arm_len*arm_len};
    Eigen::Matrix3d inertia_inv{Eigen::Vector3d(1/inertia_x,1/inertia_y,1/inertia_z).asDiagonal()};
    Eigen::Matrix3d inertia{Eigen::Vector3d(inertia_x,inertia_y,inertia_z).asDiagonal()};
    const double grav{9.81};
    const double mu{0.1};
    const double throttle_eq{0.55};
    dyn::uVec u_eq{throttle_eq,throttle_eq,throttle_eq,throttle_eq};
    const double k1{mass*grav/(4*throttle_eq)};
    const double k2{0.2};
    Eigen::Matrix4d mixer;
    Eigen::ColPivHouseholderQR<Eigen::Matrix4d> mixer_qr{4,4};

    Params()
    {
        setMixer();
    }

private:
    void setMixer()
    {
        mixer << k1,k1,k1,k1, 0,-arm_len*k1,0,arm_len*k1, arm_len*k1,0,-arm_len*k1,0, -k2, k2, -k2, k2;
        mixer_qr.compute(mixer);
    }
};

}// end namespace quadcopter

namespace fixedwing
{

struct Params
{
    dyn::params_t dyn;
    double Jx{0.8244};
    double Jy{1.135};
    double Jz{1.759};
    double Jxz{0.1204};
    // physical params
    double S_wing{0.55};
    double b{2.8956};
    double c{0.18994};
    double S_prop{0.2027};
    double rho{1.2682};
    double e{0.9};
    double AR{b*b/S_wing};
    //longitudinal coefficients
    double C_L_0{0.23};
    double C_L_alpha{5.61};
    double C_L_q{7.95};
    double C_L_de{0.13};

    double C_D_0{0.043};
    double C_D_alpha{0.03};
    double C_D_p{0.0};
    double C_D_q{0.0};
    double C_D_de{0.0135};

    double C_m_0{0.0135};
    double C_m_alpha{-2.74};
    double C_m_q{-38.21};
    double C_m_de{-0.99};
    // lateral coefficients
    double C_Y_0{0.0};
    double C_Y_beta{-0.98};
    double C_Y_p{0.0};
    double C_Y_r{0.0};
    double C_Y_da{0.075};
    double C_Y_dr{0.19};

    double C_l_0{0.0};
    double C_l_beta{-0.13};
    double C_l_p{-0.51};
    double C_l_r{0.25};
    double C_l_da{0.17};
    double C_l_dr{0.0024};

    double C_n_0{0.0};
    double C_n_beta{0.073};
    double C_n_p{0.069};
    double C_n_r{-0.095};
    double C_n_da{-0.011};
    double C_n_dr{-0.069};

//    double C_prop{1.0};
    double M{50.0};
    double alpha0{0.47};
    double epsilon{0.16};
    // prop/motor coefficients
    double D_prop{20*0.0254};
    double K_V{145.0};
    double KQ{(1/K_V) * 60/(2*3.14159)};
    double R_motor{0.042};
    double i0{1.5};
    int nCells{12};
    double V_max{4.0*nCells};
    double C_Q2{-0.01664};
    double C_Q1{0.004970};
    double C_Q0{0.005230};
    double C_T2{-0.1079};
    double C_T1{-0.06044};
    double C_T0{0.09357};

    Params()
    {
        setInertia();
        dyn.grav = 9.80665;
        dyn.mass = 11.0;
    }

    void setInertia()
    {
        double gamma{Jx*Jz-Jxz*Jxz};
        dyn.inertia << Jx,0,-Jxz, 0,Jy,0, -Jxz,0,Jz;
        dyn.inertia_inv << Jx/gamma,0,Jxz/gamma, 0,1/Jy,0, Jxz/gamma,0,Jx/gamma;
    }
};

enum
{
    CONVENTIONAL,
    V_TAIL,
    FLYING_WING
};

} // end namespace fixedwing

namespace sensor
{
    struct Params
    {
        double accel_sigma{0.0025*9.80665};
        double gyro_sigma{0.13*3.14159/180.0};
        double gyro_bias_x{0};
        double gyro_bias_y{0};
        double gyro_bias_z{0};
        double static_pres_sigma{0.01*1000};
        double diff_pres_sigma{0.002*1000};
        double gps_ts{1.0};
        double gps_beta{1.0/1100.0};
        double gps_sigma_n{0.21};
        double gps_sigma_e{0.21};
        double gps_sigma_h{0.4};
        double gps_sigma_Vg{0.05};
        double gps_sigma_chi{gps_sigma_Vg/10.0};
    };

} // end namespace sensor

#endif // PARAMS_HPP
