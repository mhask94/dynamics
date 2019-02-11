#include "dynamics/fixedwing.hpp"
#include <math.h>

FixedWing::FixedWing(int vehicle_type) :
    m_vehicle_type{vehicle_type}
{
    m_params = m_p.dyn;
    m_rk4.dt = 0.002;
    m_x.wn = 0.0;
    m_x.we = 0.0;
    m_wind_ss << m_x.wn, m_x.we, 0.0;
    m_wind_gust.setZero();
    m_states.p(2) = -20;
    m_states.v(0) = 25;
    m_x.Va = m_states.v.norm();
//    m_params.grav = 0;
}

FixedWing::~FixedWing()
{}

void FixedWing::sendWrench(const dyn::Wrench& inputs)
{
    this->updateDynamics(inputs);
    this->updateVelData();
}

void FixedWing::sendDeltas(const fixedwing::Input &deltas)
{
    double pVa2S_2{0.5*m_p.rho*m_x.Va*m_x.Va*m_p.S_wing};
    dyn::Wrench inputs;

    this->calcLonDyn(pVa2S_2,deltas.de,inputs);
    this->calcLatDyn(pVa2S_2,deltas.da,deltas.dr,inputs);
    this->calcMotorDyn(deltas.dt,inputs);

    this->sendWrench(inputs);
}

void FixedWing::setWindSS(const Eigen::Vector3d& wind)
{
    m_wind_ss = wind;
}

void FixedWing::setWindGust(const Eigen::Vector3d& wind)
{
    m_wind_gust = wind;
}

void FixedWing::updateVelData()
{
    Eigen::Vector3d Vw_body;
    Vw_body = m_states.q.rotp(m_wind_ss) + m_wind_gust;

    Eigen::Vector3d Va_body;
    Va_body = m_states.v - Vw_body;

    m_x.Va = Va_body.norm();
    m_x.alpha = std::atan2(Va_body(2),Va_body(0));
    m_x.beta = std::asin(Va_body(1)/m_x.Va);
}

void FixedWing::calcLonDyn(double pVa2S_2,double de,dyn::Wrench& f_tau)
{
    double num{1+std::exp(-m_p.M*(m_x.alpha-m_p.alpha0))+std::exp(m_p.M*(m_x.alpha+m_p.alpha0))};
    double den{(1+std::exp(-m_p.M*(m_x.alpha-m_p.alpha0)))*(1+std::exp(m_p.M*(m_x.alpha+m_p.alpha0)))};
    double sigma{(num/den)};
    int sign = (m_x.alpha >= 0) ? 1 : -1;
    double sin{std::sin(m_x.alpha)};
    double cos{std::cos(m_x.alpha)};
    double C_L{(1-sigma)*(m_p.C_L_0+m_p.C_L_alpha*m_x.alpha)+sigma*2*sign*sin*sin*cos};
    double C_D{m_p.C_D_p + std::pow(m_p.C_L_0+m_p.C_L_alpha*m_x.alpha,2)/(3.14159*m_p.AR*m_p.e)};

    double lift{pVa2S_2*(C_L+m_p.C_L_q*m_p.c*m_states.w(1)/2.0/m_x.Va + m_p.C_L_de*de)};
    double drag{pVa2S_2*(C_D+m_p.C_D_q*m_p.c*m_states.w(1)/2.0/m_x.Va + m_p.C_D_de*de)};
    double m{pVa2S_2*m_p.c*(m_p.C_m_0+m_p.C_m_alpha*m_x.alpha+m_p.C_m_q*m_p.c*m_states.w(1)/2.0/m_x.Va+m_p.C_m_de*de)};

    f_tau.f(0) = (-drag*cos + lift*sin);
    f_tau.f(2) = (-drag*sin - lift*cos);
    f_tau.tau(1) = m;
}

void FixedWing::calcLatDyn(double pVa2S_2,double da,double dr,dyn::Wrench &f_tau)
{
    double fy{pVa2S_2*(m_p.C_Y_0+m_p.C_Y_beta*m_x.beta+m_p.b/(2.0*m_x.Va)*(m_p.C_Y_p*m_states.w(0)+
              m_p.C_Y_r*m_states.w(2))+m_p.C_Y_da*da+m_p.C_Y_dr*dr)};
    double l{pVa2S_2*m_p.b*(m_p.C_l_0+m_p.C_l_beta*m_x.beta+m_p.b/(2.0*m_x.Va)*(m_p.C_l_p*m_states.w(0)+
             m_p.C_l_r*m_states.w(2))+m_p.C_l_da*da+m_p.C_l_dr*dr)};
    double n{pVa2S_2*m_p.b*(m_p.C_n_0+m_p.C_n_beta*m_x.beta+m_p.b/(2.0*m_x.Va)*(m_p.C_n_p*m_states.w(0)+
             m_p.C_n_r*m_states.w(2))+m_p.C_n_da*da+m_p.C_n_dr*dr)};

    f_tau.f(1) = fy;
    f_tau.tau(0) = l;
    f_tau.tau(2) = n;
}

void FixedWing::calcMotorDyn(double dt,dyn::Wrench &f_tau)
{
    double V_in{m_p.V_max*dt};
    double a{m_p.rho*std::pow(m_p.D_prop,5)/std::pow(2*3.14159,2)*m_p.C_Q0};
    double b{m_p.rho*std::pow(m_p.D_prop,4)/(2*3.14159)*m_p.C_Q1*m_x.Va+m_p.KQ*m_p.KQ/m_p.R_motor};
    double c{m_p.rho*std::pow(m_p.D_prop,3)*m_p.C_Q2*m_x.Va*m_x.Va-m_p.KQ/m_p.R_motor*V_in+m_p.KQ*m_p.i0};
    double Omega_op{(-b+std::sqrt(b*b-4*a*c))/(2*a)};
    double J_op{2*3.14159*m_x.Va/(Omega_op*m_p.D_prop)};
    double n{Omega_op/(2*3.14159)};

    double C_T{m_p.C_T2*J_op*J_op+m_p.C_T1*J_op+m_p.C_T0};
    double C_Q{m_p.C_Q2*J_op*J_op+m_p.C_Q1*J_op+m_p.C_Q0};
    double T_p{m_p.rho*n*n*std::pow(m_p.D_prop,4)*C_T};
    double Q_p{m_p.rho*n*n*std::pow(m_p.D_prop,5)*C_Q};

    f_tau.f(0) += T_p;
    f_tau.tau(0) += Q_p;
}

void FixedWing::resetStates()
{
    m_states.vec.setZero();
    m_states.q = quat::Quatd::Identity();
    m_states.p(2) = -20;
    m_states.v(0) = 25;
    m_x.Va = m_states.v.norm();
}
