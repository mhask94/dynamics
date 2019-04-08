#include "dynamics/fixedwing.hpp"
#include <math.h>
#include <random>

double rand(double mean, double sigma)
{
    static std::default_random_engine eng;
    static std::normal_distribution<double> dist{0,1};
    double random_num{dist(eng)};

    return mean + sigma*random_num;
}

FixedWing::FixedWing(int vehicle_type) :
    m_vehicle_type{vehicle_type},
    m_t_gps{999},
    m_gps_eta_n{0},
    m_gps_eta_e{0},
    m_gps_eta_h{0},
    m_use_gust{false}
{
    m_params = m_p.dyn;
    m_rk4.dt = 0.002;
    m_x.wn = 0.0;
    m_x.we = 0.0;
    m_wind_ss << m_x.wn, m_x.we, 0.0;
    m_states.p(2) = -50;
    m_states.v(0) = 25;
    m_x.Va = m_states.v.norm();
    m_x.dyn = m_states;

    double test1 = rand(5, 0.5);
    double test2 = rand(2, 0.1);
    double test3 = rand(-10, 1.0);
}

FixedWing::~FixedWing()
{}

void FixedWing::sendWrench(const dyn::Wrench& inputs)
{
    this->updateDynamics(inputs);
    this->updateVelData();
    this->updateFixedwingState();
    this->updateSensors();
}

void FixedWing::sendDeltas(const fixedwing::Input &deltas)
{
    double pVa2S_2{0.5*m_p.rho*m_x.Va*m_x.Va*m_p.S_wing};
    dyn::Wrench inputs;

    this->calcLonDyn(pVa2S_2,deltas.de,inputs);
    this->calcLatDyn(pVa2S_2,deltas.da,deltas.dr,inputs);
    this->calcMotorDyn(deltas.dt,inputs);

    m_fx = inputs.f(0);
    m_fy = inputs.f(1);
    m_fz = inputs.f(2);

    this->sendWrench(inputs);
}

void FixedWing::setWindSS(const Eigen::Vector3d& wind)
{
    m_wind_ss = wind;
}

void FixedWing::setUseGust(const bool use_gust)
{
    m_use_gust = use_gust;
}

fixedwing::State FixedWing::getFixedwingStates() const
{
    return m_x;
}

fixedwing::Input FixedWing::getEquilibriumInputs() const
{
    fixedwing::Input eq_input;
    eq_input.vec << -0.124778073, 0.676752115, 1.83617600e-03, -3.02606668e-04;
    return eq_input;
}

void FixedWing::updateVelData()
{
    Eigen::Vector3d Vw_body;
    Eigen::Vector3d wind_gust;
    if (m_use_gust)
        wind_gust = m_wind_gust.update(m_x.Va);
    else
        wind_gust.setZero();
    Vw_body = m_states.q.rotp(m_wind_ss) + wind_gust;

    Eigen::Vector3d Va_body;
    Va_body = m_states.v - Vw_body;

    m_x.wn = Vw_body(0);
    m_x.we = Vw_body(1);
    m_x.Va = Va_body.norm();
    if (Va_body(0) == 0.0)
        m_x.alpha = (Va_body(2) > 0) ? 3.14159/2.0 : -3.14159/2.0;
    else
        m_x.alpha = std::atan2(Va_body(2),Va_body(0));
    if (Va_body(0) == 0.0 && Va_body(2) == 0.0)
        m_x.beta = (Va_body(1) > 0) ? 3.14159/2.0 : -3.14159/2.0;
    else
        m_x.beta = std::asin(Va_body(1)/m_x.Va);
}

void FixedWing::updateFixedwingState()
{
    m_x.dyn = m_states;
    Eigen::Vector3d Vg_i{m_states.q.rota(m_states.v)};
    m_x.Vg = Vg_i.norm();
    m_x.gamma = asin(-Vg_i(2) / m_x.Vg);
    m_x.chi = atan2(Vg_i(1), Vg_i(0));
}

void FixedWing::updateSensors()
{
    m_sensors.gyro_x = m_states.w(0) + rand(m_sensor_p.gyro_bias_x,m_sensor_p.gyro_sigma);
    m_sensors.gyro_y = m_states.w(1) + rand(m_sensor_p.gyro_bias_y,m_sensor_p.gyro_sigma);
    m_sensors.gyro_z = m_states.w(2) + rand(m_sensor_p.gyro_bias_z,m_sensor_p.gyro_sigma);
    m_sensors.accel_x = rand(m_fx, m_sensor_p.accel_sigma);
    m_sensors.accel_y = rand(m_fy, m_sensor_p.accel_sigma);
    m_sensors.accel_z = rand(m_fz, m_sensor_p.accel_sigma);
    double pgh{m_p.rho*m_params.grav*-m_states.p(2)};
    double pressure_bias{0};
    m_sensors.static_pressure = pgh + rand(pressure_bias,m_sensor_p.static_pres_sigma);
    double pVa2_2{0.5*m_p.rho*m_x.Va*m_x.Va};
    m_sensors.diff_pressure = pVa2_2 + rand(pressure_bias,m_sensor_p.diff_pres_sigma);

    if (m_t_gps >= m_sensor_p.gps_ts)
    {
        double C_gps{exp(-m_sensor_p.gps_beta*m_sensor_p.gps_ts)};
        m_gps_eta_n = rand(C_gps*m_gps_eta_n,m_sensor_p.gps_sigma_n);
        m_gps_eta_e = rand(C_gps*m_gps_eta_e,m_sensor_p.gps_sigma_e);
        m_gps_eta_h = rand(C_gps*m_gps_eta_h,m_sensor_p.gps_sigma_h);
        m_sensors.gps_n = m_states.p(0) + m_gps_eta_n;
        m_sensors.gps_e = m_states.p(1) + m_gps_eta_e;
        m_sensors.gps_h = -m_states.p(2) + m_gps_eta_h;
        m_sensors.gps_Vg = rand(m_x.Vg, m_sensor_p.gps_sigma_Vg);
        m_sensors.gps_chi = rand(m_x.chi, m_sensor_p.gps_sigma_chi);
        m_t_gps = 0;
    }
    else
        m_t_gps += m_rk4.dt;
}

void FixedWing::calcLonDyn(double pVa2S_2,double de,dyn::Wrench& f_tau)
{
    double num{1+std::exp(-m_p.M*(m_x.alpha-m_p.alpha0))+std::exp(m_p.M*(m_x.alpha+m_p.alpha0))};
    double den{(1+std::exp(-m_p.M*(m_x.alpha-m_p.alpha0)))*(1+std::exp(m_p.M*(m_x.alpha+m_p.alpha0)))};
    double sigma{(num/den)};
    int sign = (m_x.alpha >= 0) ? 1 : -1;
    double sin{std::sin(m_x.alpha)};
    double cos{std::cos(m_x.alpha)};
    double unitless_q{m_p.c/(2*m_x.Va) * m_states.w(1)};

    double C_L{(1-sigma)*(m_p.C_L_0+m_p.C_L_alpha*m_x.alpha)+sigma*2*sign*sin*sin*cos};
    double C_D{m_p.C_D_p + std::pow(m_p.C_L_0+m_p.C_L_alpha*m_x.alpha,2)/(3.14159*m_p.AR*m_p.e)};

    double lift{pVa2S_2*(C_L + m_p.C_L_q*unitless_q + m_p.C_L_de*de)};
    double drag{pVa2S_2*(C_D + m_p.C_D_q*unitless_q + m_p.C_D_de*de)};
    double m{pVa2S_2*m_p.c*(m_p.C_m_0 + m_p.C_m_alpha*m_x.alpha + m_p.C_m_q*unitless_q + m_p.C_m_de*de)};

    f_tau.f(0) = (-drag*cos + lift*sin);
    f_tau.f(2) = (-drag*sin - lift*cos);
    f_tau.tau(1) = m;
}

void FixedWing::calcLatDyn(double pVa2S_2,double da,double dr,dyn::Wrench &f_tau)
{
    double unitless_p{m_p.b/(2*m_x.Va) * m_states.w(0)};
    double unitless_r{m_p.b/(2*m_x.Va) * m_states.w(2)};

    double fy{pVa2S_2 * (m_p.C_Y_0 + m_p.C_Y_beta*m_x.beta + m_p.C_Y_p*unitless_p +
              m_p.C_Y_r*unitless_r + m_p.C_Y_da*da + m_p.C_Y_dr*dr)};
    double l{pVa2S_2*m_p.b * (m_p.C_l_0 + m_p.C_l_beta*m_x.beta + m_p.C_l_p*unitless_p +
             m_p.C_l_r*unitless_r + m_p.C_l_da*da + m_p.C_l_dr*dr)};
    double n{pVa2S_2*m_p.b * (m_p.C_n_0 + m_p.C_n_beta*m_x.beta + m_p.C_n_p*unitless_p +
             m_p.C_n_r*unitless_r + m_p.C_n_da*da + m_p.C_n_dr*dr)};

    f_tau.f(1) = fy;
    f_tau.tau(0) = l;
    f_tau.tau(2) = n;
}

void FixedWing::calcMotorDyn(double dt,dyn::Wrench &f_tau)
{
    double pi2{2*3.14159};

    double V_in{m_p.V_max*dt};

    double a{m_p.rho * std::pow(m_p.D_prop,5)/std::pow(pi2,2) * m_p.C_Q0};
    double b{m_p.rho * std::pow(m_p.D_prop,4)/pi2 * m_p.C_Q1*m_x.Va + m_p.KQ*m_p.KQ/m_p.R_motor};
    double c{m_p.rho*std::pow(m_p.D_prop,3)*m_p.C_Q2*m_x.Va*m_x.Va - m_p.KQ/m_p.R_motor*V_in+m_p.KQ*m_p.i0};

    double Omega_op{(-b+std::sqrt(b*b-4*a*c))/(2*a)};
    double n{Omega_op/(2*3.14159)};
    double J_op{m_x.Va/(n*m_p.D_prop)};

    double C_T{m_p.C_T2*J_op*J_op + m_p.C_T1*J_op + m_p.C_T0};
    double C_Q{m_p.C_Q2*J_op*J_op + m_p.C_Q1*J_op + m_p.C_Q0};

    double T_p{m_p.rho * n*n * std::pow(m_p.D_prop,4) * C_T};
    double Q_p{m_p.rho * n*n * std::pow(m_p.D_prop,5) * C_Q};

    f_tau.f(0) += T_p;
    f_tau.tau(0) -= Q_p;
}

void FixedWing::resetStates()
{
    m_states.vec.setZero();
    m_states.q = quat::Quatd::Identity();
    m_states.p(2) = -20;
    m_states.v(0) = 25;
    m_x.Va = m_states.v.norm();
}
