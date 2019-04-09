#ifndef FIXEDWING_HPP
#define FIXEDWING_HPP

#include "dynamics.hpp"
#include "params.hpp"
#include "input.hpp"
#include "dynamics/winddynamics.hpp"

namespace fixedwing
{

struct State
{
    dyn::State dyn;
    double Va;
    double alpha{0};
    double beta{0};
    double Vg{0};
    double gamma{0};
    double chi{0};
    double wn{0};
    double we{0};
    double bx{0};
    double by{0};
    double bz{0};
};

struct Sensors
{
    double gyro_x{0};
    double gyro_y{0};
    double gyro_z{0};
    double accel_x{0};
    double accel_y{0};
    double accel_z{0};
    double static_pressure{0};
    double diff_pressure{0};
    double gps_n{0};
    double gps_e{0};
    double gps_h{0};
    double gps_Vg{0};
    double gps_chi{0};
};

} //end namespace fixedwing

class FixedWing : public dyn::Dynamics
{
public:
    FixedWing(int vehicle_type=fixedwing::CONVENTIONAL);
    virtual ~FixedWing();
    void sendWrench(const dyn::Wrench& inputs);
    void sendDeltas(const fixedwing::Input& deltas);
    void setWindSS(const Eigen::Vector3d& wind);
    void setUseGust(const bool use_gust);
    void resetStates();
    fixedwing::State getFixedwingStates() const;
    fixedwing::Sensors getSensorData() const;
    fixedwing::Input getEquilibriumInputs() const;

protected:
    void updateVelData();
    void updateFixedwingState();
    void updateSensors();
    void calcLonDyn(double pVa2S_2,double de,dyn::Wrench& f_tau);
    void calcLatDyn(double pVa2S_2,double da,double dr,dyn::Wrench& f_tau);
    void calcMotorDyn(double dt,dyn::Wrench& f_tau);

private:    
    int m_vehicle_type;
    fixedwing::Params m_p;
    fixedwing::State m_x;
    fixedwing::Sensors m_sensors;
    double m_fx;
    double m_fy;
    double m_fz;
    double m_t_gps;
    double m_gps_eta_n;
    double m_gps_eta_e;
    double m_gps_eta_h;
    sensor::Params m_sensor_p;
    Eigen::Vector3d m_wind_ss;
    dyn::WindDynamics m_wind_gust;
    bool m_use_gust;
};

#endif // FIXEDWING_HPP
