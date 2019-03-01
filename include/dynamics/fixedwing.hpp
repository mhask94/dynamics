#ifndef FIXEDWING_HPP
#define FIXEDWING_HPP

#include "dynamics.hpp"
#include "params.hpp"
#include "input.hpp"

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

} //end namespace fixedwing

class FixedWing : public dyn::Dynamics
{
public:
    FixedWing(int vehicle_type=fixedwing::CONVENTIONAL);
    virtual ~FixedWing();
    void sendWrench(const dyn::Wrench& inputs);
    void sendDeltas(const fixedwing::Input& deltas);
    void setWindSS(const Eigen::Vector3d& wind);
    void setWindGust(const Eigen::Vector3d& wind);
    void resetStates();
    fixedwing::State getFixedwingStates() const;
    fixedwing::Input getEquilibriumInputs() const;

protected:
    void updateVelData();
    void calcLonDyn(double pVa2S_2,double de,dyn::Wrench& f_tau);
    void calcLatDyn(double pVa2S_2,double da,double dr,dyn::Wrench& f_tau);
    void calcMotorDyn(double dt,dyn::Wrench& f_tau);

private:    
    int m_vehicle_type;
    fixedwing::Params m_p;
    fixedwing::State m_x;
    Eigen::Vector3d m_wind_ss;
    Eigen::Vector3d m_wind_gust;
};

#endif // FIXEDWING_HPP
