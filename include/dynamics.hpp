#ifndef DYNAMICS_HPP
#define DYNAMICS_HPP

#include "types.hpp"

namespace dyn
{

class Dynamics
{
public:
    Dynamics();
    virtual ~Dynamics();
    void updateDynamics(const Wrench& inputs);
    State getStates() const;
    Input getEquilibriumInputs() const;
    double getDt(const bool milli=true) const;
    void setDt(const double dt);
    void resetStates();

protected:
    void derivatives(const State& states,const Wrench& inputs,ErrorState& k);

    typedef struct
    {
        ErrorState k1;
        ErrorState k2;
        ErrorState k3;
        ErrorState k4;
        double dt;
    } rk4_t;

    params_t m_p;
    double m_mass;
    State m_states;
    rk4_t m_rk4;
};

} //end namespace dyn
#endif // DYNAMICS_HPP
