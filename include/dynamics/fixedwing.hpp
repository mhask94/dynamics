#ifndef FIXEDWING_HPP
#define FIXEDWING_HPP

#include "dynamics.hpp"
#include "params.hpp"

class FixedWing : public dyn::Dynamics
{
public:
    FixedWing();
    virtual ~FixedWing();
    void sendWrench(const dyn::Wrench& inputs);
    void setInertia(bool zero_cross_terms=true);

private:
    fixedwing::Params m_params;
};

#endif // FIXEDWING_HPP
