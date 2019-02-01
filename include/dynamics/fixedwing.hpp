#ifndef FIXEDWING_HPP
#define FIXEDWING_HPP

#include "dynamics.hpp"

class FixedWing : public dyn::Dynamics
{
public:
    FixedWing(dyn::params_t params);
    virtual ~FixedWing();
    void sendWrench(const dyn::Wrench& inputs);
};

#endif // FIXEDWING_HPP
