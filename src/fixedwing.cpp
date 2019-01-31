#include "fixedwing.hpp"

FixedWing::FixedWing(dyn::params_t params)
{
    m_p = params;
}

FixedWing::~FixedWing()
{}

void FixedWing::sendWrench(const dyn::Wrench &inputs)
{
    this->updateDynamics(inputs);
}
