#include "dynamics/fixedwing.hpp"

FixedWing::FixedWing()
{
    m_p = m_params.P;
    m_p.grav = 0;
    m_p.mu = 0;
    m_rk4.dt = 0.002;
}

FixedWing::~FixedWing()
{}

void FixedWing::sendWrench(const dyn::Wrench &inputs)
{
    this->updateDynamics(inputs);
}

void FixedWing::setInertia(bool zero_cross_terms)
{
    if (zero_cross_terms)
    {
        m_params.setZeroJxzInertia();
        m_p.inertia = m_params.P.inertia;
        m_p.inertia_inv = m_params.P.inertia_inv;
        m_p.grav = 0;
        m_p.mu = 0;
        m_rk4.dt = 0.002;
    }
    else
    {
       m_params.setInertia();
       m_p.inertia = m_params.P.inertia;
       m_p.inertia_inv = m_params.P.inertia_inv;
       m_p.grav = 0;
       m_p.mu = 0;
       m_rk4.dt = 0.002;
    }
}
