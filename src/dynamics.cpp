#include "dynamics.hpp"

namespace dyn
{

Dynamics::Dynamics()
{
}

Dynamics::~Dynamics()
{}

void Dynamics::updateDynamics(const Wrench& inputs)
{
    this->derivatives(m_states, inputs, m_rk4.k1);
    this->derivatives(m_states + m_rk4.k1*m_rk4.dt*0.5, inputs, m_rk4.k2);
    this->derivatives(m_states + m_rk4.k2*m_rk4.dt*0.5, inputs, m_rk4.k3);
    this->derivatives(m_states + m_rk4.k3*m_rk4.dt, inputs, m_rk4.k4);
    m_states += (m_rk4.k1 + m_rk4.k2*2 + m_rk4.k3*2 + m_rk4.k4) * (m_rk4.dt/6.0);
}

State Dynamics::getStates() const
{
    return m_states;
}

Input Dynamics::getEquilibriumInputs() const
{
    Input out;
    return out;
}

double Dynamics::getDt(const bool milli) const
{
    if (milli)
        return m_rk4.dt*1000;
    else
        return m_rk4.dt;
}

void Dynamics::setDt(const double dt)
{
    m_rk4.dt = dt;
}

void Dynamics::resetStates()
{
    m_states.vec.setZero();
    m_states.q = quat::Quatd::Identity();
}

void Dynamics::derivatives(const State& x,const Wrench& u,ErrorState& k)
{
    k.p = x.q.rota(x.v);
    k.q = x.w;
    k.v = x.v.cross(x.w) + x.q.rotp(quat::e3*m_p.grav) - (u.f + m_p.mu*x.v)/m_p.mass;
    k.w = m_p.inertia_inv*(u.tau - x.w.cross(m_p.inertia*x.w));
}
} // end namespace dyn
