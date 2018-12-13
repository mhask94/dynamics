#include "drone.hpp"
#include "quat.hpp"

namespace dyn
{

Drone::Drone()
{
    m_p.setMixer();
    m_states.setZero(STATE_SIZE,1);
    m_att_rot = Eigen::Matrix3d::Identity();
    m_rk4.dt = .002;
}

Drone::~Drone()
{
}

void Drone::sendAttitudeCmds(const cmdVec& cmds)
{

}

void Drone::sendMotorCmds(const uVec& inputs)
{
    uVec force_tau{m_p.mixer*inputs};
    this->derivatives(m_states, force_tau, m_rk4.k1);
    this->derivatives(m_states + m_rk4.dt/2.0*m_rk4.k1, force_tau, m_rk4.k2);
    this->derivatives(m_states + m_rk4.dt/2.0*m_rk4.k2, force_tau, m_rk4.k3);
    this->derivatives(m_states + m_rk4.dt*m_rk4.k3, force_tau, m_rk4.k4);
    m_states += m_rk4.dt/6.0 * (m_rk4.k1 + 2*m_rk4.k2 + 2*m_rk4.k3 + m_rk4.k4);
}

xVec Drone::getStates() const
{
    return m_states;
}

uVec Drone::getEquilibriumInputs() const
{
    return m_p.u_eq;
}

double Drone::getDt(const bool milli) const
{
    if (milli)
        return m_rk4.dt*1000;
    else
        return m_rk4.dt;
}

double Drone::setDt(const double dt)
{
    m_rk4.dt = dt;
}

void Drone::derivatives(const xVec& x,const uVec& u,xVec& k)
{
    quat::Quatd q_i2b{quat::Quatd::from_euler(x(RX),x(RY),x(RZ))};
    k.segment<3>(PX) = q_i2b.rota(x.segment<3>(VX));
    k(PZ) *= -1;

    m_att_rot.block<3,2>(0,1) << sin(x(RX))*tan(x(RY)),cos(x(RX))*tan(x(RY)),
                               cos(x(RX)), -sin(x(RX)),
                               sin(x(RX))/cos(x(RY)),cos(x(RX))/cos(x(RY));
    k.segment<3>(RX) = m_att_rot*x.segment<3>(WX);

    k.segment<3>(VX) = x.segment<3>(VX).cross(x.segment<3>(WX)) + q_i2b.rotp(quat::e3*m_p.grav)
                       - (quat::e3*u(U1) + m_p.mu*x.segment<3>(VX))/m_p.mass;

    k.segment<3>(WX) = m_p.inertia_inv * (u.segment<3>(U2) - x.segment<3>(WX).cross(
                       m_p.inertia*x.segment<3>(WX)));
}

void Drone::resetStates()
{
    m_states.setZero(dyn::STATE_SIZE,1);
}

} //end namespace dyn
