#include "drone.hpp"
#include <Eigen/Core>
#include "quat.hpp"

namespace dyn
{

Drone::Drone()
{
    m_p.mixer << m_p.k1,m_p.k1,m_p.k1,m_p.k1, 0,-m_p.arm_len*m_p.k1,0,m_p.arm_len*m_p.k1,
            m_p.arm_len*m_p.k1,0,-m_p.arm_len*m_p.k1,0, -m_p.k2, m_p.k2, -m_p.k2, m_p.k2;
    m_states.setZero(STATE_SIZE,1);
    m_att_R = Eigen::Matrix3d::Identity();
//    m_cmds.setZero(CMD_SIZE,1);
//    m_inputs.setZero(INPUT_SIZE,1);
}

Drone::~Drone()
{
}

void Drone::sendAttitudeCmds(const cmdVec &cmds)
{

}

void Drone::sendMotorCmds(const uVec &inputs)
{
    uVec force_tau{m_p.mixer*inputs};
    this->derivatives(m_states, force_tau, m_rk4.k1);
    this->derivatives(m_states + m_rk4.dt/2*m_rk4.k1, force_tau, m_rk4.k2);
    this->derivatives(m_states + m_rk4.dt/2*m_rk4.k2, force_tau, m_rk4.k3);
    this->derivatives(m_states + m_rk4.dt*m_rk4.k3, force_tau, m_rk4.k4);
    m_states += m_rk4.dt/6 * (m_rk4.k1 + 2*m_rk4.k2 + 2*m_rk4.k3 + m_rk4.k4);
}

void Drone::derivatives(const xVec &x,const uVec &u,xVec &k)
{
    //    posd = Rb2v*[u;v;w];
    //    pnd = posd(1);
    //    ped = posd(2);
    //    hd  = -posd(3);

    quat::Quatd q_i2b{quat::Quat<double>::from_euler(x(RX),x(RY),x(RZ))};
    k.segment<3>(PX) = q_i2b.rota(x.segment<3>(VX));
    k(PZ) *= -1;

//    attd = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);0 cos(phi) -sin(phi);...
//            0 sin(phi)/cos(theta) cos(phi)/cos(theta)] * [p;q;r];
//    phid   = attd(1);
//    thetad = attd(2);
//    psid   = attd(3);
    m_att_R.block<2,3>(0,1) << sin(x(RX))*tan(x(RY)),cos(x(RX))*tan(x(RY)),
                               cos(x(RX)), -sin(x(RX)),
                               sin(x(RX))/cos(x(RY)),cos(x(RX))/cos(x(RY));
    k.segment<3>(RX) = m_att_R*x.segment<3>(WX);

//      % 0.1 seems to be a decent wind disturbance
//    N_wind = 0.0; %acceleration of quad from north wind
//    E_wind = 0.0; %acceleration of quad from east wind

//    ud = r*v-q*w - g*sin(theta) - mu/(4*mp+mc)*u +N_wind*cos(psi)+E_wind*sin(psi);
//    vd = p*w-r*u + g*cos(theta)*sin(phi) - mu/(4*mp+mc)*v -N_wind*sin(psi)+E_wind*cos(psi);
//    wd = q*u-p*v + g*cos(theta)*cos(phi) - F/(4*mp+mc) - mu/(4*mp+mc)*w;

    k.segment<3>(VX) = x.segment<3>(VX).cross(x.segment<3>(WX)) + q_i2b.rotp(quat::e3*m_p.grav)
                       - (quat::e3*u(U1) + m_p.mu*x.segment<3>(VX))/m_p.mass;
//    pd = (Jy-Jz)/Jx * q*r + T_phi/Jx;
//    qd = (Jz-Jx)/Jy * p*r + T_theta/Jy;
//    rd = (Jx-Jy)/Jz * p*q + T_psi/Jz;

    k.segment<3>(WX) = m_p.inertia_inv * (u.segment<3>(U2) - x.segment<3>(WX).cross(
                       m_p.inertia*x.segment<3>(WX)));
}

} //end namespace dyn
