#ifndef DRONE_HPP
#define DRONE_HPP

#include "types.hpp"

namespace dyn
{

typedef struct
{
    double center_mass{2.0};
    double prop_mass{0.25};
    double mass{center_mass+4*prop_mass};
    double arm_len{0.3};
    double est_radius{0.1};
    double inertia_x{0.4*center_mass*est_radius*est_radius+2*prop_mass*arm_len*arm_len};
    double inertia_y{inertia_x};
    double inertia_z{0.4*center_mass*est_radius*est_radius+4*prop_mass*arm_len*arm_len};
    Eigen::Matrix3d inertia{Eigen::Vector3d(inertia_x,inertia_y,inertia_z).asDiagonal()};
    Eigen::Matrix3d inertia_inv{Eigen::Vector3d(1/inertia_x,1/inertia_y,1/inertia_z).asDiagonal()};
    double grav{9.81};
    double mu{0.1};
    double u_eq{0.55};
    double k1{mass*grav/(4*u_eq)};
    double k2{0.2};
    Eigen::Matrix4d mixer;
} params_t;

class Drone
{
public:
    Drone();
    virtual ~Drone();
    void sendAttitudeCmds(const cmdVec &cmds);
    void sendMotorCmds(const uVec &inputs);
    xVec getStates() const;
    double getDt(const bool milli=true) const;
    double setDt(const double dt);
    void derivatives(const xVec &getStates,const uVec &inputs,xVec &k);

private:
    params_t m_p;
    xVec m_states;
    rk4_t m_rk4;
    Eigen::Matrix3d m_att_rot;
};

} // end namespace dyn
#endif // DRONE_HPP
