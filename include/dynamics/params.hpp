#ifndef PARAMS_HPP
#define PARAMS_HPP

#include <Eigen/Core>
#include "types.hpp"

namespace quadcopter
{

struct Params
{
    const double center_mass{2.0};
    const double prop_mass{0.25};
    const double mass{center_mass+4*prop_mass};
    const double arm_len{0.3};
    const double est_radius{0.1};
    const double inertia_x{0.4*center_mass*est_radius*est_radius+2*prop_mass*arm_len*arm_len};
    const double inertia_y{inertia_x};
    const double inertia_z{0.4*center_mass*est_radius*est_radius+4*prop_mass*arm_len*arm_len};
    Eigen::Matrix3d inertia_inv{Eigen::Vector3d(1/inertia_x,1/inertia_y,1/inertia_z).asDiagonal()};
    Eigen::Matrix3d inertia{Eigen::Vector3d(inertia_x,inertia_y,inertia_z).asDiagonal()};
    const double grav{9.81};
    const double mu{0.1};
    const double throttle_eq{0.55};
    dyn::uVec u_eq{throttle_eq,throttle_eq,throttle_eq,throttle_eq};
    const double k1{mass*grav/(4*throttle_eq)};
    const double k2{0.2};
    Eigen::Matrix4d mixer;
    Eigen::ColPivHouseholderQR<Eigen::Matrix4d> mixer_qr{4,4};

    Params()
    {
        setMixer();
    }

private:
    void setMixer()
    {
        mixer << k1,k1,k1,k1, 0,-arm_len*k1,0,arm_len*k1, arm_len*k1,0,-arm_len*k1,0, -k2, k2, -k2, k2;
        mixer_qr.compute(mixer);
    }
};

}// end namespace quadcopter

namespace fixedwing
{

struct Params
{
    dyn::params_t P;
    double Jx{0.8244};
    double Jy{1.135};
    double Jz{1.759};
    double Jxz{0.1204};

    Params()
    {
        setInertia();
        P.grav = 9.80665;
        P.mass = 13.5;
        P.mu = 0.1;
    }

    void setInertia()
    {
        double gamma{Jx*Jz-Jxz*Jxz};
        P.inertia << Jx,0,-Jxz, 0,Jy,0, -Jxz,0,Jz;
        P.inertia_inv << Jx/gamma,0,Jxz/gamma, 0,1/Jy,0, Jxz/gamma,0,Jx/gamma;
    }

    void setZeroJxzInertia()
    {
        P.inertia << Jx,0,0, 0,Jy,0, 0,0,Jz;
        P.inertia_inv << 1/Jx,0,0, 0,1/Jy,0, 0,0,1/Jz;
    }
};

} // end namespace fixedwing

#endif // PARAMS_HPP
