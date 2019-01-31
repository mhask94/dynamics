#ifndef TYPES_HPP
#define TYPES_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "quat.hpp"

namespace dyn
{

struct ErrorState
{
    enum
    {
        SIZE = 12
    };

    Eigen::Matrix<double,SIZE,1> vec;
    Eigen::Map<Eigen::Vector3d> p;
    Eigen::Map<Eigen::Vector3d> q;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> w;

    ErrorState() :
        p{vec.data()},
        q{vec.data()+3},
        v{vec.data()+6},
        w{vec.data()+9}
    {}

    ErrorState(const ErrorState& dx) :
        p{vec.data()},
        q{vec.data()+3},
        v{vec.data()+6},
        w{vec.data()+9}
    {
        vec = dx.vec;
    }

    ErrorState& operator =(const ErrorState& dx)
    {
        vec = dx.vec;
        return *this;
    }

    ErrorState operator *(const double& scalar)
    {
        ErrorState out;
        out.vec = scalar * vec;
        return out;
    }

    ErrorState operator +(const ErrorState& dx)
    {
        ErrorState out;
        out.vec = dx.vec + vec;
        return out;
    }
};

struct State
{
    enum
    {
        SIZE = 13
    };

    Eigen::Matrix<double,SIZE,1> vec;
    Eigen::Map<Eigen::Vector3d> p;
    quat::Quatd q;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> w;

    State() :
        p{vec.data()},
        q{vec.data()+3},
        v{vec.data()+7},
        w{vec.data()+10}
    {
        vec.setZero();
        q = quat::Quatd::Identity();
    }

    State(const State& x) :
        p{vec.data()},
        q{vec.data()+3},
        v{vec.data()+7},
        w{vec.data()+10}
    {
        vec = x.vec;
    }

    State& operator =(const State& x)
    {
        vec = x.vec;
        return *this;
    }

    State operator +(const ErrorState& dx)
    {
        State out;
        out.p = p + dx.p;
        out.q = q + dx.q;
        out.v = v + dx.v;
        out.w = w + dx.w;
        return out;
    }

    State& operator +=(const ErrorState& dx)
    {
        p = p + dx.p;
        q = q + dx.q;
        v = v + dx.v;
        w = w + dx.w;
        return *this;
    }

    ErrorState operator -(const State& x) const
    {
        ErrorState out;
        out.p = p - x.p;
        out.q = q - x.q;
        out.v = v - x.v;
        out.w = w - x.w;
        return out;
    }
};

struct Wrench
{
    enum
    {
        SIZE = 6
    };

    Eigen::Matrix<double,SIZE,1> vec;
    Eigen::Map<Eigen::Vector3d> f;
    Eigen::Map<Eigen::Vector3d> tau;

    Wrench() :
        f{vec.data()},
        tau{vec.data()+3}
    {
        vec.setZero();
    }

    Wrench(const Wrench& obj) :
        f{vec.data()},
        tau{vec.data()+3}
    {
        vec = obj.vec;
    }
};

struct Input
{
    enum
    {
        SIZE = 4
    };

    Eigen::Matrix<double,SIZE,1> vec;
    double& T;
    Eigen::Map<Eigen::Vector3d> tau;
    Eigen::Map<Eigen::Vector3d> del;

    Input():
        T{*vec.data()},
        tau{vec.data()+1},
        del{vec.data()+1}
    {
        vec.setZero();
    }
};

enum
{
    PX = 0,
    PY = 1,
    PZ = 2,
    RX = 3,
    RY = 4,
    RZ = 5,
    VX = 6,
    VY = 7,
    VZ = 8,
    WX = 9,
    WY = 10,
    WZ = 11,
    STATE_SIZE = 12,
};
typedef Eigen::Matrix<double,STATE_SIZE,1> xVec;
typedef Eigen::Matrix<double,STATE_SIZE,STATE_SIZE> MatrixA;
typedef Eigen::Matrix3d RotMatrix;

enum
{
    THRUST,
    ROLL_C,
    PITCH_C,
    YAW_RATE_C,
    CMD_SIZE
};
typedef Eigen::Matrix<double,CMD_SIZE,1> cmdVec;

enum
{
    U1,
    U2,
    U3,
    U4,
    INPUT_SIZE
};
typedef Eigen::Matrix<double,INPUT_SIZE,1> uVec;
typedef Eigen::Matrix<double,STATE_SIZE,INPUT_SIZE> MatrixB;

typedef struct
{
    double mass;
    Eigen::Matrix3d inertia_inv;
    Eigen::Matrix3d inertia;
    double grav{9.80655};
    double mu;
} params_t;

//typedef struct
//{
//    const double center_mass{2.0};
//    const double prop_mass{0.25};
//    const double mass{center_mass+4*prop_mass};
//    const double arm_len{0.3};
//    const double est_radius{0.1};
//    const double inertia_x{0.4*center_mass*est_radius*est_radius+2*prop_mass*arm_len*arm_len};
//    const double inertia_y{inertia_x};
//    const double inertia_z{0.4*center_mass*est_radius*est_radius+4*prop_mass*arm_len*arm_len};
//    Eigen::Matrix3d inertia_inv{Eigen::Vector3d(1/inertia_x,1/inertia_y,1/inertia_z).asDiagonal()};
//    Eigen::Matrix3d inertia{Eigen::Vector3d(inertia_x,inertia_y,inertia_z).asDiagonal()};
//    const double grav{9.81};
//    const double mu{0.1};
//    const double throttle_eq{0.55};
//    uVec u_eq{throttle_eq,throttle_eq,throttle_eq,throttle_eq};
//    const double k1{mass*grav/(4*throttle_eq)};
//    const double k2{0.2};
//    Eigen::Matrix4d mixer;
//    Eigen::ColPivHouseholderQR<Eigen::Matrix4d> mixer_qr{4,4};
//    void setMixer()
//    {
//        mixer << k1,k1,k1,k1, 0,-arm_len*k1,0,arm_len*k1, arm_len*k1,0,-arm_len*k1,0, -k2, k2, -k2, k2;
//        mixer_qr.compute(mixer);
//    }
//} params_t; //TODO can give a better name

} // end namespace dyn

#endif // TYPES_HPP
