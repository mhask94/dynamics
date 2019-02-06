#ifndef TYPES_HPP
#define TYPES_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include "geometry/quat.h"

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

struct Wind
{
    enum
    {
        SIZE = 6
    };
    Eigen::Matrix<double,SIZE,1> vec;
    Eigen::Map<Eigen::Vector3d> ss;
    Eigen::Map<Eigen::Vector3d> gust;

    Wind() :
        ss{vec.data()},
        gust{vec.data()+3}
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
} params_t;

} // end namespace dyn

#endif // TYPES_HPP
