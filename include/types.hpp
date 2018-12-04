#ifndef TYPES_HPP
#define TYPES_HPP

#include <eigen3/Eigen/Core>

namespace dyn
{

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

typedef struct
{
    xVec k1;
    xVec k2;
    xVec k3;
    xVec k4;
    double dt;
} rk4_t;

} // end namespace dyn

#endif // TYPES_HPP
