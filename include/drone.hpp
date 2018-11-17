#ifndef DRONE_HPP
#define DRONE_HPP

#include <Eigen/Core>
//#include <Eigen/Geometry>

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
    double center_mass{2.0};
    double prop_mass{0.25};
    double mass{center_mass+4*prop_mass};
    double arm_len{0.3};
    double est_radius{0.1};
    double inertia_x{0.4*center_mass*est_radius*est_radius+2*prop_mass*arm_len*arm_len};
    double inertia_y{inertia_x};
    double inertia_z{0.4*center_mass*est_radius*est_radius+4*prop_mass*arm_len*arm_len};
    Eigen::Matrix inertia = Eigen::Vector3d(inertia_x,inertia_y,inertia_z); //TODO
    Eigen::Matrix inertia_inv = Eigen::Vector3d(1/inertia_x,1/inertia_y,1/inertia_z); //TODO
    double grav{9.81};
    double mu{0.1};
    double u_eq{0.55};
    double k1{mass*grav/(4*u_eq)};
    double k2{0.1};
    Eigen::Matrix4d mixer;
}params_t;

typedef struct
{
    xVec k1;
    xVec k2;
    xVec k3;
    xVec k4;
    double dt;
}rk4_t;

//typedef struct
//{
//    double pn;
//    double pe;
//    double pd;

//    double phi;
//    double theta;
//    double psi;

//    double u;
//    double v;
//    double w;

//    double p;
//    double q;
//    double r;
//}States;

//typedef struct
//{
//    double u1;
//    double u2;
//    double u3;
//    double u4;

//    double F;
//    double phi_c;
//    double theta_c;
//    double r_c;
//}Inputs;

class Drone
{
public:
    Drone();
    virtual ~Drone();
    void sendAttitudeCmds(const cmdVec &cmds);
    void sendMotorCmds(const uVec &inputs);

private:
    params_t m_p;
    xVec m_states;
    rk4_t m_rk4;
    Eigen::Matrix3d m_att_R;
    void derivatives(const xVec &states,const uVec &inputs,xVec &k);
//    cmdVec m_cmds;
//    uVec m_inputs;
};

} // end namespace dyn
#endif // DRONE_HPP