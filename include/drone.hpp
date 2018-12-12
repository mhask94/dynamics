#ifndef DRONE_HPP
#define DRONE_HPP

#include "types.hpp"

namespace dyn
{

class Drone
{   
public:
    Drone();
    virtual ~Drone();
    void sendAttitudeCmds(const cmdVec &cmds);
    void sendMotorCmds(const uVec &inputs);
    xVec getStates() const;
    uVec getEquilibriumInputs() const;
    double getDt(const bool milli=true) const;
    double setDt(const double dt);
    void derivatives(const xVec &getStates,const uVec &inputs,xVec &k);

private:
    typedef struct
    {
        xVec k1;
        xVec k2;
        xVec k3;
        xVec k4;
        double dt;
    } rk4_t;

    params_t m_p;
    xVec m_states;
    rk4_t m_rk4;
    Eigen::Matrix3d m_att_rot;
};

} // end namespace dyn
#endif // DRONE_HPP
