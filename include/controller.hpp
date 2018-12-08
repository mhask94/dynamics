#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

extern "C"
{
#include "solver.h"
}
#include "types.hpp"
#include "nav_msgs/Odometry.h"

class Controller
{
public:
    Controller();
    virtual ~Controller();
    dyn::uVec calculateControl(const dyn::xVec &states);

protected:
    void load_data();

private:
    Vars m_vars;
    Params m_params;
    Workspace m_ws;
    Settings m_settings;
    dyn::xVec m_x;
    dyn::xVec m_ref;
    dyn::uVec m_u;
};

#endif // CONTROLLER_HPP
