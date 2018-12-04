#include "controller.hpp"

Controller::Controller()
{

}

Controller::~Controller()
{

}

dyn::uVec Controller::calculateControl(const dyn::xVec &states)
{
    double u_equilibrium{0.55};
    dyn::uVec inputs{u_equilibrium,u_equilibrium,u_equilibrium,u_equilibrium};
    return inputs;
}
