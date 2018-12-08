#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "types.hpp"

class Controller
{
public:
    Controller();
    virtual ~Controller();
    dyn::uVec calculateControl(const dyn::xVec &states);

protected:
    void setX0(const dyn::xVec &x0);
    void load_data();

private:
    dyn::xVec m_x;
    dyn::xVec m_ref;
    dyn::uVec m_u;
};

#endif // CONTROLLER_HPP
