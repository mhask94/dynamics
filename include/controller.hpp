#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "types.hpp"

class Controller
{
public:
    Controller();
    virtual ~Controller();
    dyn::uVec calculateControl(const dyn::xVec &states);
};

#endif // CONTROLLER_HPP
