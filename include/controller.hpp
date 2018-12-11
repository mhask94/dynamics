#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "types.hpp"

class Controller
{
public:
    Controller();
    virtual ~Controller();
    dyn::uVec calculateControl(const dyn::xVec &states);
    dyn::RotMatrix getRotation() const;
    dyn::MatrixA getA() const;
    dyn::MatrixA getAd() const;
    dyn::MatrixB getBd() const;

protected:
    void initializeA();
    void initializeB();
    void setX0(const dyn::xVec &x0);
    void setConstRef(const dyn::xVec &ref);
    void setA(const dyn::MatrixA &A);
    void setB(const dyn::MatrixB &B);
    void setStateWeights(const dyn::xVec &weights,bool final=false);
    void setInputWeights(const dyn::uVec &weights);
    void setEquilibriumInputs(const dyn::uVec &u_eq);
    void setInputLimits(double min,double max);
    void setSlewRate(double slew_rate);
    void linearizeAboutCurrentAttitude();
    void updateRotation();
    void updateA();
    void discretizeAB();
    void load_data();
    dyn::xVec m_x;

private:
    double m_rate;
    dyn::params_t m_p;
    dyn::xVec m_ref;
    dyn::uVec m_u;
    dyn::MatrixA m_A;
    dyn::MatrixA m_Ad;
    dyn::MatrixB m_B;
    dyn::MatrixB m_Bd;
    dyn::RotMatrix m_R_b2i;
};

#endif // CONTROLLER_HPP
