#include "gtest/gtest.h"
#include "drone.hpp"

bool expectVecNear(const dyn::xVec &a,const dyn::xVec &b,double delta)
{
    dyn::xVec diff = (a-b).cwiseAbs();
    return diff.maxCoeff() < delta;
}

TEST(QuadcopterAtEquilibrium,givenEquilibriumInputs_DoesNotMove)
{
    dyn::Drone Quadcopter;
    dyn::uVec u{0.55,0.55,0.55,0.55};
    Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.00001));
}

TEST(QuadcopterAtEquilibrium,givenAboveEquilibriumInputs_MovesUp)
{
    dyn::Drone Quadcopter;
    dyn::uVec u{0.8,0.8,0.8,0.8};
    Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    expected_states(dyn::PZ) = 0.02232;
    expected_states(dyn::VZ) = -0.44665;

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.00001));
}
