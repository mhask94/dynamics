#include "gtest/gtest.h"
#include "drone.hpp"

TEST(QuadcopterAtEquilibrium,givenEquilibriumInputs_DoesNotMove)
{
    dyn::Drone Quadcopter;
    dyn::uVec u{0.55,0.55,0.55,0.55};
    Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_EQ(expected_states,actual_states);
}

TEST(QuadcopterAtEquilibrium,givenAboveEquilibriumInputs_MovesUp)
{
    dyn::Drone Quadcopter;
    dyn::uVec u{0.8,0.8,0.8,0.8};
    Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_EQ(expected_states,actual_states);
}
