#include "gtest/gtest.h"
#include "drone.hpp"

bool expectVecNear(const dyn::xVec &a,const dyn::xVec &b,double delta)
{
    dyn::xVec diff = (a-b).cwiseAbs();
    return diff.maxCoeff() < delta;
}

TEST(QuadcopterAtEquilibrium,GivenEquilibriumInputs_DoesNotMove)
{
    dyn::Drone Quadcopter;
    double eq{0.55};
    dyn::uVec u{eq,eq,eq,eq};
    Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.000001));
}

TEST(QuadcopterAtEquilibrium,GivenAboveEquilibriumInputs_MovesUp)
{
    dyn::Drone Quadcopter;
    double above_eq{0.8};
    dyn::uVec u{above_eq,above_eq,above_eq,above_eq};
    for (int i{0}; i < 500; i++)
        Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    expected_states(dyn::PZ) = 2.204978;
    expected_states(dyn::VZ) = -4.385592;

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.000001));
}

TEST(QuadcopterAtEquilibrium,GivenInputsToYawCCW_YawsCCW)
{
    dyn::Drone Quadcopter;
    double eq{0.55};
    double eq_off{0.1};
    dyn::uVec u{eq+eq_off,eq-eq_off,eq+eq_off,eq-eq_off};
    for (int i{0}; i < 500; i++)
        Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    expected_states(dyn::RZ) = -0.408163;
    expected_states(dyn::WZ) = -0.816327;

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.000001));
}

TEST(QuadcopterAtEquilibrium,GivenInputsToYawCW_YawsCW)
{
    dyn::Drone Quadcopter;
    double eq{0.55};
    double eq_off{0.1};
    dyn::uVec u{eq-eq_off,eq+eq_off,eq-eq_off,eq+eq_off};
    for (int i{0}; i < 500; i++)
        Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    expected_states(dyn::RZ) = 0.408163;
    expected_states(dyn::WZ) = 0.816327;

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.000001));
}

TEST(QuadcopterAtEquilibrium,GivenInputsToRoll_Rolls)
{
    dyn::Drone Quadcopter;
    double eq{0.55};
    double eq_off{0.1};
    dyn::uVec u{eq,eq-eq_off,eq,eq+eq_off};
    for (int i{0}; i < 100; i++)
        Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    expected_states(dyn::PY) = 0.009859;
    expected_states(dyn::VY) = 0.192859;
    expected_states(dyn::PZ) = -0.000598;
    expected_states(dyn::VZ) = -0.041511;
    expected_states(dyn::RX) = 0.302882;
    expected_states(dyn::WX) = 3.028816;

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.000001));
}

TEST(QuadcopterAtEquilibrium,GivenInputsToPitch_Pitches)
{
    dyn::Drone Quadcopter;
    double eq{0.55};
    double eq_off{0.1};
    dyn::uVec u{eq+eq_off,eq,eq-eq_off,eq};
    for (int i{0}; i < 100; i++)
        Quadcopter.sendMotorCmds(u);

    dyn::xVec expected_states;
    expected_states.setZero(dyn::STATE_SIZE,1);
    expected_states(dyn::PX) = -0.009859;
    expected_states(dyn::VX) = -0.192859;
    expected_states(dyn::PZ) = -0.000598;
    expected_states(dyn::VZ) = -0.041511;
    expected_states(dyn::RY) = 0.302882;
    expected_states(dyn::WY) = 3.028816;

    dyn::xVec actual_states{Quadcopter.getStates()};

    EXPECT_TRUE(expectVecNear(expected_states,actual_states,0.000001));
}
