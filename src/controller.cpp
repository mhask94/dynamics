#include "controller.hpp"
#include <unsupported/Eigen/MatrixFunctions>

extern "C"
{
#include "solver.h"
}
Vars vars;
Params params;
Workspace work;
Settings settings;

Controller::Controller()
{
    m_rate = 0.01;
    m_p.setMixer();
    m_x.setZero(dyn::STATE_SIZE,1);
    m_ref.setZero(dyn::STATE_SIZE,1);
    m_u = m_p.u_eq;
    this->initializeA();
    this->initializeB();
//    m_R_b2i << 1,0,0, 0,1,0, 0,0,1;
    this->linearizeAboutCurrentAttitude();
    load_data();
    set_defaults();
    setup_indexing();
}

Controller::~Controller()
{

}

dyn::uVec Controller::calculateControl(const dyn::xVec &states)
{
    double u_equilibrium{0.55};
    dyn::uVec inputs{u_equilibrium,u_equilibrium,u_equilibrium,u_equilibrium};

//    int num_iters;
//    num_iters = solve();

    return inputs;
}

dyn::RotMatrix Controller::getRotation() const
{
    return m_R_b2i;
}

dyn::MatrixA Controller::getA() const
{
    return m_A;
}

dyn::MatrixA Controller::getAd() const
{
    return m_Ad;
}

dyn::MatrixB Controller::getBd() const
{
    return m_Bd;
}

void Controller::initializeA()
{
    m_A.setZero(dyn::STATE_SIZE,dyn::STATE_SIZE);
    m_A(0,6) = 1;
    m_A(1,7) = 1;
    m_A(2,8) = -1;
    m_A(3,9) = 1;
    m_A(4,10) = 1;
    m_A(5,11) = 1;
    m_A(6,4) = -m_p.grav;
    m_A(7,3) = m_p.grav;
    m_A(6,6) = -m_p.mu/m_p.mass;
    m_A(7,7) = -m_p.mu/m_p.mass;
    m_A(8,8) = -m_p.mu/m_p.mass;
}

void Controller::initializeB()
{
    m_B.setZero(dyn::STATE_SIZE,dyn::INPUT_SIZE);
    m_B(8,0) = -1/m_p.mass;
    m_B(9,1) = 1/m_p.inertia_x;
    m_B(10,2) = 1/m_p.inertia_y;
    m_B(11,3) = 1/m_p.inertia_z;
    m_B.block(8,0,4,4) *= m_p.mixer;
}

void Controller::setX0(const dyn::xVec &x0)
{
    for (int i{0};i < x0.rows();i++)
        params.x_0[i] = x0[i];
}

void Controller::setConstRef(const dyn::xVec &ref)
{
    for (int i{0};i < ref.rows();i++)
    {
        params.x_des_0[i] = ref[i];
        params.x_des_1[i] = ref[i];
        params.x_des_2[i] = ref[i];
        params.x_des_3[i] = ref[i];
        params.x_des_4[i] = ref[i];
        params.x_des_5[i] = ref[i];
        params.x_des_6[i] = ref[i];
        params.x_des_7[i] = ref[i];
        params.x_des_8[i] = ref[i];
        params.x_des_9[i] = ref[i];
        params.x_des_10[i] = ref[i];
        params.x_des_11[i] = ref[i];
    }
}

void Controller::setA(const dyn::MatrixA &A)
{
    for (int j{0};j<A.cols();j++)
        for (int i{0};i<A.rows();i++)
            params.A[i+j*A.rows()] = A(i,j);
//    params.A[0] = 1.0;
//    params.A[1] = 0.0;
//    params.A[2] = 0.0;
//    params.A[3] = 0.0;
//    params.A[4] = 0.0;
//    params.A[5] = 0.0;
//    params.A[6] = 0.0;
//    params.A[7] = 0.0;
//    params.A[8] = 0.0;
//    params.A[9] = 0.0;
//    params.A[10] = 0.0;
//    params.A[11] = 0.0;
//    params.A[12] = 0.0;
//    params.A[13] = 1.0;
//    params.A[14] = 0.0;
//    params.A[15] = 0.0;
//    params.A[16] = 0.0;
//    params.A[17] = 0.0;
//    params.A[18] = 0.0;
//    params.A[19] = 0.0;
//    params.A[20] = 0.0;
//    params.A[21] = 0.0;
//    params.A[22] = 0.0;
//    params.A[23] = 0.0;
//    params.A[24] = 0.0;
//    params.A[25] = 0.0;
//    params.A[26] = 1.0;
//    params.A[27] = 0.0;
//    params.A[28] = 0.0;
//    params.A[29] = 0.0;
//    params.A[30] = 0.0;
//    params.A[31] = 0.0;
//    params.A[32] = 0.0;
//    params.A[33] = 0.0;
//    params.A[34] = 0.0;
//    params.A[35] = 0.0;
//    params.A[36] = 0.0;
//    params.A[37] = 0.231996974555643;
//    params.A[38] = 0.0;
//    params.A[39] = 0.095967774773050;
//    params.A[40] = 0.0;
//    params.A[41] = 0.0;
//    params.A[42] = 0.0;
//    params.A[43] = 1.133903260002749;
//    params.A[44] = 0.0;
//    params.A[45] = -0.752452412936392;
//    params.A[46] = 0.0;
//    params.A[47] = 0.0;
//    params.A[48] = -0.231381406579070;
//    params.A[49] = 0.0;
//    params.A[50] = 0.0;
//    params.A[51] = 0.0;
//    params.A[52] = 0.094806908607072;
//    params.A[53] = 0.0;
//    params.A[54] = -1.129574827519892;
//    params.A[55] = 0.0;
//    params.A[56] = 0.0;
//    params.A[57] = 0.0;
//    params.A[58] = -0.747231085281847;
//    params.A[59] = 0.0;
//    params.A[60] = 0.0;
//    params.A[61] = 0.0;
//    params.A[62] = 0.0;
//    params.A[63] = 0.0;
//    params.A[64] = 0.0;
//    params.A[65] = 1.0;
//    params.A[66] = 0.0;
//    params.A[67] = 0.0;
//    params.A[68] = 0.0;
//    params.A[69] = 0.0;
//    params.A[70] = 0.0;
//    params.A[71] = 0.0;
//    params.A[72] = 0.298504987524958;
//    params.A[73] = 0.0;
//    params.A[74] = 0.0;
//    params.A[75] = 0.0;
//    params.A[76] = 0.0;
//    params.A[77] = 0.0;
//    params.A[78] = 0.990049833749167;
//    params.A[79] = 0.0;
//    params.A[80] = 0.0;
//    params.A[81] = 0.0;
//    params.A[82] = 0.0;
//    params.A[83] = 0.0;
//    params.A[84] = 0.0;
//    params.A[85] = 0.298504987524958;
//    params.A[86] = 0.0;
//    params.A[87] = 0.0;
//    params.A[88] = 0.0;
//    params.A[89] = 0.0;
//    params.A[90] = 0.0;
//    params.A[91] = 0.990049833749167;
//    params.A[92] = 0.0;
//    params.A[93] = 0.0;
//    params.A[94] = 0.0;
//    params.A[95] = 0.0;
//    params.A[96] = 0.0;
//    params.A[97] = 0.0;
//    params.A[98] = -0.3;
//    params.A[99] = 0.0;
//    params.A[100] = 0.0;
//    params.A[101] = 0.0;
//    params.A[102] = 0.0;
//    params.A[103] = 0.0;
//    params.A[104] = 1.0;
//    params.A[105] = 0.0;
//    params.A[106] = 0.0;
//    params.A[107] = 0.0;
//    params.A[108] = 0.0;
//    params.A[109] = 0.0002459134805559943;
//    params.A[110] = 0.0;
//    params.A[111] = 0.0001031180502859246;
//    params.A[112] = 0.0;
//    params.A[113] = 0.0;
//    params.A[114] = 0.0;
//    params.A[115] = 0.001207173053843;
//    params.A[116] = 0.0;
//    params.A[117] = -0.0008085154202901194;
//    params.A[118] = 0.0;
//    params.A[119] = 0.0;
//    params.A[120] = -0.0002470958706659638;
//    params.A[121] = 0.0;
//    params.A[122] = 0.0;
//    params.A[123] = 0.0;
//    params.A[124] = 0.0001026472725536907;
//    params.A[125] = 0.0;
//    params.A[126] = -0.001211603053725;
//    params.A[127] = 0.0;
//    params.A[128] = 0.0;
//    params.A[129] = 0.0;
//    params.A[130] = -0.0008090257766910727;
//    params.A[131] = 0.0;
//    params.A[132] = 0.0;
//    params.A[133] = 0.0;
//    params.A[134] = 0.0;
//    params.A[135] = 0.0;
//    params.A[136] = 0.0;
//    params.A[137] = 0.018943050883580;
//    params.A[138] = 0.0;
//    params.A[139] = 0.0;
//    params.A[140] = 0.0;
//    params.A[141] = 0.0;
//    params.A[142] = 0.0;
    //    params.A[143] = 0.0000001324659772452711;
}

void Controller::setB(const dyn::MatrixB &B)
{
    for (int j{0};j<B.cols();j++)
        for (int i{0};i<B.rows();i++)
            params.B[i+j*B.rows()] = B(i,j);
//    params.B[0] = 0.0;
//    params.B[1] = 0.0;
//    params.B[2] = 0.015;
//    params.B[3] = 0.0;
//    params.B[4] = 0.0;
//    params.B[5] = 0.0;
//    params.B[6] = 0.0;
//    params.B[7] = 0.0;
//    params.B[8] = -0.1;
//    params.B[9] = 0.0;
//    params.B[10] = 0.0;
//    params.B[11] = 0.0;
//    params.B[12] = 0.0;
//    params.B[13] = 0.207985196849101;
//    params.B[14] = 0.0;
//    params.B[15] = 0.904032225226949;
//    params.B[16] = 0.0;
//    params.B[17] = 0.0;
//    params.B[18] = 0.0;
//    params.B[19] = 1.794430667617089;
//    params.B[20] = 0.0;
//    params.B[21] = 0.752452412936399;
//    params.B[22] = 0.0;
//    params.B[23] = 0.0;
//    params.B[24] = -0.208600764825675;
//    params.B[25] = 0.0;
//    params.B[26] = 0.0;
//    params.B[27] = 0.0;
//    params.B[28] = 0.905193091392930;
//    params.B[29] = 0.0;
//    params.B[30] = -1.798759100099951;
//    params.B[31] = 0.0;
//    params.B[32] = 0.0;
//    params.B[33] = 0.0;
//    params.B[34] = 0.747231085281837;
//    params.B[35] = 0.0;
//    params.B[36] = 0.0;
//    params.B[37] = 0.0;
//    params.B[38] = 0.0;
//    params.B[39] = 0.0;
//    params.B[40] = 0.0;
//    params.B[41] = 0.281056949116420;
//    params.B[42] = 0.0;
//    params.B[43] = 0.0;
//    params.B[44] = 0.0;
//    params.B[45] = 0.0;
//    params.B[46] = 0.0;
    //    params.B[47] = 0.999999867534023;
}

void Controller::setStateWeights(const dyn::xVec &weights,bool final)
{
    for (int i{0};i < weights.rows();i++)
    {
        params.Wy[i] = weights[i];
        if (final)
            params.Wy_final[i] = weights[i];
        else
            params.Wy_final[i] = 0.0;
    }
//    params.Wy[0] = 1.5;
//    params.Wy[1] = 1.5;
//    params.Wy[2] = 20.0;
//    params.Wy[3] = 1.0;
//    params.Wy[4] = 1.0;
//    params.Wy[5] = 5.0;
//    params.Wy[6] = 2.0;
//    params.Wy[7] = 2.0;
//    params.Wy[8] = 2.0;
//    params.Wy[9] = 50.0;
//    params.Wy[10] = 50.0;
//    params.Wy[11] = 1.0;

//    params.Wy_final[0] = 0.0;
//    params.Wy_final[1] = 0.0;
//    params.Wy_final[2] = 0.0;
//    params.Wy_final[3] = 0.0;
//    params.Wy_final[4] = 0.0;
//    params.Wy_final[5] = 0.0;
//    params.Wy_final[6] = 0.0;
//    params.Wy_final[7] = 0.0;
//    params.Wy_final[8] = 0.0;
//    params.Wy_final[9] = 0.0;
//    params.Wy_final[10] = 0.0;
//    params.Wy_final[11] = 0.0;
}

void Controller::setInputWeights(const dyn::uVec &weights)
{
    for (int i{0};i < weights.rows();i++)
        params.Wu[i] = weights[i];
//    params.Wu[0] = 1.0*0;
//    params.Wu[1] = 1.0*0;
//    params.Wu[2] = 1.0*0;
    //    params.Wu[3] = 1.0*0;
}

void Controller::setEquilibriumInputs(const dyn::uVec &u_eq)
{
    for (int i{0};i < u_eq.rows();i++)
    {
        params.u_des_0[i] = u_eq[i];
        params.u_des_1[i] = u_eq[i];
        params.u_des_2[i] = u_eq[i];
        params.u_des_3[i] = u_eq[i];
        params.u_des_4[i] = u_eq[i];
        params.u_des_5[i] = u_eq[i];
        params.u_des_6[i] = u_eq[i];
        params.u_des_7[i] = u_eq[i];
        params.u_des_8[i] = u_eq[i];
        params.u_des_9[i] = u_eq[i];
        params.u_des_10[i] = u_eq[i];
    }
//    params.u_des_0[0] = 0.55;
//    params.u_des_0[1] = 0.55;
//    params.u_des_0[2] = 0.55;
//    params.u_des_0[3] = 0.55;

//    params.u_des_1[0] = 0.0;
//    params.u_des_1[1] = 0.0;
//    params.u_des_1[2] = 0.0;
//    params.u_des_1[3] = 0.0;

//    params.u_des_2[0] = 0.0;
//    params.u_des_2[1] = 0.0;
//    params.u_des_2[2] = 0.0;
//    params.u_des_2[3] = 0.0;

//    params.u_des_3[0] = 0.0;
//    params.u_des_3[1] = 0.0;
//    params.u_des_3[2] = 0.0;
//    params.u_des_3[3] = 0.0;

//    params.u_des_4[0] = 0.0;
//    params.u_des_4[1] = 0.0;
//    params.u_des_4[2] = 0.0;
//    params.u_des_4[3] = 0.0;

//    params.u_des_5[0] = 0.0;
//    params.u_des_5[1] = 0.0;
//    params.u_des_5[2] = 0.0;
//    params.u_des_5[3] = 0.0;

//    params.u_des_6[0] = 0.0;
//    params.u_des_6[1] = 0.0;
//    params.u_des_6[2] = 0.0;
//    params.u_des_6[3] = 0.0;

//    params.u_des_7[0] = 0.0;
//    params.u_des_7[1] = 0.0;
//    params.u_des_7[2] = 0.0;
//    params.u_des_7[3] = 0.0;

//    params.u_des_8[0] = 0.0;
//    params.u_des_8[1] = 0.0;
//    params.u_des_8[2] = 0.0;
//    params.u_des_8[3] = 0.0;

//    params.u_des_9[0] = 0.0;
//    params.u_des_9[1] = 0.0;
//    params.u_des_9[2] = 0.0;
//    params.u_des_9[3] = 0.0;

//    params.u_des_10[0] = 0.0;
//    params.u_des_10[1] = 0.0;
//    params.u_des_10[2] = 0.0;
    //    params.u_des_10[3] = 0.0;
}

void Controller::setInputLimits(double min, double max)
{
    params.u_min[0] = min;
    params.u_max[0] = max;
//    params.u_min[0] = 0.0;
    //    params.u_max[0] = 1.0;
}

void Controller::setSlewRate(double slew_rate)
{
    params.S[0] = slew_rate;
//    params.S[0] = 0.005;
}

void Controller::linearizeAboutCurrentAttitude()
{
    this->updateRotation();
    this->updateA();
    this->discretizeAB();
    this->setA(m_Ad);
    this->setB(m_Bd);
}

void Controller::updateRotation()
{
    double roll{m_x(dyn::RX)};
    double pitch{m_x(dyn::RY)};
    double yaw{m_x(dyn::RZ)};
    m_R_b2i(0,0) = cos(pitch)*cos(yaw);
    m_R_b2i(0,1) = sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw);
    m_R_b2i(0,2) = cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw);
    m_R_b2i(1,0) = cos(pitch)*sin(yaw);
    m_R_b2i(1,1) = sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw);
    m_R_b2i(1,2) = cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw);
    m_R_b2i(2,0) = -sin(pitch);
    m_R_b2i(2,1) = sin(roll)*cos(pitch);
    m_R_b2i(2,2) = cos(roll)*cos(pitch);
}

void Controller::updateA()
{
    Eigen::Matrix3d down_to_height;
    down_to_height << 1,0,0, 0,1,0, 0,0,-1;
    m_A.block(dyn::PX,dyn::VX,3,3) = down_to_height*m_R_b2i;
    m_A.block(dyn::VX,dyn::RX,3,3) << 0,-m_p.grav*cos(m_x(dyn::RY)),0,
                                      m_p.grav*cos(m_x(dyn::RY))*cos(m_x(dyn::RX)),-m_p.grav*sin(m_x(dyn::RY))*sin(m_x(dyn::RX)),0,
                                     -m_p.grav*cos(m_x(dyn::RY))*sin(m_x(dyn::RX)),-m_p.grav*sin(m_x(dyn::RY))*cos(m_x(dyn::RX)),0;
}

void Controller::discretizeAB()
{
    m_Ad = (m_A*m_rate).exp();
    m_Bd = m_rate*(Eigen::MatrixXd::Identity(dyn::STATE_SIZE,dyn::STATE_SIZE)+m_A*m_rate/2+m_A*m_A*m_rate*m_rate/6+m_A*m_A*m_A*m_rate*m_rate*m_rate/24)*m_B;
}

void Controller::load_data()
{
    dyn::xVec x0;
    setX0(x0.setZero(dyn::STATE_SIZE,1));
    dyn::xVec ref;
    setConstRef(ref.setZero(dyn::STATE_SIZE,1));
    dyn::MatrixA A;
    int count{0};


    for (int i{0};i<A.cols();i++)
        for (int j{0};j<A.rows();j++)
        {
            A(j,i) = count;
            count++;
        }


    setA(A);
    setB(m_B);
    linearizeAboutCurrentAttitude();

    dyn::xVec initial_state_weights;
    initial_state_weights << 1.5,1.5,20,1,1,5,2,2,2,50,50,1;
    setStateWeights(initial_state_weights);

    dyn::uVec initial_input_weights;
    initial_input_weights.setZero(dyn::INPUT_SIZE,1);
    setInputWeights(initial_input_weights);

    setEquilibriumInputs(m_p.u_eq);

    double min_motor_signal{0};
    double max_motor_signal{1};
    setInputLimits(min_motor_signal,max_motor_signal);

    double initial_slew_rate{0.005};
    setSlewRate(initial_slew_rate);
}
