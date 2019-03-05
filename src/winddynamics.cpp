#include "dynamics/winddynamics.hpp"
#include "math.h"
#include <random>

namespace dyn
{

WindDynamics::WindDynamics() :
    m_Lu{200},
    m_Lv{m_Lu},
    m_Lw{50},
    m_sigma_u{2.12},
    m_sigma_v{m_sigma_u},
    m_sigma_w{1.4},
    m_dt{0.05}
{
    m_A << -1,0,0,0,0, 0,-1,-1,0,0, 0,1,0,0,0, 0,0,0,-1,-1, 0,0,0,1,0;
    m_B << 1,0,0, 0,1,0, 0,0,0, 0,0,1, 0,0,0;
    m_C << 1,0,0,0,0, 0,1,1,0,0, 0,0,0,1,1;
    m_x.setZero();
}

Eigen::Vector3d WindDynamics::update(const double Va)
{
    this->updateAandC(Va);
    Eigen::Vector3d u;
    u << getRandGaussian(), getRandGaussian(), getRandGaussian();
    m_x += m_dt*(m_A*m_x + m_B*u);
    return m_C*m_x;
}

void WindDynamics::updateAandC(const double Va)
{
    m_A(0,0) = -Va/m_Lu;
    m_A(1,1) = -2*Va/m_Lv;
    m_A(1,2) = -Va*Va/(m_Lv*m_Lv);
    m_A(3,3) = -2*Va/m_Lw;
    m_A(3,4) = -Va*Va/(m_Lw*m_Lw);

    m_C(0,0) = m_sigma_u*std::sqrt(2*Va/m_Lu);
    m_C(1,1) = m_sigma_v*std::sqrt(3*Va/m_Lv);
    m_C(1,2) = std::sqrt(Va*Va*Va/(m_Lv*m_Lv*m_Lv));
    m_C(2,3) = m_sigma_w*std::sqrt(3*Va/m_Lv);
    m_C(2,4) = std::sqrt(Va*Va*Va/(m_Lw*m_Lw*m_Lw));
}

double WindDynamics::getRandGaussian()
{
    std::random_device rd;
    std::mt19937_64 gen{rd()};
    double mean{0};
    double std_dev{1.0};
    std::normal_distribution<> dis{mean, std_dev};
    return dis(gen);
}

} // end namespace dyn
