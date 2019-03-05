#ifndef WINDDYNAMICS_HPP
#define WINDDYNAMICS_HPP

#include <Eigen/Core>

namespace dyn
{

class WindDynamics
{
public:
    WindDynamics();
    Eigen::Vector3d update(const double Va);

protected:
    void updateAandC(const double Va);
    double getRandGaussian();

private:
    double m_Lu;
    double m_Lv;
    double m_Lw;
    double m_sigma_u;
    double m_sigma_v;
    double m_sigma_w;
    double m_dt;
    Eigen::Matrix<double,5,5> m_A;
    Eigen::Matrix<double,5,3> m_B;
    Eigen::Matrix<double,3,5> m_C;
    Eigen::Matrix<double,5,1> m_x;
};

} // end namespace dyn

#endif // WINDDYNAMICS_HPP
