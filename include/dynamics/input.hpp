#ifndef INPUT_HPP
#define INPUT_HPP

#include <Eigen/Core>

namespace fixedwing
{

struct Input
{
    enum
    {
        SIZE=4
    };

    Eigen::Matrix<double,SIZE,1> vec;
    double& de;
    double& dt;
    double& da;
    double& dr;

    Input():
        de{*vec.data()},
        dt{*(vec.data()+1)},
        da{*(vec.data()+2)},
        dr{*(vec.data()+3)}
    {
        vec.setZero();
    }

    Input& operator =(const Input& d)
    {
        vec = d.vec;
        return *this;
    }

    void operator +=(const Input& d)
    {
        vec += d.vec;
    }
};

} //end namespace fixedwing

namespace quadcopter
{

struct Input
{
    enum
    {
        SIZE = 4
    };

    Eigen::Matrix<double,SIZE,1> vec;
    double& T;
    Eigen::Map<Eigen::Vector3d> tau;
    Eigen::Map<Eigen::Vector3d> del;

    Input():
        T{*vec.data()},
        tau{vec.data()+1},
        del{vec.data()+1}
    {
        vec.setZero();
    }
};

}

#endif // INPUT_HPP
