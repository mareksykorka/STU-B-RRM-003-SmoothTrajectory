#ifndef CATKIN_TRETIE_ZADANIE_POLYNOMIAL_TOOLS_H
#define CATKIN_TRETIE_ZADANIE_POLYNOMIAL_TOOLS_H

#include <ros/ros.h>
#include <Eigen/Geometry>

class Polynomial
{
    public:
        Polynomial() = delete;
        Polynomial(int derivation, int degree);
        Polynomial(int derivation, Eigen::VectorXd baseCoeff);
        double calc(double time) const;
        double value(int index, double time) const;
        int size() const;
    private:
        std::vector<double> coeff_;
        std::vector<int> exp_;
};

#endif //CATKIN_TRETIE_ZADANIE_POLYNOMIAL_TOOLS_H