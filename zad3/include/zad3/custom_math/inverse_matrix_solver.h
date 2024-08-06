#ifndef CATKIN_TRETIE_ZADANIE_INVERSE_MATRIX_SOLVER_H
#define CATKIN_TRETIE_ZADANIE_INVERSE_MATRIX_SOLVER_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include "custom_math/polynomial_tools.h"

class MatrixSolver
{
    public:
        MatrixSolver() = delete;
        MatrixSolver(int size);
        MatrixSolver(std::vector<int> conditionTime);
        Eigen::VectorXd solve(std::vector<int> conditionTime, Eigen::VectorXd conditions);
    private:
        Eigen::MatrixXd matrix_;
        std::vector<Polynomial> basePolynomials_;
};

#endif //CATKIN_TRETIE_ZADANIE_INVERSE_MATRIX_SOLVER_H