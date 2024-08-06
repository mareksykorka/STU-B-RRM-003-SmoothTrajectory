#ifndef CATKIN_TRETIE_ZADANIE_TRAJECTORY_PLAN_H
#define CATKIN_TRETIE_ZADANIE_TRAJECTORY_PLAN_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include "zad3/custom_math/polynomial_tools.h"
#include "zad3/custom_math/inverse_matrix_solver.h"

class TrajectoryPlan
{
    public:
        TrajectoryPlan(std::vector<int> conditionTime, std::vector<double> vector, MatrixSolver* Matrix);
        void createPolynomials(int maxDerivation);
        bool isInTime(double time);
        void solve();
        std::vector<Polynomial> polynomials_;
    private:
        std::vector<int> conditionTime_;
        Eigen::VectorXd conditions_;
        MatrixSolver* matrix_;
        Eigen::VectorXd solution_;
};

#endif //CATKIN_TRETIE_ZADANIE_TRAJECTORY_PLAN_H
