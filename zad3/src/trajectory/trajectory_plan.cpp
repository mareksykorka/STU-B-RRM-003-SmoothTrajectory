#include "trajectory/trajectory_plan.h"

TrajectoryPlan::TrajectoryPlan(std::vector<int> conditionTime, std::vector<double> vector, MatrixSolver* Matrix)
{
    conditionTime_ = conditionTime;

    conditions_ = Eigen::VectorXd::Zero(vector.size());;
    for (int i = 0; i < vector.size(); i++)
        conditions_[i] = vector[i];

    matrix_ = Matrix;
    solution_ = Eigen::VectorXd::Zero(conditionTime.size());
}

void TrajectoryPlan::createPolynomials(int maxDerivation)
{
    for (int i = 0; i <= maxDerivation; i++)
        polynomials_.push_back(Polynomial(i,solution_));
}

bool TrajectoryPlan::isInTime(double time)
{
    return ((conditionTime_[0] <= time) && ( time <= conditionTime_[conditionTime_.size()-1]));
}

void TrajectoryPlan::solve()
{
    solution_ = matrix_->solve(conditionTime_,conditions_);
}