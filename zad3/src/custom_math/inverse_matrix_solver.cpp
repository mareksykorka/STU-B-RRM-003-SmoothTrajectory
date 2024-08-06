#include "custom_math/inverse_matrix_solver.h"

MatrixSolver::MatrixSolver(int size)
{
    matrix_ = Eigen::MatrixXd::Zero(size,size);

    for(int i = 0; i<2; i++)
        for(int j = 0; j<(size/2); j++)
            basePolynomials_.push_back(Polynomial(j, size));
}

MatrixSolver::MatrixSolver(std::vector<int> conditionTime)
{
    matrix_ = Eigen::MatrixXd::Zero(conditionTime.size(),conditionTime.size());

    int derivation = 0, lastTime = -1;
    for (int i = 0; i < conditionTime.size(); i++)
    {
        (lastTime == conditionTime[i])?(derivation+=1):(derivation = 0);
        basePolynomials_.push_back(Polynomial(derivation, conditionTime.size()));
        lastTime = conditionTime[i];
    }
}

Eigen::VectorXd MatrixSolver::solve(std::vector<int> conditionTime, Eigen::VectorXd conditions)
{
    Eigen::VectorXd solution = Eigen::VectorXd::Zero(conditions.size());

    for (int i = 0; i < basePolynomials_.size(); i++)
        for (int j = 0; j < basePolynomials_[i].size(); j++)
            matrix_(i,j) = basePolynomials_[i].value(j,conditionTime[i]);

    if (matrix_.determinant() != 0)
        solution = matrix_.inverse() * conditions;  // Equivalent for matlab a = A\q
    else
        std::cerr << "Matrix " << matrix_ << " is not invertible." << std::endl;

    return solution;
}