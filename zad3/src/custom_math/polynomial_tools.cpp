#include "custom_math/polynomial_tools.h"

Polynomial::Polynomial(int derivation, int degree)
{
    for(int baseExp = 0; baseExp < degree; baseExp++)
    {
        exp_.push_back(((baseExp - derivation) < 0) ? (0) : (baseExp - derivation));

        int multiplier = 1;
        for (int j = 0; j < derivation; j++)
            multiplier *= baseExp - j;

        coeff_.push_back((baseExp < derivation) ? (0) : (multiplier));
    }
}

Polynomial::Polynomial(int derivation, Eigen::VectorXd baseCoeff)
{
    for(int baseExp = 0; baseExp < baseCoeff.size(); baseExp++)
    {
        exp_.push_back(((baseExp - derivation) < 0) ? (0) : (baseExp - derivation));

        int multiplier = 1;
        for (int j = 0; j < derivation; j++)
            multiplier *= baseExp - j;

        coeff_.push_back((baseExp < derivation) ? (0) : (baseCoeff[baseExp] * multiplier));
    }
}

double Polynomial::calc(double time) const
{
    double result = 0;
    for(int i = 0; i < coeff_.size(); i++)
        result += value(i,time); // a_0 + a_1 + ... + a_i
    return result;
}

double Polynomial::value(int index, double time) const
{
    return coeff_[index]*(pow(time,exp_[index])); // coef_x * t^exp_x
}

int Polynomial::size() const
{
    return coeff_.size();
}