#include <iostream>
#include "tools.h"

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
Tools::Tools()
{
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
Tools::~Tools()
{
}

/**************************************************************************************************
 *  @brief A helper method to calculate RMSE.
 *************************************************************************************************/
VectorXd Tools::CalculateRMSE(
        const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) const
{
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (uint32_t i = 0; i < estimations.size(); ++i)
    {
        VectorXd residual = (estimations[i] - ground_truth[i]);
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}
