#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <fstream>

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class Tools
{
public:
    /**
     * Constructor.
     */
    Tools();

    /**
     * Destructor.
     */
    virtual ~Tools();

    /**
     * @brief A helper method to calculate RMSE.
     */
    VectorXd CalculateRMSE(
            const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) const;
};

#endif /* TOOLS_H_ */
