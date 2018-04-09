/*
 * algoIf.hpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

#if !defined(ALGO_IF_HPP_)
#define ALGO_IF_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "Eigen/Dense"
#include "measurement_package.h"

#include <tuple>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
/**
 * @brief This class provides abstraction for state prediction algorithms.
 * Every state prediction algorithm must implement this class.
 */
class AlgoIf
{
public:
    AlgoIf()
    {
    };

    virtual ~AlgoIf()
    {
    };

    /**
     * @breif Invoke the processing for the filter.
     * @param[in] Measurement package
     * @return Return a tuple consisting of the estimated position and the NIS value of the filter.
     */
    virtual std::tuple<Eigen::VectorXd, double> processData(
            const MeasurementPackage &meas_package) = 0;
};

#endif // ALGO_IF_HPP_
