/*
 * kalmanFilter.cpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "kalmanFilter.hpp"

#include <iostream>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;
using namespace Eigen;

/**************************************************************************************************
 *  CLASS FUNCTIONS
 *************************************************************************************************/

/**************************************************************************************************
 *  Constructor
 *************************************************************************************************/
KalmanFilter::KalmanFilter()
{
    m_lastUpdateTime = 0;
    m_filterState = INIT;
}

/**************************************************************************************************
 *  @brief Invoke the processing for the filter.
 *************************************************************************************************/
std::tuple<Eigen::VectorXd, double> KalmanFilter::processData(
        const MeasurementPackage &meas_package)
{
    std::tuple<Eigen::VectorXd, double> tp;

    if (INIT == m_filterState)
    {
        cout << "First Measurement" << endl;
        tp = init(meas_package);
        m_lastUpdateTime = meas_package.timestamp_;
        m_filterState = ACTIVE;
    }
    else
    {
        //compute the time elapsed between the current and previous measurements
        double dt = (meas_package.timestamp_ - m_lastUpdateTime) / 1000000.0;
        m_lastUpdateTime = meas_package.timestamp_;

        predict(dt);

        tp = update(meas_package);
    }

    return tp;
}

