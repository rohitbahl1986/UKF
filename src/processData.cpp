/*
 * processData.cpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "processData.hpp"
#include "ukf.h"

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
ProcessData::ProcessData() :
        algorithm(new UKF)
{
    fLidar.open("../lidarNis.txt", ios::out);
    fRadar.open("../radarNis.txt", ios::out);
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
ProcessData::~ProcessData()
{
    fLidar.close();
    fRadar.close();
}

/**************************************************************************************************
 *  @brief Process the received measurement and predict the position.
 *************************************************************************************************/
VectorXd ProcessData::estimatePosition(const MeasurementPackage &meas_package)
{
    auto tuple = algorithm->processData(meas_package);

    if (MeasurementPackage::SensorType::LASER == meas_package.sensor_type_)
    {
        fLidar << std::get<1>(tuple) << endl;
    }
    else if (MeasurementPackage::SensorType::RADAR == meas_package.sensor_type_)
    {
        fRadar << std::get<1>(tuple) << endl;
    }
    return std::get<0>(tuple);
}

