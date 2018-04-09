/*
 * processData.hpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohitbahl
 */

#if !defined(PROCESS_DATA_HPP_)
#define PROCESS_DATA_HPP_

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "algoIf.hpp"
#include "measurement_package.h"
#include "tools.h"
#include "ukf.h"

#include <fstream>
#include <memory>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class ProcessData
{
public:
    ProcessData();

    ~ProcessData();

    /**
     *  @brief Process the received measurement and predict the position.
     */
    VectorXd estimatePosition(const MeasurementPackage &meas_package);

private:
    std::fstream fLidar;
    std::fstream fRadar;

    const Tools m_tools;
    std::unique_ptr<AlgoIf> algorithm;
};

#endif /// PROCESS_DATA_HPP_
