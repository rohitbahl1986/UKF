/*
 * kalmanFilter.hpp
 *
 *  Created on: Apr 8, 2018
 *      Author: rohit bahl
 */

#if !defined(KALMAN_FILTER_HPP_)
#define KALMAN_FILTER_HPP_

#include "algoIf.hpp"
#include "measurement_package.h"

/**
 * @brief This is the base class for all Kalman Filter algorithms.
 */
class KalmanFilter: public AlgoIf
{
public:

    enum FilterState
    {
        INIT,
        ACTIVE
    };

    KalmanFilter();

    virtual ~KalmanFilter()
    {
    };

    std::tuple<Eigen::VectorXd, double> processData(const MeasurementPackage &meas_package) final;

    void setFilterState(FilterState state)
    {
        m_filterState = state;
    }

    FilterState getFilterState() const
    {
        return m_filterState;
    }

    void setPositionEstimate(Eigen::VectorXd positionEstimate)
    {
        m_positionEstimate = positionEstimate;
    }

    Eigen::VectorXd getPositionEstimate() const
    {
        return m_positionEstimate;
    }

private:

    // time when the state is true, in us
    long long m_lastUpdateTime;

    // initially set to false, set to true in first call of ProcessMeasurement
    FilterState m_filterState;

    Eigen::VectorXd m_positionEstimate;

    /**
     * @brief Interface for initializing kalam filter.
     */
    virtual std::tuple<Eigen::VectorXd, double> init(const MeasurementPackage &meas_package) = 0;

    /**
     * @brief Interface for the prediction stage of kalman filter.
     */
    virtual void predict(double dt) = 0;

    /**
     * @brief Interface for the update stage of the kalman filter.
     */
    virtual std::tuple<Eigen::VectorXd, double> update(const MeasurementPackage &meas_package) = 0;
};

#endif // KALMAN_FILTER_HPP_
