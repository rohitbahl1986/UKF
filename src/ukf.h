#ifndef UKF_H
#define UKF_H

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "measurement_package.h"
#include "tools.h"

#include "Eigen/Dense"
#include <stdint.h>
#include <vector>
#include <string>
#include "kalmanFilter.hpp"

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class UKF: public KalmanFilter
{
public:

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    virtual std::tuple<Eigen::VectorXd, double> init(const MeasurementPackage &meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    virtual void predict(double dt);

    virtual std::tuple<Eigen::VectorXd, double> update(const MeasurementPackage &meas_package);

    double getNisRadar()
    {
        return m_nisRadar;
    }

    double getNisLidar()
    {
        return m_nisRadar;
    }
private:

    // State dimension
    int32_t m_nState;

    // Augmented state dimension
    int32_t m_nStateAug;

    // Sigma point spreading parameter
    double m_lambda;

    int32_t m_numSigmaPoints;

    // if this is false, laser measurements will be ignored (except for init)
    bool m_useLidar;

    // if this is false, radar measurements will be ignored (except for init)
    bool m_useRadar;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd m_stateMean;

    // state covariance matrix
    MatrixXd m_stateCovar;

    // predicted sigma points matrix
    MatrixXd m_sigPred;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    double m_stda;

    // Process noise standard deviation yaw acceleration in rad/s^2
    double m_stdYawdd;

    // Laser measurement noise standard deviation position1 in m
    double m_stdLaspx;

    // Laser measurement noise standard deviation position2 in m
    double m_stdLaspy;

    // Radar measurement noise standard deviation radius in m
    double m_stdRadr;

    // Radar measurement noise standard deviation angle in rad
    double m_stdRadphi;

    // Radar measurement noise standard deviation radius change in m/s
    double m_stdRadrd;

    uint32_t m_radarDim;

    uint32_t m_lidarDim;

    VectorXd m_weights;

    double m_nisRadar, m_nisLidar;

    // Lidar measurement noise covariance matrix
    MatrixXd m_measNoiseCovarLidar;

    // Radar measurement noise covariance matrix
    MatrixXd m_measNoiseCovarRadar;

    void initWithLaserData(const MeasurementPackage &meas_package);
    void initWithRadarData(const MeasurementPackage &meas_package);
    void generateSigmaPoints(MatrixXd &Xsig_gen_);
    void predictSigmaPoints(double dt, const MatrixXd &Xsig_gen_);
    void predictMeanNCovar();
    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);

    void update(MatrixXd &S, VectorXd &z_pred, MatrixXd &Zsig, MatrixXd &Tc, VectorXd &z);
    void normalizeAngle(double &angle);
    double calcNis(const MatrixXd &S, const VectorXd &z_pred, const VectorXd &z) const;

};

#endif /* UKF_H */
