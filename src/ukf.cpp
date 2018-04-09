/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**************************************************************************************************
 *  CLASS FUNCTIONS
 *************************************************************************************************/

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
UKF::UKF() :
                m_nState(5), m_nStateAug(7), m_lambda(3 - m_nStateAug),
                m_numSigmaPoints((m_nStateAug << 1) + 1), m_useLidar(true), m_useRadar(true),
                m_stateMean(VectorXd(m_nState)), m_stateCovar(MatrixXd::Identity(m_nState, m_nState)),
                m_sigPred(MatrixXd(m_nState, m_numSigmaPoints)), m_stda(3), m_stdYawdd(1),
                m_stdLaspx(0.15), m_stdLaspy(0.15), m_lidarDim(2), m_stdRadr(0.3), m_stdRadphi(0.03),
                m_stdRadrd(0.3), m_radarDim(3), m_nisRadar(0), m_nisLidar(0)
{
    m_sigPred.fill(0);

    // set weights
    m_weights = VectorXd(m_numSigmaPoints);
    m_weights(0) = m_lambda / (m_lambda + m_nStateAug);
    for (int i = 1; i < m_numSigmaPoints; ++i)
    {
        m_weights(i) = 0.5 / (m_nStateAug + m_lambda);
    }

    m_measNoiseCovarLidar = MatrixXd(m_lidarDim, m_lidarDim);
    m_measNoiseCovarLidar << m_stdLaspx * m_stdLaspx, 0, 0, m_stdLaspy * m_stdLaspy;

    m_measNoiseCovarRadar = MatrixXd(m_radarDim, m_radarDim);

    m_measNoiseCovarRadar << m_stdRadr * m_stdRadr, 0, 0,
                             0, m_stdRadphi * m_stdRadphi, 0,
                             0, 0, m_stdRadrd * m_stdRadrd;
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
UKF::~UKF()
{
}

/**************************************************************************************************
 *  @brief Initialize the UKF
 *************************************************************************************************/
std::tuple<Eigen::VectorXd, double> UKF::init(const MeasurementPackage &meas_package)
{
    double nis = 0;

    if (MeasurementPackage::SensorType::LASER == meas_package.sensor_type_)
    {
        initWithLaserData(meas_package);
    }
    else if (MeasurementPackage::SensorType::RADAR == meas_package.sensor_type_)
    {
        initWithRadarData(meas_package);
    }

    setFilterState(ACTIVE);

    return std::make_tuple(m_stateMean, nis);
}

/**************************************************************************************************
 *  @brief Implement the pipeline for prediction mean and covariance of the state vector
 *************************************************************************************************/
void UKF::predict(double dt)
{
    MatrixXd Xsig_gen_ = MatrixXd(m_nStateAug, m_numSigmaPoints);
    Xsig_gen_.fill(0);
    generateSigmaPoints(Xsig_gen_);
    predictSigmaPoints(dt, Xsig_gen_);
    predictMeanNCovar();
}

/**************************************************************************************************
 *  @brief Update the state using the measurement received at K+1
 *************************************************************************************************/
std::tuple<Eigen::VectorXd, double> UKF::update(const MeasurementPackage &meas_package)
{
    double nis = 0;

    if (MeasurementPackage::SensorType::LASER == meas_package.sensor_type_ && m_useLidar)
    {
        UpdateLidar(meas_package);
        nis = m_nisLidar;
    }
    else if (MeasurementPackage::SensorType::RADAR == meas_package.sensor_type_ && m_useRadar)
    {
        UpdateRadar(meas_package);
        nis = m_nisRadar;
    }

    return std::make_tuple(m_stateMean, nis);
}

/**************************************************************************************************
 *  @brief Generate the sigma points for the current distribution.
 *************************************************************************************************/
void UKF::generateSigmaPoints(MatrixXd &Xsig_gen_)
{
    //create augmented mean vector
    VectorXd x_aug = VectorXd(m_nStateAug);
    x_aug.head(5) = m_stateMean;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(m_nStateAug, m_nStateAug);
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5) = m_stateCovar;
    P_aug(5, 5) = m_stda * m_stda;
    P_aug(6, 6) = m_stdYawdd * m_stdYawdd;

    //set first column of sigma point matrix
    Xsig_gen_.col(0) = x_aug;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //set remaining sigma points
    for (int i = 0; i < m_nStateAug; ++i)
    {
        Xsig_gen_.col(i + 1) = x_aug + sqrt(m_lambda + m_nStateAug) * L.col(i);
        Xsig_gen_.col(i + 1 + m_nStateAug) = x_aug - sqrt(m_lambda + m_nStateAug) * L.col(i);
    }
}

/**************************************************************************************************
 *  @brief Predict the sigma points of the new distribution.
 *************************************************************************************************/
void UKF::predictSigmaPoints(double dt, const MatrixXd &Xsig_gen_)
{
    //predict sigma points
    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        //extract values for better readability
        const double p_x = Xsig_gen_(0, i);
        const double p_y = Xsig_gen_(1, i);
        const double v = Xsig_gen_(2, i);
        const double yaw = Xsig_gen_(3, i);
        const double yawd = Xsig_gen_(4, i);
        const double nu_a = Xsig_gen_(5, i);
        const double nu_yawdd = Xsig_gen_(6, i);

        //predicted state values
        double px_p, py_p;

        //avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
        }
        else
        {
            px_p = p_x + v * dt * cos(yaw);
            py_p = p_y + v * dt * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * dt;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * dt * dt * cos(yaw);
        py_p = py_p + 0.5 * nu_a * dt * dt * sin(yaw);
        v_p = v_p + nu_a * dt;

        yaw_p = yaw_p + 0.5 * nu_yawdd * dt * dt;
        yawd_p = yawd_p + nu_yawdd * dt;

        //write predicted sigma point into right column
        m_sigPred(0, i) = px_p;

        m_sigPred(1, i) = py_p;

        m_sigPred(2, i) = v_p;
        m_sigPred(3, i) = yaw_p;
        m_sigPred(4, i) = yawd_p;
    }
}

/**************************************************************************************************
 *  @brief Predict the mean and covariance at time K+1
 *************************************************************************************************/
void UKF::predictMeanNCovar()
{
    //predicted state mean
    m_stateMean.fill(0.0);
    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        m_stateMean = m_sigPred * m_weights;
    }

    //predicted state covariance matrix
    m_stateCovar.fill(0.0);
    for (int i = 0; i < m_numSigmaPoints; i++)
    {
        // state difference
        VectorXd x_diff = m_sigPred.col(i) - m_stateMean;
        normalizeAngle(x_diff(3));

        m_stateCovar = m_stateCovar + m_weights(i) * x_diff * x_diff.transpose();
    }
}

/**************************************************************************************************
 *  @brief Updates the state and the state covariance matrix using a lidar measurement.
 *************************************************************************************************/
void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(m_lidarDim, m_numSigmaPoints);
    Zsig.fill(0);

    //transform sigma points into measurement space
    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        Zsig(0, i) = m_sigPred(0, i);
        Zsig(1, i) = m_sigPred(1, i);
    }

    VectorXd z_pred = VectorXd(m_lidarDim);
    z_pred.fill(0);

    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        z_pred = z_pred + m_weights(i) * Zsig.col(i);
    }

    //innovation covariance matrix S
    MatrixXd S = MatrixXd(m_lidarDim, m_lidarDim);
    S.fill(0.0);

    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        S = S + m_weights(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(m_lidarDim, m_lidarDim);
    R << m_stdLaspx * m_stdLaspx, 0, 0, m_stdLaspy * m_stdLaspy;
    S = S + m_measNoiseCovarLidar;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(m_nState, m_lidarDim);
    //calculate cross correlation matrix
    Tc.fill(0.0);

    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // state difference
        VectorXd x_diff = m_sigPred.col(i) - m_stateMean;
        Tc = Tc + m_weights(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    VectorXd z = VectorXd(m_lidarDim);
    z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1);

    //residual
    VectorXd z_diff = z - z_pred;

    //update state mean and covariance matrix
    m_stateMean = m_stateMean + K * z_diff;
    m_stateCovar = m_stateCovar - K * S * K.transpose();

    m_nisLidar = calcNis(S, z_pred, z);
}

/**************************************************************************************************
 *  @brief Updates the state and the state covariance matrix using a radar measurement.
 *************************************************************************************************/
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    //innovation covariance matrix S
    MatrixXd S = MatrixXd(m_radarDim, m_radarDim);
    S.fill(0.0);
    VectorXd z_pred = VectorXd(m_radarDim);
    z_pred.fill(0);
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(m_radarDim, m_numSigmaPoints);
    Zsig.fill(0);

    //transform sigma points into measurement space
    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        const double p_x = m_sigPred(0, i);
        const double p_y = m_sigPred(1, i);
        const double v = m_sigPred(2, i);
        const double yaw = m_sigPred(3, i);

        const double v1 = cos(yaw) * v;
        const double v2 = sin(yaw) * v;

        // measurement model
        Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                        //r
        Zsig(1, i) = atan2(p_y, p_x);                                 //phi
        Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);   //r_dot
    }

    for (int i = 0; i < m_numSigmaPoints; i++)
    {
        z_pred = z_pred + m_weights(i) * Zsig.col(i);
    }
    normalizeAngle(z_pred(1));

    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization

        normalizeAngle(z_diff(1));
        S = S + m_weights(i) * z_diff * z_diff.transpose();
    }

    S = S + m_measNoiseCovarRadar;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(m_nState, m_radarDim);
    //calculate cross correlation matrix
    Tc.fill(0.0);

    VectorXd z = VectorXd(m_radarDim);
    z << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), meas_package
            .raw_measurements_(2);

    update(S, z_pred, Zsig, Tc, z);

    m_nisRadar = calcNis(S, z_pred, z);
}

/**************************************************************************************************
 *  @brief BHelper function to update the state
 *************************************************************************************************/
void UKF::update(MatrixXd &S, VectorXd &z_pred, MatrixXd &Zsig, MatrixXd &Tc, VectorXd &z)
{
    for (int i = 0; i < m_numSigmaPoints; ++i)
    {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        normalizeAngle(z_diff(1));

        // state difference
        VectorXd x_diff = m_sigPred.col(i) - m_stateMean;
        normalizeAngle(x_diff(3));

        Tc = Tc + m_weights(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    normalizeAngle(z_diff(1));

    //update state mean and covariance matrix
    m_stateMean = m_stateMean + K * z_diff;
    m_stateCovar = m_stateCovar - K * S * K.transpose();
}

/**************************************************************************************************
 *  @brief Initialize the state vector with Radar data.
 *************************************************************************************************/
void UKF::initWithRadarData(const MeasurementPackage &measurement_pack)
{
    // Convert radar from polar to Cartesian coordinates and initialize state.
    double rho = measurement_pack.raw_measurements_[0];
    double phi = measurement_pack.raw_measurements_[1];

    // X-co-ordinate is the projection of the radial distance on the X axis
    double position_x = rho * cos(phi);

    //Y-co-ordinate is the projection of the radial distance on the Y axis
    double position_y = rho * sin(phi);

    m_stateMean << position_x, position_y, 0, 0, 0;

    cout << "Init done with Radar data" << endl;
}

/**************************************************************************************************
 *  @brief Initialize the state vector with Laser data.
 *************************************************************************************************/
void UKF::initWithLaserData(const MeasurementPackage &measurement_pack)
{
    // Initialize state for Laser data.
    m_stateMean << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    cout << "Init done with Laser data" << endl;
}

/**************************************************************************************************
 *  @brief Bring angle in range of pi to -pi
 *************************************************************************************************/
void UKF::normalizeAngle(double &angle)
{
    while (angle > M_PI || angle < -M_PI)
    {
        double delta = (angle > M_PI) ? -M_PI : M_PI;
        angle += delta;
    }
}

/**************************************************************************************************
 *  @brief Calculate NIS
 *************************************************************************************************/
double UKF::calcNis(const MatrixXd &S, const VectorXd &z_pred, const VectorXd &z) const
{
    double nis;

    nis = (z - z_pred).transpose() * S.inverse() * (z - z_pred);

    return nis;
}
