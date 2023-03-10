#include "ukf.h"

#include <iostream>

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_ = 0;

    // time when the state is true, in us
    time_us_ = 0;

    // PLEASE Check src/simpleukf_tools.h for noise constants.
    // The main logic is encapsulated in the submodule src/simpleukf [https://github.com/andrea-nisti/simple-ukf]
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    // Initialize the filter with a measure
    if (not is_initialized_)
    {

        return;
    }

    const double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;

    // check if last fusion timestamp is recent or old
    if (delta_t > 0.0f)
    {
        Prediction(delta_t);
    }

    // fuse correct measurement
    bool measure_fused{true};
    if ((meas_package.sensor_type_ == meas_package.LASER) and use_laser_)
    {
        UpdateLidar(meas_package);
    }
    else if ((meas_package.sensor_type_ == meas_package.RADAR) and use_radar_)
    {
        UpdateRadar(meas_package);
    }
    else
    {
        // inverse logic to include prediction when laser and radar are not used
        measure_fused = false;
        std::cout << "Received wrong sensor type, skipping package" << std::endl;
    }

    // update last fusion timestamp
    if (measure_fused)
    {
        time_us_ = meas_package.timestamp_;
    }
}

void UKF::Prediction(double delta_t)
{
    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use lidar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use radar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */
}