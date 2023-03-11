#include "ukf.h"

#include <iostream>

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
namespace fusion
{

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
    bool is_initialized_ = false;

    // time when the state is true, in us
    time_us_ = 0;

    // PLEASE Check src/simpleukf_tools.h for noise constants.
    // The main logic is encapsulated in the submodule src/simpleukf [https://github.com/andrea-nisti/simple-ukf]
}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    bool state_update{false};
    if (not is_initialized_)
    {
        // Initialize the filter with a laser measure
        CTRVFusionFilter::StateVector_t predicted_state;
        predicted_state.fill(0.0f);
        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            predicted_state.head(2) = meas_package.raw_measurements_;
            ctrv_ukf_filter_.Init(predicted_state, CTRVFusionFilter::StateCovMatrix_t::Identity());
            x_ = ctrv_ukf_filter_.GetCurrentStateVector();
            P_ = ctrv_ukf_filter_.GetCurrentCovarianceMatrix();

            std::cout << ctrv_ukf_filter_.GetCurrentStateVector() << std::endl;
            is_initialized_ = true;
            state_update = true;
        }
    }
    else
    {
        const double delta_t = (meas_package.timestamp_ - time_us_) / 1e6;

        // check if last fusion timestamp is recent or old
        if (delta_t > 0.00001f)
        {
            Prediction(delta_t);
            state_update = true;
        }

        // fuse correct measurement
        if ((meas_package.sensor_type_ == MeasurementPackage::LASER) and use_laser_)
        {
            UpdateLidar(meas_package);
            state_update = true;
        }
        else if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) and use_radar_)
        {
            UpdateRadar(meas_package);
            state_update = true;
        }
        else
        {
            std::cout << "Received wrong sensor type, skipping package" << std::endl;
        }
    }

    // update last fusion timestamp and filter states
    if (state_update)
    {
        time_us_ = meas_package.timestamp_;
    }
}

void UKF::Prediction(double delta_t)
{
    ctrv_ukf_filter_.PredictProcessMeanAndCovariance(delta_t);

    x_ = ctrv_ukf_filter_.GetCurrentStateVector();
    P_ = ctrv_ukf_filter_.GetCurrentCovarianceMatrix();
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    // clang-format off
    static const auto H = ( Eigen::Matrix<double, 2, 5>() << 
                                                        1, 0, 0, 0, 0,
                                                        0, 1, 0, 0, 0).finished();
    // clang-format on

    auto strategy = simpleukf::ukf::LinearUpdateStrategy<CTRVProcessModel, LidarMeasurementModel>{H};
    ctrv_ukf_filter_.UpdateState<LidarMeasurementModel>(meas_package.raw_measurements_, strategy);

    x_ = ctrv_ukf_filter_.GetCurrentStateVector();
    P_ = ctrv_ukf_filter_.GetCurrentCovarianceMatrix();
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    auto strategy = simpleukf::ukf::UnscentedUpdateStrategy<CTRVProcessModel, RadarMeasurementModel>{
        ctrv_ukf_filter_.GetCurrentPredictedSigmaMatrix()};
    ctrv_ukf_filter_.UpdateState<RadarMeasurementModel>(meas_package.raw_measurements_, strategy);

    x_ = ctrv_ukf_filter_.GetCurrentStateVector();
    P_ = ctrv_ukf_filter_.GetCurrentCovarianceMatrix();
}

}  // namespace fusion