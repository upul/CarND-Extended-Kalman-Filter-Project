#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;



    /**
    TODO:
      * Finish initializing the FusionEKF.
    */
    H_laser_ << 1, 0, 0, 0,
            0, 1, 1, 1;

    MatrixXd F_ = MatrixXd(4, 4);
    MatrixXd Q_ = MatrixXd(4, 4);
    VectorXd x_ = VectorXd(4);

    x_ << 1, 1, 1, 1;

    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "EKF: " << endl;
        //ekf_.x_ = VectorXd(4);
        //ekf_.x_ << 1, 1, 1, 1;
        float px = 0.0;
        float py = 0.0;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            float r = measurement_pack.raw_measurements_[0];
            float theta = measurement_pack.raw_measurements_[1];
            px = r * cos(theta);
            py = r * sin(theta);


        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];

        }

        // done initializing, no need to predict or update
        ekf_.x_ << px, py, 0, 0;
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use sigma_ax = 9 and sigma_ay = 9 for your Q matrix.
     */
    float current_timestamp = measurement_pack.timestamp_;
    float time_diff = (previous_timestamp_ - current_timestamp) / 1000000.0;
    ekf_.F_ << 1, 0, time_diff, 0,
            0, 1, 0, time_diff,
            0, 0, 1, 0,
            0, 0, 0, 1;

    float dt_4 = time_diff*time_diff*time_diff*time_diff;
    float dt_3 = time_diff*time_diff*time_diff;
    float dt_2 = time_diff*time_diff;

    ekf_.Q_ << dt_4/4*sigma_ax, 0, dt_3/2*sigma_ax, 0,
            0, dt_4/4*sigma_ay, 0, dt_3/2*sigma_ay,
            dt_3/2*sigma_ax, 0, dt_2*sigma_ax, 0,
            0, dt_3/2*sigma_ay, 0, dt_2*sigma_ax;


    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates

    } else {
        // Laser updates
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
