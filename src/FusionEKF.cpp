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
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

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
    * Set the process and measurement noises
  */
  ekf_ = KalmanFilter();
  H_laser_ << 1, 0, 0, 0,
		  	  0, 1, 0, 0;
  ekf_.F_ = MatrixXd::Zero(4,4);
  ekf_.Q_ = MatrixXd::Zero(4,4);
  ekf_.P_ = MatrixXd::Zero(4,4);
  ekf_.P_ << 1, 0, 0, 0,
		  	 0, 1, 0, 0,
			 0, 0, 1000, 0,
			 0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

	const VectorXd& raw_meas = measurement_pack.raw_measurements_;
  if (!is_initialized_)
  {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
    		float rho = raw_meas(0);
    		float phi = raw_meas(1);
    		ekf_.x_(0) = rho * cos(phi);
    		ekf_.x_(1) = rho * sin(phi);
    }
    else
    	{
    		ekf_.x_(0) = raw_meas(0);
    		ekf_.x_(1) = raw_meas(1);
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
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
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt =  (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt2 = dt*dt;
  float dt3 = dt2*dt / 2.0;
  float dt4 = dt3*dt / 4.0;
  float noise_ax = 9, noise_ay = 9;
  ekf_.F_ << 1, 0, dt, 0,
		  	 0, 1, 0, dt,
			 0, 0, 1, 0,
			 0, 0, 0, 1;
  ekf_.Q_ << dt4*noise_ax, 0, dt3*noise_ax, 0,
		  	  0, dt4*noise_ay, 0, dt3*noise_ay,
			  dt3*noise_ax, 0, dt2*noise_ax, 0,
			  0, dt3*noise_ay, 0, dt2*noise_ay;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
	  ekf_.R_ = R_radar_;
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }
  previous_timestamp_ = measurement_pack.timestamp_;
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
