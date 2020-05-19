#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // set state dimension
  n_x_ = 5;

  // set augmented dimension
  n_aug_ = 7;

  // set sigma points number
  n_sig_ = 2* n_aug_ + 1;

  //create sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, N_sig_);
  Xsig_pred_.fill(0.0);

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set vector for weights
  weights_ = VectorXd(n_sig_);
    
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights(0) = lambda_ / (lambda_ + n_aug_);

  ///* the current NIS for radar
  NIS_radar_ = 0.0;

  ///* the current NIS for laser
  NIS_laser_ = 0.0;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  // TODO : Check if data is initialized ?

  if (!is_initialized_) {
    /**
     * TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       */
      double rho = meas_package.raw_measurements_[0]; // range
      double phi = meas_package.raw_measurements_[1]; // bearing
      double rho_dot = meas_package.raw_measurements_[2]; // velocity of rho

      double px = rho * cos(phi); // position x
      double py = rho * sin(phi); // position y
      double vx = rho_dot * cos(phi); // velocity x
      double vy = rho_dot * sin(phi); // velocity y
      double v = sqrt(vx * vx + vy * vy); // velocity
      
      x_ << px, py, v, 0, 0;
    } // MeasurementPackage::RADAR
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
      /**
       * Initialize state.
       */

      double px = meas_package.raw_measurements_[0]; // position x
      double py = meas_package.raw_measurements_[1]; // position y

      x_ << px, py, 0, 0, 0;
    } // MeasurementPackage::LASER
    else {
      cout << "Undefined measurement package." << endl;
    }

    // Initial measurement timestamp
    time_us_ = meas_package.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  } // is_initialized_ 

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // TODO : Call predition step with calculated time interval

  // Calculate time interval dt
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;

  // Update measurement timestamp  
  time_us_ = meas_package.timestamp_;

  // Call prediction
  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  // TODO : Call update step with given senser type
	
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }
  else {
    cout << "Undefined measurement package." << endl;
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
}