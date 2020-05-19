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
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);
  Xsig_pred_.fill(0.0);

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set vector for weights
  weights_ = VectorXd(n_sig_);
    
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights(0) = lambda_ / (lambda_ + n_aug_);

  // Create lidar covariance matrix
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_,                       0,
	                                  0, std_laspy_ * std_laspy_;

  // Create radar covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_,                         0,                       0,
	                                0, std_radphi_ * std_radphi_,                       0,
	                                0,                         0, std_radrd_ * std_radrd_;

  ///* the current NIS for laser
  NIS_laser_ = 0.0;

  ///* the current NIS for radar
  NIS_radar_ = 0.0;
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

  /*****************************************************************************
  *  Augment Sigma Points
  ****************************************************************************/
  // TODO : Augment Sigma Points

  // Create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // Create augmented sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  // Create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.fill(0.0);
  Xsig_aug.col(0) = x_aug;

  double sig_sqrt = sqrt(lambda_ + n_aug_);
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1)          = x_aug + L.col(i) * sig_sqrt;
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - L.col(i) * sig_sqrt;
  }

  /*****************************************************************************
  *  Predict Sigma Points
  ****************************************************************************/
  // TODO : Predict sigma points

  for (int i = 0; i < n_sig_; i++) {
    // Read values from current state vector
    double p_x      = Xsig_aug(0, i);
    double p_y      = Xsig_aug(1, i);
    double v        = Xsig_aug(2, i);
    double yaw      = Xsig_aug(3, i);
    double yawd     = Xsig_aug(4, i);
    double nu_a     = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // Initialize predicted state
    double px_p, py_p;

    // Avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * ( sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (-cos(yaw + yawd * delta_t) + cos(yaw));
    }
    else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    //predict sigma points

    double v_p    = v;
    double yaw_p  = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // Add Noise
    px_p   += 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p   += 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p    += nu_a * delta_t;
    yaw_p  += 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p += nu_yawdd * delta_t;

    //write predicted sigma points into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /*****************************************************************************
  *  Calculate mean and variance
  ****************************************************************************/
  // TODO : Calculate mean
  // TODO : Calculate variance
  // TODO : Normalize angles

  // Predict state mean
  x_ += Xsig_pred_ * weights_;

  // Predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    // Calculate the residual on x
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Normalize angle
    while (x_diff(3) > M_PI) {
      x_diff(3) -= 2.0 * M_PI;
    }

    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2.0 * M_PI;
    }

    P_ += weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  /*****************************************************************************
  *  Predict lidar measurement
  ****************************************************************************/

  //set measurement dimension, lidar can measure position x and position y
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);
  Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);

	//calculate mean predicted measurement
	z_pred.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}

  //calculate innovation covariance matrix S
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) { //2n+1 simga points
		// Calculate the residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		S += weights_(i) * z_diff * z_diff.transpose();
	}

	// Add measurement noise to covariance matrix
	S += R_lidar_;

  /*****************************************************************************
	*  UKF update lidar measurement
	****************************************************************************/
	
  //create example vector for incoming radar measurement
	VectorXd z = meas_package.raw_measurements_;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);

	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < n_sig_; i++) { //2n+1 simga points
		// Calculate the residual on z
		VectorXd z_diff = Zsig.col(i) - z_pred;

		// Calculate the difference on state
		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//update state mean and covariance matrix
	// Calculate the residual on z
	VectorXd z_diff = z - z_pred;

	// Update state mean and covariance matrix
	x_ += K * z_diff;
	P_ -= K * S * K.transpose();

	// Calculate NIS update
	NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  /*****************************************************************************
  *  Predict radar measurement
  ****************************************************************************/

  //set measurement dimension, radar can measure r, phi, and r_dot
	int n_z = 3;

	//create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, n_sig_);

	//mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);

	//measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z, n_z);

	//transform sigma points into measurement space
	for (int i = 0; i < n_sig_; i++) {
		// Read values from current state vector
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v   = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1  = cos(yaw) * v;
		double v2  = sin(yaw) * v;

		// Update measurement model
		Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                         // r
		Zsig(1, i) = atan2(p_y, p_x);                                     // phi
		Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); // r_dot
	}

	//calculate mean predicted measurement
	z_pred.fill(0.0);
	for (int i = 0; i < n_sig_; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}

	//calculate innovation covariance matrix S
	S.fill(0.0);
	for (int i = 0; i < n_sig_; i++) { //2n+1 simga points
    // Calculate the residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		// Normalize angle
		while (z_diff(1) > M_PI) {
			z_diff(1) -= 2.0 * M_PI;
		}

		while (z_diff(1) < -M_PI) {
			z_diff(1) += 2.0 * M_PI;
		}

		S += weights_(i) * z_diff * z_diff.transpose();
	}

	// Add measurement noise to covariance matrix
	S += R_radar_;

	/*****************************************************************************
	*  UKF update radar measurement
	****************************************************************************/

	//create example vector for incoming radar measurement
	VectorXd z = meas_package.raw_measurements_;

	//create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);

	//calculate cross correlation matrix
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) { //2n+1 simga points
    // Calculate the residual on z
		VectorXd z_diff = Zsig.col(i) - z_pred;

		// Normalize angle
		while (z_diff(1) > M_PI) {
			z_diff(1) -= 2.0 * M_PI;
		}

		while (z_diff(1) < -M_PI) {
			z_diff(1) += 2.0 * M_PI;
		}

		// Calculate the difference on state
		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		// Normalize angle
		while (x_diff(3) > M_PI) {
			x_diff(3) -= 2.0 * M_PI;
		}

		while (x_diff(3) < -M_PI) {
			x_diff(3) += 2.0 * M_PI;
		}

		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	//calculate Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//update state mean and covariance matrix
	// Calculate the residual on z
	VectorXd z_diff = z - z_pred;

	// Normalize angle
	while (z_diff(1) > M_PI) {
		z_diff(1) -= 2.0 * M_PI;
	}

	while (z_diff(1) < -M_PI) {
		z_diff(1) += 2.0 * M_PI;
	}

	// Update state mean and covariance matrix
	x_ += K * z_diff;
	P_ -= K * S * K.transpose();

	// Calculate NIS update
	NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}