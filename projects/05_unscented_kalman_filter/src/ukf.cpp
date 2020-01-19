#include "ukf.h"
#include "Eigen/Dense"

#include <iostream>

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
	std_a_ = 1.0;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 0.5;
  
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
	n_x_ = 5;
	n_aug_ = 7;
	lambda_ = 3 - n_aug_;

	Xsig_pred_ = Eigen::MatrixXd(n_x_, 2*n_aug_+1);
	weights_ = Eigen::VectorXd(2*n_aug_+1);

	double weight_0 = lambda_/(lambda_ + n_aug_);
	double weight = 0.5/(lambda_ + n_aug_);
	weights_(0) = weight_0;

	for (int i{1}; i<2*n_aug_+1; ++i) {
		weights_(i) = weight;
	}	

}



UKF::~UKF() {}



void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
	if (! is_initialized_) {
		x_.fill(0);
		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			// initialize px, py with laser measurements, rest are 0
			x_.head(2) = meas_package.raw_measurements_;
		} 

		P_ = 0.2 * Eigen::MatrixXd::Identity(n_x_, n_x_);
		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;
		return;
	}

	double delta_t { (meas_package.timestamp_-time_us_)/1000000.0 };
	time_us_ = meas_package.timestamp_;

	Prediction(delta_t);
	
	if (meas_package.sensor_type_==MeasurementPackage::LASER) {
		UpdateLidar(meas_package);
	} else {
		UpdateRadar(meas_package);
	}

}



void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

	Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_); // augmented mean vector
	Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_); // augmented state covariance
	Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2*n_aug_+1); // sigma point matrix

	// fill augmented state
	x_aug.head(n_x_) = x_;
	x_aug(n_x_) = 0;
	x_aug(n_x_+1) = 0;

	// fill augmented covariance matrix
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_ * std_a_;
	P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

	// create square root matrix
	Eigen::MatrixXd L = P_aug.llt().matrixL();

	// create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i{0}; i<n_aug_; ++i) {
		Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
	}

	// predict sigma points
	for (int i{0}; i<Xsig_aug.cols(); ++i) {
		// extract values for better readability
		double p_x = Xsig_aug(0,i);
		double p_y = Xsig_aug(1,i);
		double v = Xsig_aug(2,i);
		double yaw = Xsig_aug(3,i);
		double yawd = Xsig_aug(4,i);
		double nu_a = Xsig_aug(5,i);
		double nu_yawdd = Xsig_aug(6,i);

		// predicted state values
		double px_p, py_p;

		// avoid division by zero
		if (fabs(yawd) > 0.001) { 
		    px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
		    py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
		} else {
		    px_p = p_x + v*delta_t*cos(yaw);
		    py_p = p_y + v*delta_t*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd*delta_t;
		double yawd_p = yawd;

		// add noise
		px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
		py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
		v_p = v_p + nu_a*delta_t;

		yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
		yawd_p = yawd_p + nu_yawdd*delta_t;

		// write predicted sigma point into right column
		Xsig_pred_(0,i) = px_p;
		Xsig_pred_(1,i) = py_p;
		Xsig_pred_(2,i) = v_p;
		Xsig_pred_(3,i) = yaw_p;
		Xsig_pred_(4,i) = yawd_p;
	}

	// predicted state mean
	x_.fill(0);
	for (int i{0}; i<2*n_aug_+1; ++i) {  // iterate over sigma points
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
		// I do it	
	}

	// // angle normalization
	// while (x_(3)> M_PI) x_(3)-=2.*M_PI;
	// while (x_(3)<-M_PI) x_(3)+=2.*M_PI;	

	// predicted state covariance matrix
	P_.fill(0);
	for (int i{0}; i<2*n_aug_+1; ++i) {  // iterate over sigma points
		// state difference
		Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
		NormalizeAngle(&x_diff(3));

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
	}	


}


void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

	int n_z{2};

	// create matrix for sigma points in measurement space
	Eigen::MatrixXd Zsig = Xsig_pred_.topLeftCorner(n_z, 2*n_aug_+1);

	// mean predicted measurement
	Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
	z_pred.fill(0);

	// measurement covariance matrix S
	Eigen::MatrixXd S = Eigen::MatrixXd(n_z,n_z);
	S.fill(0);

	// calculate mean predicted measurement
	for (int i=0; i<Xsig_pred_.cols(); ++i) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}  

	// calculate innovation covariance matrix S
	for (int i=0; i<Zsig.cols(); ++i) {
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred; 
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}  

	S(0,0) += std_laspx_ * std_laspx_;
	S(1,1) += std_laspy_ * std_laspy_;

	Eigen::VectorXd z = meas_package.raw_measurements_;

	// create matrix for cross correlation Tc
	Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
	Tc.fill(0);

	// calculate cross correlation matrix
	for (int i{0}; i<Zsig.cols(); ++i) {
		// residual
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		NormalizeAngle(&x_diff(3));
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	// calculate Kalman gain K;
	Eigen::MatrixXd K = Tc * S.inverse();

	// update state mean and covariance matrix
	Eigen::VectorXd z_diff = z - z_pred;

	// update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();	

}


void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

	int n_z {3}; // dimension of radar measurement

	// create matrix for sigma points in measurement space
	Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

	// mean predicted measurement
	Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
	z_pred.fill(0);

	// measurement covariance matrix S
	Eigen::MatrixXd S = Eigen::MatrixXd(n_z,n_z);
	S.fill(0);

	/**
	* Student part begin
	*/

	// transform sigma points into measurement space
	for (int i=0; i<Xsig_pred_.cols(); ++i) {
		double px = Xsig_pred_(0,i);
		double py = Xsig_pred_(1,i);
		double v = Xsig_pred_(2,i);
		double phi = Xsig_pred_(3,i);
		double dphi = Xsig_pred_(4,i);

		double rho = sqrt( px * px + py * py );
		double shi = atan2(py, px);
		NormalizeAngle(&shi);

		double drho = (px * cos(phi) + py * sin(phi) ) * v / rho;

		Zsig(0,i) = rho;
		Zsig(1,i) = shi;
		Zsig(2,i) = drho;
	}

	// calculate mean predicted measurement
	for (int i=0; i<Xsig_pred_.cols(); ++i) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}  

	// calculate innovation covariance matrix S
	for (int i=0; i<Zsig.cols(); ++i) {
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred; 
		NormalizeAngle(&z_diff(1));
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}  

	S(0,0) += std_radr_ * std_radr_;
	S(1,1) += std_radphi_ * std_radphi_;
	S(2,2) += std_radrd_ * std_radrd_;


	Eigen::VectorXd z = meas_package.raw_measurements_;

	// create matrix for cross correlation Tc
	Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);
	Tc.fill(0);

	// calculate cross correlation matrix
	for (int i{0}; i<Zsig.cols(); ++i) {
		// residual
		Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
		// angle normalization
		while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		NormalizeAngle(&x_diff(3));
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	// calculate Kalman gain K;
	Eigen::MatrixXd K = Tc * S.inverse();

	// update state mean and covariance matrix
	Eigen::VectorXd z_diff = z - z_pred;

	NormalizeAngle(&z_diff(1));
	// update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();	


}

