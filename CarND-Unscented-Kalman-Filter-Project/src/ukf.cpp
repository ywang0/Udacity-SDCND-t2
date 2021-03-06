#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 1.0e-6

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // define constants
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  n_z_laser_ = 2;
  n_z_radar_ = 3;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2 (needs to be tuned)
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2 (needs to be tuned)
  std_yawdd_ = 0.6;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // state covariance matrix (needs to be tuned)
  P_  << 0.01, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0,
        0, 0, 3, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // laser measurement space mapping matrix
  H_laser_ = MatrixXd(n_z_laser_, n_x_);
  H_laser_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;

  // laser measurement noise covariance matrix
  R_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
  R_laser_ << std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;

  // radar measurement noise covariance matrix
  R_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0, std_radrd_*std_radrd_;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  // weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  // Normalized Innovation Squared (NIS)
  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The first measurement data of
 * either radar or laser.
 */
void UKF::InitValues(const MeasurementPackage meas_package) {
  x_ << 1, 1, 1, 1, 1;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    double rho = meas_package.raw_measurements_[0];
    double phi = meas_package.raw_measurements_[1];
    x_(0) = rho * cos(phi);
    x_(1) = rho * sin(phi);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    x_(0) = meas_package.raw_measurements_[0];
    x_(1) = meas_package.raw_measurements_[1];
  }

  time_us_ = meas_package.timestamp_;
  is_initialized_ = true;

  return;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    cout << "UKF: " << endl;
    InitValues(meas_package);
    return;
  }

  // compute the time elapsed between the current and previous measurements
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  // predict next state
  Prediction(dt);

  // measurement update
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    UpdateRadar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    UpdateLidar(meas_package);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
   * calculate augmented sigma points Xsig_aug
   */
  VectorXd x_aug(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  MatrixXd P_aug(n_aug_, n_aug_);
  MatrixXd Q(2, 2);
  Q << std_a_*std_a_, 0,
      0, std_yawdd_*std_yawdd_;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  // square root of state augmented covariance matrix
  MatrixXd L = P_aug.llt().matrixL();

  MatrixXd Xsig_aug(n_aug_, 2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  /**
   * predict augmented sigma points
   */
  for (int i = 0; i< 2*n_aug_+1; i++) {
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    double px_pred, py_pred;
    //avoid division by zero
    if (fabs(yawd) > EPS) {
      px_pred = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_pred = py + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_pred = px + v * delta_t * cos(yaw);
      py_pred = py + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;
    //add noise
    px_pred = px_pred + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_pred = py_pred + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;
    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /**
   * predict state mean and state covariance
   */
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  P_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  * transform sigma points into measurement space
  */

  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - H_laser_ * x_;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S(n_z_laser_, n_z_laser_);
  S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ *  Ht * Si;

  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
  NIS_laser_ = y.transpose() * Si * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * transform sigma points into measurement space
   */
  MatrixXd Zsig(n_z_radar_, 2*n_aug_+1);
  VectorXd z_pred(n_z_radar_);
  MatrixXd S(n_z_radar_, n_z_radar_);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double phi = Xsig_pred_(3, i);
    double rho = sqrt(px*px + py*py);

    Zsig(0, i) = rho;
    Zsig(1, i) = (py==0.0 && px==0.0)? 0.0 : atan2(py, px);
    Zsig(2, i) = (px*cos(phi)*v + py*sin(phi)*v) / max(EPS, rho);
  }

  /**
   * calculate mean predicted measurement
   */
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  /**
   * calculate innovation covariance matrix S
   */
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R_radar_;

  /**
   * calculate cross correlation matrix Tc
   */
  MatrixXd Tc(n_x_, n_z_radar_);

  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  /**
   * calculate Kalman gain K
   */
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc * Si;

  /**
   * update state mean and covariance matrix
   */
  VectorXd z = meas_package.raw_measurements_;
  VectorXd residual = z - z_pred;
  //angle normalization
  while (residual(1)> M_PI) residual(1)-=2.*M_PI;
  while (residual(1)<-M_PI) residual(1)+=2.*M_PI;

  x_ = x_ + K * residual;
  P_ = P_ - K * S * K.transpose();
  NIS_radar_ = residual.transpose() * Si * residual;
}
