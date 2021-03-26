#include "ukf.h"
#include <chrono>
#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double normalize_angle(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0) {
    angle += 2 * M_PI;
  }
  return angle - M_PI;
};

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF(std::string name) {
  name_ = name;
  // std::cout << "I'm " << this << std::endl;

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
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;
  sigma_point_size_ = 2 * n_aug_ + 1;

  // create vector for weights
  weights_ = VectorXd(sigma_point_size_);
  // set weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  double temp = 0.5 / (lambda_ + n_aug_);
  for (int i = 1; i < sigma_point_size_; i++) {
    weights_(i) = temp;
  }
  // std::cout << this << " weights: \n" << weights_ << "\n" << std::endl;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if (!is_initialized_) {
    is_initialized_ = true;
    // Initialize x_ and P_
    P_ = MatrixXd::Identity(n_x_, n_x_);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1], 0.0, 0.0, 0.0;
    } else {
      x_ << 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    // std::cout << "I'm " << this << std::endl;
    // std::cout << "Init x: \n" << x_ << "\n" << std::endl;
    // std::cout << "Init P: \n" << P_ << "\n" << std::endl;

    return;
  }

  // Do UKF update
  std::cout << "ProcessMeasurement++" << std::endl;
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } else {
    UpdateRadar(meas_package);
  }
  std::cout << "ProcessMeasurement--" << std::endl;
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location.
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */
  // Generate sigma points
  // std::cout << "Prediction++" << std::endl;
  // std::chrono::steady_clock::time_point t1 =
  // std::chrono::steady_clock::now();
  MatrixXd Xsig_aug;
  AugmentedSigmaPoints(Xsig_aug);
  // std::chrono::steady_clock::time_point t2 =
  // std::chrono::steady_clock::now(); std::cout
  //     << "AugmentedSigmaPoints: "
  //     << std::chrono::duration_cast<std::chrono::milliseconds>(t2 -
  //     t1).count()
  //     << std::endl;
  // Predict sigma points
  SigmaPointPrediction(Xsig_aug, delta_t);
  // std::chrono::steady_clock::time_point t3 =
  // std::chrono::steady_clock::now(); std::cout
  //     << "SigmaPointPrediction: "
  //     << std::chrono::duration_cast<std::chrono::milliseconds>(t3 -
  //     t2).count()
  //     << std::endl;
  // Predict mean and covariance
  PredictMeanAndCovariance();
  // std::chrono::steady_clock::time_point t4 =
  // std::chrono::steady_clock::now(); std::cout
  //     << "PredictMeanAndCovariance: "
  //     << std::chrono::duration_cast<std::chrono::milliseconds>(t4 -
  //     t3).count()
  //     << std::endl;
  // std::cout << "I'm " << this << std::endl;
  std::cout << "Xsig_aug: \n" << Xsig_aug << "\n" << std::endl;
  std::cout << "Prediction Xsig_pred_: \n" << Xsig_pred_ << "\n" << std::endl;
  std::cout << "Predict x: \n" << x_ << "\n" << std::endl;
  std::cout << "Predict P: \n" << P_ << "\n\n" << std::endl;
  // std::cout << "Prediction--\n\n" << std::endl;
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd& Xsig_aug) {
  // create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  // create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  // create sigma point matrix
  Xsig_aug = MatrixXd(n_aug_, sigma_point_size_);

  // create augmented mean state
  x_aug.head(n_x_) = x_;

  // create augmented covariance matrix
  MatrixXd Q = MatrixXd::Zero(2, 2);
  Q(0, 0) = std::pow(std_a_, 2);
  Q(1, 1) = std::pow(std_yawdd_, 2);
  P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug.bottomRightCorner(Q.rows(), Q.cols()) = Q;

  // create square root matrix of P_aug
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  static const double square_root_lambda = std::sqrt(lambda_ + n_aug_);
  Xsig_aug.col(0) = x_aug;
  for (int i = 1; i <= n_aug_; i++) {
    VectorXd sigma = square_root_lambda * A.col(i - 1);
    Xsig_aug.col(i) = x_aug + sigma;
    Xsig_aug.col(i + n_aug_) = x_aug - sigma;
  }
}

void UKF::SigmaPointPrediction(Eigen::MatrixXd& Xsig_aug, double delta_t) {
  // create matrix with predicted sigma points as columns
  // std::cout << "SigmaPointPrediction++" << std::endl;
  Xsig_pred_ = MatrixXd(n_x_, sigma_point_size_);

  double delta_t_squared = std::pow(delta_t, 2);
  for (int i = 0; i < Xsig_aug.cols(); i++) {
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    VectorXd noise = VectorXd(n_x_);
    noise(0) = 0.5 * delta_t_squared * std::cos(yaw) * nu_a;
    noise(1) = 0.5 * delta_t_squared * std::sin(yaw) * nu_a;
    noise(2) = delta_t * nu_a;
    noise(3) = 0.5 * delta_t_squared * nu_yawdd;
    noise(4) = delta_t * nu_yawdd;

    // predict sigma points
    VectorXd pred_sigma_point = VectorXd(n_x_);
    if (std::fabs(yawd) > std::numeric_limits<double>::epsilon()) {
      double temp1 = v / yawd;
      double temp2 = yaw + yawd * delta_t;
      pred_sigma_point(0) =
          p_x + temp1 * (std::sin(temp2) - std::sin(yaw)) + noise(0);
      pred_sigma_point(1) =
          p_y + temp1 * (-std::cos(temp2) + std::cos(yaw)) + noise(1);

    } else {
      // avoid division by zero
      pred_sigma_point(0) = p_x + v * std::cos(yaw) * delta_t + noise(0);
      pred_sigma_point(1) = p_y + v * std::sin(yaw) * delta_t + noise(1);
    }

    pred_sigma_point(2) = v + noise(2);
    pred_sigma_point(3) = yaw + yawd * delta_t + noise(3);
    pred_sigma_point(4) = yawd + noise(4);

    // write predicted sigma points into right column
    Xsig_pred_.col(i) = pred_sigma_point;
  }
  // std::cout << "SigmaPointPrediction Xsig_pred_: \n"
  //           << Xsig_pred_ << "\n"
  //           << std::endl;
}

void UKF::PredictMeanAndCovariance() {
  // predict state mean
  x_.fill(0.0);
  for (int i = 0; i < sigma_point_size_; ++i) {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < sigma_point_size_; ++i) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // angle normalization
    // x_diff(3) = lambda(x_diff(3));
    if (x_diff(3) > M_PI) {
      // int temp = x_diff(3) / (2. * M_PI);
      // x_diff(3) -= temp * 2. * M_PI;
      // std::cout << "yaw++: " << x_diff(3) << std::endl;
      x_diff(3) = normalize_angle(x_diff(3));
      // std::cout << "yaw--: " << x_diff(3) << std::endl;
    } else if (x_diff(3) < -M_PI) {
      // int temp = std::fabs(x_diff(3)) / (2. * M_PI);
      // x_diff(3) += temp * 2. * M_PI;
      // std::cout << "yaw++: " << x_diff(3) << std::endl;
      x_diff(3) = normalize_angle(x_diff(3));
      // std::cout << "yaw--: " << x_diff(3) << std::endl;
    }

    /* while (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    }
    while (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    } */

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  // std::cout << "PredictMeanAndCovariance Xsig_pred_: \n"
  //           << Xsig_pred_ << "\n"
  //           << std::endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  MatrixXd Zsig;
  VectorXd z_pred;
  MatrixXd S;
  // std::cout << "I'm " << this << std::endl;
  std::cout << "UpdateLidar Xsig_pred_: \n" << Xsig_pred_ << "\n" << std::endl;
  std::cout << "UpdateLidar x: \n" << x_ << "\n" << std::endl;
  std::cout << "UpdateLidar P: \n" << P_ << "\n\n" << std::endl;
  // std::cout << "UpdateLidar" << std::endl;
  PredictLidarMeasurement(Zsig, z_pred, S);
  std::cout << "UpdateLidar, Zsig: \n" << Zsig << "\n" << std::endl;
  std::cout << "UpdateLidar, z_pred: \n" << z_pred << "\n" << std::endl;
  std::cout << "UpdateLidar, S: \n" << S << "\n\n" << std::endl;
  UpdateState(meas_package, Zsig, z_pred, S);
  std::cout << "UpdateLidar x: \n" << x_ << "\n" << std::endl;
  std::cout << "UpdateLidar P: \n" << P_ << "\n\n" << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  MatrixXd Zsig;
  VectorXd z_pred;
  MatrixXd S;
  std::cout << "UpdateRadar Xsig_pred_: \n" << Xsig_pred_ << "\n" << std::endl;
  std::cout << "UpdateRadar x: \n" << x_ << "\n" << std::endl;
  std::cout << "UpdateRadar P: \n" << P_ << "\n\n" << std::endl;
  PredictRadarMeasurement(Zsig, z_pred, S);
  std::cout << "UpdateRadar, Zsig: \n" << Zsig << "\n" << std::endl;
  std::cout << "UpdateRadar, z_pred: \n" << z_pred << "\n" << std::endl;
  std::cout << "UpdateRadar, S: \n" << S << "\n\n" << std::endl;
  UpdateState(meas_package, Zsig, z_pred, S);
  std::cout << "UpdateRadar x: \n" << x_ << "\n" << std::endl;
  std::cout << "UpdateRadar P: \n" << P_ << "\n\n" << std::endl;
}

void UKF::PredictLidarMeasurement(Eigen::MatrixXd& Zsig,
                                  Eigen::VectorXd& z_pred, Eigen::MatrixXd& S) {
  // set measurement dimension, radar can measure r, phi, and r_dot
  constexpr int n_z = 2;
  // create matrix for sigma points in measurement space
  Zsig = MatrixXd(n_z, sigma_point_size_);

  // mean predicted measurement
  z_pred = VectorXd::Zero(n_z);

  // measurement covariance matrix S
  S = MatrixXd::Zero(n_z, n_z);

  // transform sigma points into measurement space
  MatrixXd H = MatrixXd::Zero(n_z, n_x_);
  H(0, 0) = 1;
  H(1, 1) = 1;

  std::cout << "PredictLidar, Xsig_pred_: \n"
            << Xsig_pred_ << "\n"
            << std::endl;
  Zsig = H * Xsig_pred_;
  // std::cout << "Predict lidar: \n";
  // std::cout << "Zsig: \n" << Zsig << "\n" << std::endl;

  // calculate mean predicted measurement
  for (int i = 0; i < sigma_point_size_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  // std::cout << "z_pred: \n" << z_pred << "\n" << std::endl;

  // calculate innovation covariance matrix S
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;

  for (int i = 0; i < sigma_point_size_; i++) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R;
  // std::cout << "S: \n" << S << "\n\n" << std::endl;
}

void UKF::PredictRadarMeasurement(Eigen::MatrixXd& Zsig,
                                  Eigen::VectorXd& z_pred, Eigen::MatrixXd& S) {
  // set measurement dimension, radar can measure r, phi, and r_dot
  constexpr int n_z = 3;
  // create matrix for sigma points in measurement space
  Zsig = MatrixXd(n_z, sigma_point_size_);

  // mean predicted measurement
  z_pred = VectorXd::Zero(n_z);

  // measurement covariance matrix S
  S = MatrixXd::Zero(n_z, n_z);

  // transform sigma points into measurement space
  std::cout << "PredictRadar, Xsig_pred_: \n"
            << Xsig_pred_ << "\n"
            << std::endl;
  for (int i = 0; i < sigma_point_size_; i++) {
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double vel = Xsig_pred_(2, i);
    double psi = Xsig_pred_(3, i);
    double psi_dot = Xsig_pred_(4, i);

    double rho = std::sqrt(std::pow(px, 2) + std::pow(py, 2));
    double phi = std::atan2(py, px);
    double rho_dot =
        (px * std::cos(psi) * vel + py * std::sin(psi) * vel) / rho;

    Zsig(0, i) = rho;
    Zsig(1, i) = phi;
    Zsig(2, i) = rho_dot;
  }
  std::cout << "PredictRadar, Zsig: \n" << Zsig << "\n" << std::endl;

  // calculate mean predicted measurement
  for (int i = 0; i < sigma_point_size_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  std::cout << "PredictRadar, z_pred: \n" << z_pred << "\n" << std::endl;

  // calculate innovation covariance matrix S
  MatrixXd R = MatrixXd::Zero(n_z, n_z);
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;

  for (int i = 0; i < sigma_point_size_; i++) {
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    if (z_diff(1) > M_PI) {
      // std::cout << "yaw++: " << z_diff(1) << std::endl;
      z_diff(1) = normalize_angle(z_diff(1));
      // std::cout << "yaw--: " << z_diff(1) << std::endl;
    } else if (z_diff(1) < -M_PI) {
      // std::cout << "yaw++: " << z_diff(1) << std::endl;
      z_diff(1) = normalize_angle(z_diff(1));
      // std::cout << "yaw--: " << z_diff(1) << std::endl;
    }
    /* while (z_diff(1) > M_PI) {
      z_diff(1) -= 2. * M_PI;
    }
    while (z_diff(1) < -M_PI) {
      z_diff(1) += 2. * M_PI;
    } */

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  S = S + R;
  std::cout << "PredictRadar, S: \n" << S << "\n" << std::endl;
}

void UKF::UpdateState(MeasurementPackage& meas_package, Eigen::MatrixXd& Zsig,
                      Eigen::VectorXd& z_pred, Eigen::MatrixXd& S) {
  // create matrix for cross correlation Tc
  int n_z = meas_package.raw_measurements_.size();
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // calculate cross correlation matrix
  for (int i = 0; i < sigma_point_size_; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    VectorXd z_diff = Zsig.col(i) - z_pred;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  std::cout << "Tc: " << Tc << "\n" << std::endl;

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  std::cout << "S.inverse(): " << S.inverse() << "\n" << std::endl;
  std::cout << "K: " << K << "\n" << std::endl;

  // update state mean and covariance matrix
  x_ = x_ + K * (meas_package.raw_measurements_ - z_pred);
  P_ = P_ - K * S * K.transpose();
}