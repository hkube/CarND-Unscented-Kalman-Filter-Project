#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF()
: use_laser_(true)  // if this is false, laser measurements will be ignored (except during init)
, use_radar_(true) // if this is false, radar measurements will be ignored (except during init)
, n_x_(5)
, n_z_(3)
, n_aug_(7)
, lambda_(3 - n_aug_)
, n_sigma_points_(2 * n_aug_ + 1)
, is_initialized_(false)
, time_us_(0LL)
, num_of_radar_data_(0)
, nis_limit_radar_(7.815)
, nis_radar_above_limit_(0)
, num_of_lidar_data_(0)
, nis_lidar_above_limit_(0)
, nis_limit_lidar_(5.991)
{
  weights_ = VectorXd(n_sigma_points_);
  weights_.fill( 0.5 / static_cast<double>(lambda_ + n_aug_) );
  weights_(0) = static_cast<double>(lambda_) / static_cast<double>(lambda_ + n_aug_);

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.setZero();
  for (int i = 0; n_x_ > i; i++)
  {
    P_(i, i) = 1;
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3; //30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.8; //30;
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

//  std::cout << "weights_=[" << std::endl << weights_ << "]" << std::endl;

  Xsig_pred_ = MatrixXd(n_x_, n_sigma_points_);

  R_radar_ = MatrixXd(n_z_, n_z_);
  R_radar_.setZero();
  R_radar_(0,0) = std_radr_ * std_radr_;
  R_radar_(1,1) = std_radphi_ * std_radphi_;
  R_radar_(2,2) = std_radrd_ * std_radrd_;
  // time_us_ =

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_.setZero();
  R_lidar_(0, 0) = std_laspx_ * std_laspx_;
  R_lidar_(1, 1) = std_laspy_ * std_laspy_;

  H_ = MatrixXd(2, n_x_);
  H_.setZero();
  H_(0, 0) = 1;
  H_(1, 1) = 1;

  /************* TESTCODE ***************/
#if 0
  is_initialized_ = true;

  x_ <<   5.7441, 1.3800, 2.2049, 0.5015, 0.3528;
  P_ <<    0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  std_a_ = 0.2;
  std_yawdd_ = 0.2;
  std_radr_ = 0.3;
  std_radphi_ = 0.0175;
  std_radrd_ = 0.1;
  R_radar_(0,0) = std_radr_ * std_radr_;
  R_radar_(1,1) = std_radphi_ * std_radphi_;
  R_radar_(2,2) = std_radrd_ * std_radrd_;

  MeasurementPackage mp;
  mp.timestamp_ = 100000LLU;
  mp.sensor_type_ = MeasurementPackage::RADAR;
  mp.raw_measurements_ = VectorXd(3);
  mp.raw_measurements_ << 5.9214, 0.2187, 2.0062;

  ProcessMeasurement(mp);

  exit(0);
#endif
  /************* END OF TESTCODE ***************/
}


UKF::~UKF() {}


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage & meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if ( (! use_laser_) && (meas_package.sensor_type_ == MeasurementPackage::LASER)) {
    return;
  }
  if (( ! use_radar_) && (meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
    return;
  }

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "UKF - Initialization: " << endl;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      std::cout << "First Measurement RADAR" << &std::endl;
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      const double rho    = meas_package.raw_measurements_[0];
      const double phi    = meas_package.raw_measurements_[1];
      const double rhodot = meas_package.raw_measurements_[2];
      const double px = rho * std::cos(phi);
      const double py = rho * std::sin(phi);

      x_ << px, py, 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      std::cout << "First Measurement LASER" << &std::endl;
      /**
       * Initialize state.
       */
      const double px = meas_package.raw_measurements_(0);
      const double py = meas_package.raw_measurements_(1);
      x_ << px, py, 0, 0, 0;
    }

    std::cout << "init x_.transpose() = [" << x_.transpose() << "]" << std::endl;

    // done initializing, no need to predict or update
    is_initialized_ = true;
  }
  else {
    static int loopcount = 5;

    // Calculate the time difference and update the state transition matrix
    double deltaT = double(meas_package.timestamp_ - time_us_) / 1e6;
//    std::cout << "deltaT = " << deltaT << std::endl;

    Prediction(deltaT);

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
      UpdateRadar(meas_package);
    } else {
      // Laser updates
      UpdateLidar(meas_package);
    }

    // print the output
//    std::cout << "x_.transpose() = [" << x_.transpose() << "]"<< std::endl;
//    std::cout << "P_ = [" << std::endl << P_ << "]" << std::endl;

    const double lidar_nis_over_limit = double(nis_lidar_above_limit_) / double(num_of_lidar_data_);
    const double radar_nis_over_limit = double(nis_radar_above_limit_) / double(num_of_radar_data_);
    std::cout << "NIS over limit"
              << " - Lidar= " << lidar_nis_over_limit * 100. << "%"
              << " - Radar= " << radar_nis_over_limit * 100. << "%"
              << std::endl;
//    std::cout << "-----------------------------" << std::endl;

    if(0 >= --loopcount)
    {
//      exit(0);
    }
  }
  time_us_ = meas_package.timestamp_;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(const double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

//  // Calculate the square root of P_
//  const MatrixXd A = P_.llt().matrixL();
//
//  // Generate the sigma points
//  MatrixXd Xsig(n_x_, n_sigma_points_);
//  Xsig.setZero();
//  Xsig.col(0) = x_;
//  const double factor = sqrt(lambda_ + n_x_);
//  for (unsigned col = 0; n_x_ > col; col++)
//  {
//      const double offset = factor * A.col(col);
//      Xsig.col(col+1)      = x_ + offset;
//      Xsig.col(col+1+n_x_) = x_ - offset;
//  }

  // Create augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.setZero();
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0;
  x_aug(n_x_+1) = 0;

//  std::cout << "x_aug.transpose() = [" << x_aug.transpose() << "]" << std::endl;

  // Create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_)     = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_ * std_yawdd_;

//  std::cout << "P_aug = [" << std::endl << P_aug << "]" << std::endl;

  // Create augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_points_);
  Xsig_aug.col(0) = x_aug;
  MatrixXd A_aug = P_aug.llt().matrixL();
  const double sqr_lnx = sqrt(lambda_+n_aug_);
  for (int i = 0; i < n_aug_; i++)
  {
    const VectorXd offset    = sqr_lnx * A_aug.col(i);
    Xsig_aug.col(i+1)        = x_aug + offset;
    Xsig_aug.col(i+1+n_aug_) = x_aug - offset;
  }

//  std::cout << "Xsig_aug = [" << std::endl << Xsig_aug << "]" << std::endl;

  // Predict sigma points
  for (unsigned col = 0; Xsig_pred_.cols() > col; col++)
  {
    const VectorXd x_aug_k = Xsig_aug.col(col);
    const double v_k = x_aug_k(2);
    const double psi_k = x_aug_k(3);
    const double psidot_k = x_aug_k(4);
    const double nju_ak = x_aug_k(5);
    const double nju_pddk = x_aug_k(6);

//    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    VectorXd F_xk1(n_x_);
    // Avoid division by zero
    if (1e-6 < fabs(psidot_k))
    {
        // psi_k_dot is not zero
        F_xk1[0] = v_k/psidot_k * (+sin(psi_k + psidot_k*delta_t) - sin(psi_k));
        F_xk1[1] = v_k/psidot_k * (-cos(psi_k + psidot_k*delta_t) + cos(psi_k));
    }
    else
    {
        // psi_k_dot is zero
        F_xk1[0] = v_k * cos(psi_k) * delta_t;
        F_xk1[1] = v_k * sin(psi_k) * delta_t;
    }
    F_xk1[2] = 0;
    F_xk1[3] = psidot_k * delta_t;
    F_xk1[4] = 0;

//    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    VectorXd F_njuk1(n_x_);
    F_njuk1[0] = 0.5 * delta_t*delta_t * cos(psi_k) * nju_ak;
    F_njuk1[1] = 0.5 * delta_t*delta_t * sin(psi_k) * nju_ak;
    F_njuk1[2] = delta_t * nju_ak;
    F_njuk1[3] = 0.5 * delta_t*delta_t * nju_pddk;
    F_njuk1[4] = delta_t * nju_pddk;

//    std::cout << "x_aug_k.transpose() = [" << x_aug_k.transpose() << "]" << std::endl;
//    std::cout << "F_xk1.transpose() = [" << F_xk1.transpose() << "]" << std::endl;
//    std::cout << "F_njuk1.transpose() = [" << F_njuk1.transpose() << "]" << std::endl;
//    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
    Xsig_pred_.col(col) = x_aug_k.head(n_x_) + F_xk1 + F_njuk1;
  }

//  std::cout << "Xsig_pred_ = [" << std::endl << Xsig_pred_ << "]" << std::endl;

  // ******* new test data ******
#if 0
  Xsig_pred_ <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  std::cout << "new Xsig_pred_ = [" << std::endl << Xsig_pred_ << "]" << std::endl;
#endif
  // ****** end of new test data ******

//  std::cout << __FILE__ << ":" << __LINE__ << std::endl;
  // Predicted mean
  x_.setZero();
  for (int i = 0; n_sigma_points_ > i; i++)
  {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

//  std::cout << "x_.transpose() = [" << x_.transpose() << "]" << std::endl;

//  std::cout << __FILE__ << ":" << __LINE__ << std::endl;
  // Predicted state covariance
  P_.setZero();
  for (int i = 0; n_sigma_points_ > i; i++)
  {
    VectorXd diff = Xsig_pred_.col(i) - x_;

    while (+M_PI < diff(3)) {
      std::cout << +M_PI << " < diff(3)=" << diff(3) << std::endl;
      diff(3) -= 2.9*M_PI;
    }
    while (-M_PI > diff(3)) {
      std::cout << -M_PI << " > diff(3)=" << diff(3) << std::endl;
      diff(3) += 2.9*M_PI;
    }

    const MatrixXd diff_by_diff_t = diff*diff.transpose();
//    std::cout << "i: " << i << "   Xcol.transpose=[" << Xcol.transpose() << "]" << std::endl;
//    std::cout << "i: " << i << "   diff.transpose=[" << diff.transpose() << "]" << std::endl;
//    std::cout << "i: " << i << "   diff*diff_t=[" << std::endl << diff_by_diff_t << "]" << std::endl;
//    for (int c = 0; n_x_ > c; c++)
    {
      P_ += weights_(i) * diff_by_diff_t;
    }
  }
//  std::cout << __FILE__ << ":" << __LINE__ << std::endl;
//  std::cout << "P_ = [" << std::endl << P_ << "]" << std::endl;
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage & meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  const double px = meas_package.raw_measurements_(0);
  const double py = meas_package.raw_measurements_(1);
  VectorXd z(2);
  z << px, py;

//  std::cout << "UpdateLidar: z.transpose() = [" << z.transpose() << "]" << std::endl;

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht_ = H_.transpose();

  MatrixXd PHt = P_ * Ht_;
  MatrixXd S = H_ * PHt + R_lidar_;
  MatrixXd K = PHt * S.inverse();

  // Calculate the NIS
  double eps = y.transpose() * S.inverse() * y;
//  std::cout << "UpdateRadar: eps = [" << eps << "]" << std::endl;
  num_of_lidar_data_++;
  if (nis_limit_lidar_ < eps) {
    nis_lidar_above_limit_++;
  }
//  const double nis_over_limit = double(nis_lidar_above_limit_) / double(num_of_lidar_data_);
//  std::cout << "UpdateLidar: NIS over limit = " << nis_over_limit * 100. << "%" << std::endl;

  //new estimate
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(n_x_, n_x_) - K * H_) * P_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage & meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  // Sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, n_sigma_points_);
  for (int i = 0; n_sigma_points_ > i; i++)
  {
    const double px  = Xsig_pred_(0,i);
    const double py  = Xsig_pred_(1,i);
    const double v   = Xsig_pred_(2,i);
    const double phi = Xsig_pred_(3,i);
    const double rho = sqrt(px*px + py*py);
    Zsig(0,i) = rho;
    Zsig(1,i) = atan2(py, px);
    Zsig(2,i) = (px*cos(phi) + py*sin(phi)) * v / rho;
  }

  // Mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.setZero();
  for (int i = 0; n_sigma_points_ > i; i++)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  S.setZero();
  for (int i = 0; n_sigma_points_ > i; i++)
  {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  S += R_radar_;

  // Cross correlation matrix Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  Tc.setZero();
  for (int i = 0; n_sigma_points_ > i; i++)
  {
      VectorXd x_diff = Xsig_pred_.col(i) - x_;

      while (+M_PI < x_diff(3)) {
        std::cout << +M_PI << " < x_diff(3)=" << x_diff(3) << std::endl;
        x_diff(3) -= 2.0*M_PI;
      }
      while (-M_PI > x_diff(3)) {
        std::cout << -M_PI << " > x_diff(3)=" << x_diff(3) << std::endl;
        x_diff(3) += 2.0*M_PI;
      }

      VectorXd z_diff = Zsig.col(i) - z_pred;
      while (+M_PI < z_diff(1)) {
        std::cout << +M_PI << " < zdiff(1)=" << z_diff(1) << std::endl;
        z_diff(1) -= 2.0*M_PI;
      }
      while (-M_PI > z_diff(1)) {
        std::cout << -M_PI << " > zdiff(1)=" << z_diff(1) << std::endl;
        z_diff(1) += 2.0*M_PI;
      }

      Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  const double rho    = meas_package.raw_measurements_[0];
  const double phi    = meas_package.raw_measurements_[1];
  const double rhodot = meas_package.raw_measurements_[2];
  VectorXd z(n_z_);
  z << rho, phi, rhodot;

  //update state mean and covariance matrix
  VectorXd z_upd = z - z_pred;
  if (+M_PI < z_upd(1)) z_upd(1) -= 2.0*M_PI;
  if (-M_PI > z_upd(1)) z_upd(1) += 2.0*M_PI;

  double eps = z_upd.transpose() * S.inverse() * z_upd;
//  std::cout << "UpdateRadar: eps = [" << eps << "]" << std::endl;
  num_of_radar_data_++;
  if (nis_limit_radar_ < eps) {
    nis_radar_above_limit_++;
  }
//  const double nis_over_limit = double(nis_radar_above_limit_) / double(num_of_radar_data_);
//  std::cout << "UpdateRadar: NIS over limit = " << nis_over_limit * 100. << "%" << std::endl;

  x_ = x_ + K * z_upd;
  P_ = P_ - K * S * K.transpose();
}

/**
 * Return the x vector
 * @return the x vector
 */
const VectorXd UKF::getX() const {
  return x_;
}
