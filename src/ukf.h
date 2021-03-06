#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage & meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(const double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage & meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage & meas_package);

  /**
   * Return the x vector
   * @return the x vector
   */
  const VectorXd getX() const;

private:
  ///* if this is false, laser measurements will be ignored (except for init)
  const bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  const bool use_radar_;

  ///* State dimension
  const int n_x_;

  ///* State dimensionof radar measurement
  const int n_z_;

  ///* Augmented state dimension
  const int n_aug_;

  ///* Sigma point spreading parameter
  const int lambda_;

  ///* The number of sigma points
  const int n_sigma_points_;

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Radar measurement covariance
  MatrixXd R_radar_;

  ///* Lidar measurement covariance
  MatrixXd R_lidar_;

  ///* Lidar measurement matrix
  MatrixXd H_;

  ///* Number of radar data sets
  int num_of_radar_data_;

  ///* NIS limit of radar data
  const double nis_limit_radar_;

  ///* NIS radar over border
  int nis_radar_above_limit_;

  ///* Number of lidar data sets
  int num_of_lidar_data_;

  ///* NIS radar over border
  int nis_lidar_above_limit_;

  ///* NIS limit of lidar data
  const double nis_limit_lidar_;

};

#endif /* UKF_H */
