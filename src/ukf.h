#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;
  
  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement matrix
  Eigen::MatrixXd H_;

  // measurement covariance matrix
  Eigen::MatrixXd R_;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;

  //create sigma point matrix
  MatrixXd Xsig;
  // create augmented sigma point matrix
  MatrixXd Xsig_aug;

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

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;
 ///* Radar Measurement state dimension
  int n_z_;
 ///* Lidar Measurement state dimension
  int n_z_lidar_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

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
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(MatrixXd* Xsig_out, double delta_t);
/*MatrixXd* Xsig_pred_,  VectorXd* x_pred,
    MatrixXd* P_pred, double delta_t);*/

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MatrixXd Xsig_pred_, VectorXd measurements_);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MatrixXd Xsig_pred_, VectorXd measurements_);
  

    void GenerateSigmaPoints(MatrixXd* Xsig_out);

    void AugmentedSigmaPoints(MatrixXd* Xsig_out);

    void SigmaPointPrediction(MatrixXd* Xsig_pred_, double delta_t, MatrixXd Xsig_aug);

    void PredictMeanAndCovariance(MatrixXd Xsig_pred);

  void PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig, MatrixXd  Xsig_pred_);

  void UpdateRadarState(double NIS_out, MatrixXd Xsig_pred_,  VectorXd z_out, MatrixXd S_out, MatrixXd Zsig, VectorXd measurements_);

void PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig, MatrixXd  Xsig_pred_);

  void UpdateLidarState(double NIS_out, MatrixXd Xsig_pred_,  VectorXd z_out, MatrixXd S_out, MatrixXd Zsig, VectorXd measurements_);

};
#endif /* UKF_H */
