#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

//set state dimension
  n_x_ = 5;

  //set augmented dimension
  n_aug_ = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  n_z_ = 3;
//set measurement dimension, lidar can measure px and py
  n_z_lidar_ = 2;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.fill(0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.9; //0.09; //0.1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.6;

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
    Complete the initialization. See UKF.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;

  previous_timestamp_ = 0;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    /**
      * Initialize the state UKF_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //std::cout << "UKF: " << std::endl;
    x_ = VectorXd(5);
    x_ << 1, 1, 1, 1, 1;

    // initialize the state covariance matrix P_
    P_ = MatrixXd(5,5);
    P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

	
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
	//std::cout << "RADAR" << std::endl;
      /*
      Convert radar from polar to cartesian coordinates and initialize state.
      */
	//x_ is a 1x4 matrix of px, py, vx, and vy
	//Radar measurement is a 1x3 matrix of rho, phi, 
	//and rho dot 
	float rho = meas_package.raw_measurements_[0];
	float phi = meas_package.raw_measurements_[1];
	float rhoDot = meas_package.raw_measurements_[2];
	//set the state with the initial location and zero velocity
	float px = rho * cos(phi); //sqrt((rho*rho) / (tan(phi)*tan(phi) + 1));
	float py = rho * sin(phi); //tan(phi) * px;
	
	x_ <<  px, py, 0, 0, 0; //8.6, 0.25, -3.00029, 0; //rho * cos(phi), rho * sin(phi), sqrt(8.6*8.6+0.25*0.25)*cos(phi), sqrt(8.6*8.6+0.25*0.25)*sin(phi); //rho * cos(phi), rho * sin(phi), 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
	std::cout << "Laser" << std::endl;
	//set the state with the initial location and zero velocity
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
	// Check if px, py are very small
	
	x_ << px, py, 0, 0, 0;
	std::cout << "Laser: " << std::endl;
	
    }


	previous_timestamp_ = meas_package.timestamp_;
	

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
/***************************************************************
   *  Prediction
   ***************************************************************/

  	//compute the time elapsed between the current and previous measurements
	float delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//delta_t - expressed in seconds
     
	previous_timestamp_ = meas_package.timestamp_;

	//3. Call the Kalman Filter predict() function
	MatrixXd Xsig_pred_ = MatrixXd(15, 5);
  	Prediction(&Xsig_pred_, delta_t);
     
/***************************************************************
   *  Update
   ***************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	//4. Call the Kalman Filter update() function
	// with the most recent raw measurements_
	std::cout << "Radar" << std::endl;

     
     VectorXd measurements_ = VectorXd(3);
	measurements_ << meas_package.raw_measurements_[0], 
		meas_package.raw_measurements_[1], 
		meas_package.raw_measurements_[2];
     

  	UpdateRadar(Xsig_pred_, measurements_); 

  } else {
    // Laser updates
	std::cout << "Laser" << std::endl;

	//4. Call the Kalman Filter update() function
	// with the most recent raw measurements_
     
     VectorXd measurements_ = VectorXd(2);
	measurements_ << meas_package.raw_measurements_[0], 
		meas_package.raw_measurements_[1];
     
	UpdateLidar(Xsig_pred_, measurements_);
  }

	/*std::cout << "After update, x_: " << std::endl << x_ << std::endl;
    std::cout << "P_:  " << std::endl << P_ << std::endl;
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(MatrixXd* Xsig_out, double delta_t) {

  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
*/
/* *********************************** */
/* Prediction steps: 1)generate sigma points, 2)predict sigma points, and 3)predict mean and covariance matrix. */
/* ******************************* */
// 1a) generate sigma points
  //MatrixXd Xsig = MatrixXd(11, 5);
    // GenerateSigmaPoints(&Xsig) ;

// 1b) Generate augmented sigma points
    MatrixXd Xsig_aug = MatrixXd(15, 7);
    AugmentedSigmaPoints(&Xsig_aug);
// 2) predicted sigma points
    MatrixXd Xsig_pred_ = MatrixXd(15, 5);
    SigmaPointPrediction(&Xsig_pred_, delta_t, Xsig_aug);
// 3) predict mean and covariance matrix
    PredictMeanAndCovariance(Xsig_pred_); 

*Xsig_out = Xsig_pred_;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MatrixXd Xsig_pred_, VectorXd z) { 
  /**

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  /**
     * update the state by using Kalman Filter equations
  */
/*
	 * KF Measurement update step
	 */
/*
	//measurement covariance matrix - laser
	R_laser_ = MatrixXd(2,2);
  	R_laser_ << std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;
	
	MatrixXd R_ = R_laser_;
	H_ = MatrixXd(2,5);
	//Lidar H matrix
	H_ << 1,0,0,0,0,
		0,1,0,0,0;
	
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_laser_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

	//calculate NIS Lidar
	NIS_laser_ = y.transpose()*S.inverse()*y; */
	
	VectorXd z_out = VectorXd(3);
  	MatrixXd S_out = MatrixXd(3, 3);
  	MatrixXd Zsig = MatrixXd(3, 15);
  	PredictLidarMeasurement(&z_out, &S_out, &Zsig, Xsig_pred_);
  	UpdateLidarState(NIS_laser_, Xsig_pred_, z_out, S_out, Zsig, z);


  }

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MatrixXd Xsig_pred_, VectorXd measurements_) { 
	/*std::cout << "Before update Radar, x_: " << std::endl << x_ << std::endl;
    std::cout << "P_:  " << std::endl << P_ << std::endl;
*/
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  VectorXd z_out = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  MatrixXd Zsig = MatrixXd(3, 15);
  PredictRadarMeasurement(&z_out, &S_out, &Zsig, Xsig_pred_);
  
  UpdateRadarState(NIS_radar_, Xsig_pred_, z_out, S_out, Zsig, measurements_);
  

}

//Generate Sigma points
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out) {
  //define spreading parameter
  lambda_ = 3 - n_x_;

  //set example state
  //VectorXd x_ = VectorXd(n_x_); 

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();
  
  //calculate sigma points ...
  //set sigma points as columns of matrix Xsig
  //A = sqrt(pK|k)
  //Xsig = [xk|k xk|k+sqrt((lambda_+n_x_))*A xk|k-sqrt((lambda_+n_x_))*A]
  //Xsig << x, x+(lambda_+n_x_).llt.matrixL()*A, x-(lambda_+n_x_).llt().matrixL()*A;
  MatrixXd sumSig = MatrixXd(n_x_, n_x_);
  MatrixXd subSig = MatrixXd(n_x_, n_x_);
  for (int i = 0; i < n_x_; i++) {
    sumSig.col(i) = x_.col(0) + (sqrt(lambda_+n_x_)*A).col(i);
   subSig.col(i) = x_.col(0) - (sqrt(lambda_+n_x_)*A).col(i);
  }
  
  Xsig << x_, sumSig, subSig;

  //write result
  *Xsig_out = Xsig;

}

//Generate augmented sigma points
void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {
  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set example state
  //VectorXd x_ = VectorXd(n_x_);
  
  //create example covariance matrix
  //MatrixXd P_ = MatrixXd(n_x_, n_x_);
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  //create augmented mean state
  //x_aug << x, 0, 0; //same as code below
  //x.head(x) = y, where x is the number of elements from first element, and y is an input matrix of that size.
   x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  //create augmented covariance matrix
  P_aug.fill(0.0);
  //set matrix y to top left corner of matrix x.
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
   MatrixXd L = P_aug.llt().matrixL();
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < 7; i++) {
      Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)*L.col(i);
      Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_)*L.col(i);    
  }
  
  //write result
  *Xsig_out = Xsig_aug;
}

//Sigma point prediction
void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, double delta_t, MatrixXd Xsig_aug) {
  //create example sigma point matrix
  /*MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);*/
  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);


  //predict sigma points
  for (int i = 0; i < 2*n_aug_+1; i++) {
      double p_x =Xsig_aug(0,i); //px
      double p_y =Xsig_aug(1,i); //py
      double v =Xsig_aug(2,i); //v
      double yaw =Xsig_aug(3,i); //yaw
      double yawd =Xsig_aug(4,i); //yaw dot
      double nu_a =Xsig_aug(5,i); //va = longitudinal acceleration mua
      double nu_yawdd =Xsig_aug(6,i); //vyawdotdot = yaw acceleration noise mua
      
      //Predicted state values
      double px_p, py_p;
      
  //avoid division by zero
      if (fabs(yawd) > 0.001) {
          px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
          py_p = p_y + v/yawd * (-1 * cos(yaw+yawd*delta_t) + cos(yaw));
	    
      }
      else {
          px_p = p_x + v * cos(yaw) * delta_t;
          py_p = p_y + v * sin(yaw) * delta_t;
      }
      px_p = px_p + 0.5*delta_t*delta_t*cos(yaw)*nu_a;
      py_p = py_p + 0.5*delta_t*delta_t*sin(yaw)*nu_a;
      double v_p = v + 0 + delta_t*nu_a;
      double yaw_p = yaw + yawd * delta_t +0.5 * delta_t*delta_t*nu_yawdd;
      double yawd_p = yawd + 0 + delta_t * nu_yawdd;
      
  //write predicted sigma points into right column
      Xsig_pred_(0,i)= px_p;
      Xsig_pred_(1,i)= py_p;
      Xsig_pred_(2,i)= v_p;
      Xsig_pred_(3,i)= yaw_p;
      Xsig_pred_(4,i)=yawd_p;
  }

  
  //write result
  *Xsig_out = Xsig_pred_;

}

//Predict the mean and covariance
void UKF::PredictMeanAndCovariance(MatrixXd Xsig_pred_) {
  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //create example matrix with predicted sigma points
  //  MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  
  //create vector for predicted state
  //VectorXd x_ = VectorXd(n_x_);

  //create covariance matrix for prediction
  //MatrixXd P_ = MatrixXd(n_x_, n_x_);




//set weights
  for (int i=0; i < 2*n_aug_+1; i++) {
    if (i == 0)
        weights(i) = lambda_/(lambda_ + n_aug_);
     else 
        weights(i) = 1/(2*(lambda_ + n_aug_));
  }
  
  //predict state mean
  x_.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
    //iterate over sigma points
    x_ = x_ + weights(i) * Xsig_pred_.col(i);
    
  }
  //predict state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_+1; i++) {
    //iterate over sigma points
    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
    P_ = P_ + weights(i)*x_diff*x_diff.transpose();
   
    }

}

//update subroutine
void UKF::UpdateRadarState(double NIS_out, MatrixXd Xsig_pred_,  VectorXd z_out, MatrixXd S_out, MatrixXd Zsig, VectorXd measurements_) {
  
  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //create example matrix with predicted sigma points in state space
 /* MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);*/
  //create example vector for predicted state mean
  /*VectorXd x_ = VectorXd(n_x_);*/
//x_ = x_pred_; //from predicted state mean

  //create example matrix for predicted state covariance
  //MatrixXd P_ = MatrixXd(n_x_,n_x_);
   // P_ is from predicted covariance matrix

  //create example matrix with sigma points in measurement space
  //MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  
  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  
  z_pred = z_out;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z_,n_z_);
  
  S = S_out;

  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z_);
  
  z = measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);


  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
      //state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      //angle normalization
      while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
      
      //residual difference
      VectorXd z_diff = Zsig.col(i) - z_pred;
      //angle normalization
      while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
      while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
      
      Tc = Tc + weights(i)*x_diff*z_diff.transpose();
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean
  x_ = x_ + K * z_diff;
  //update state covariance matrix
  P_ = P_ - K * S * K.transpose();
  
  //update the Normalized Innovation Squared (NIS)
  NIS_out = z_diff.transpose() * S * z_diff;

}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out, MatrixXd  Xsig_pred_) {

  //define spreading parameter
  lambda_ = 3 - n_aug_;
  
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
   double weight_0 = lambda_/(lambda_+n_aug_);

   weights(0) = weight_0;

  for (int i=1; i < (2*n_aug_+1); i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  
  //create example matrix with predicted sigma points
 /* MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);*/
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_,n_z_);

  MatrixXd px = Xsig_pred_.row(0);
  MatrixXd py = Xsig_pred_.row(1);
  MatrixXd v = Xsig_pred_.row(2);
  MatrixXd yaw = Xsig_pred_.row(3);
  MatrixXd yawd = Xsig_pred_.row(4);
  
  Zsig.fill(0.0);

  for (int i = 0; i < 2*n_aug_+1; i++) {
    Zsig(0,i) = sqrt(px(i)*px(i) + py(i)*py(i));
    Zsig(1, i) = atan2(py(i),px(i));
    //avoid division by zero
    if (fabs(Zsig(0,i)) > 0.001) {
		Zsig(2, i) = (px(i) * cos(yaw(i)) * v(i) + py(i) * sin(yaw(i)) * v(i))/Zsig(0, i);
		}
    else {
		Zsig(2, i) = 0;
		}

    
  }
  

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
      //state difference
      VectorXd z_diff = Zsig.col(i) - z_pred;

      //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      S = S + weights(i) * z_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z_,n_z_);
  R.fill(0.0);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  S = S + R;

   //write result
   *z_out = z_pred;
   *S_out = S;
   *Zsig_out = Zsig;
}

/*************************************/
//update subroutine
void UKF::UpdateLidarState(double NIS_out, MatrixXd Xsig_pred_,  VectorXd z_out, MatrixXd S_out, MatrixXd Zsig, VectorXd measurements_) {
  
  //define spreading parameter
  lambda_ = 3 - n_aug_;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //create example matrix with predicted sigma points in state space
 /* MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);*/
  //create example vector for predicted state mean
  /*VectorXd x_ = VectorXd(n_x_);*/
//x_ = x_pred_; //from predicted state mean

  //create example matrix for predicted state covariance
  //MatrixXd P_ = MatrixXd(n_x_,n_x_);
   // P_ is from predicted covariance matrix

  //create example matrix with sigma points in measurement space
  //MatrixXd Zsig = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);
  
  //create example vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);
  
  z_pred = z_out;

  //create example matrix for predicted measurement covariance
  MatrixXd S = MatrixXd(n_z_lidar_,n_z_lidar_);
  
  S = S_out;

  //create example vector for incoming Lidar measurement
  VectorXd z = VectorXd(n_z_lidar_);
  
  z = measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_lidar_);


  //calculate cross correlation matrix
  Tc.fill(0.0);
  
  for (int i = 0; i < 2*n_aug_+1; i++) {
      //state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      //angle normalization
      while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
      while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
      
      //residual difference
      VectorXd z_diff = Zsig.col(i) - z_pred;
      //angle normalization
      while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
      while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
      
      Tc = Tc + weights(i)*x_diff*z_diff.transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean
  x_ = x_ + K * z_diff;
  //update state covariance matrix
  P_ = P_ - K * S * K.transpose();

  //update the Normalized Innovation Squared (NIS)
  NIS_out = z_diff.transpose() * S * z_diff;

}

void UKF::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* Zsig_out, MatrixXd  Xsig_pred_) {

  //define spreading parameter
  lambda_ = 3 - n_aug_;
  
  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
   double weight_0 = lambda_/(lambda_+n_aug_);
   weights(0) = weight_0;

  for (int i=1; i < (2*n_aug_+1); i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //create example matrix with predicted sigma points
 /* MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);*/
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_lidar_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_lidar_);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_lidar_,n_z_lidar_);
  Zsig.fill(0.0);

    Zsig.row(0) = Xsig_pred_.row(0);
    Zsig.row(1) = Xsig_pred_.row(1);
  

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
  }
  //calculate measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; i++) {
      //state difference
      VectorXd z_diff = Zsig.col(i) - z_pred;

      //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

      S = S + weights(i) * z_diff * z_diff.transpose();
  }
  MatrixXd R = MatrixXd(n_z_lidar_,n_z_lidar_);
  R.fill(0.0);
  R << std_laspx_*std_laspx_, 0,
0, std_laspy_*std_laspy_;

  S = S + R;

   //write result
   *z_out = z_pred;
   *S_out = S;
   *Zsig_out = Zsig;
}
