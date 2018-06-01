#include "fusion_ekf.h"
#include <iostream>
#include <cmath>
#include <limits>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

FusionEKF::FusionEKF(){

  skip_radar_ = false;
  skip_laser_ = false;
  is_initialized_ = false;    // We are creating the intance of FusionEKF, but the filter has not yet been initialized
  previous_timestamp_ = 0;
  R_ =  MatrixXd(3,3);        // Measurement covariance matrix, it will be equal to R_laser or R_radar depending on the measurement at each step
  H_ =  MatrixXd(3,4);        // Linear matrix to convert from state space to measurmement space, it will be equal to H_laser or Hj depending on the measurement at each step
  x_ = VectorXd(4);           // State space: (px,py,vx,vy)
  F_ = MatrixXd(4,4);         // Transition matrix for the prediction step
  P_ = MatrixXd(4,4);         // State covariance matrix. It represents the uncertainty of our current belief of the state.
  Q_ = MatrixXd(4,4);         // Process covariance matrix. It represents the uncertainty of the movement of the object in our environment. It is updated at each step
  R_laser_ = MatrixXd(2,2);   // Laser covariance matrix. It represents the uncertainty of the sensor. This values can be found in the sensor manual
  R_radar_ = MatrixXd(3,3);   // Radar covariance matrix. It represents the uncertainty of the sensor. This values can be found in the sensor manual
  H_laser_ = MatrixXd(2,4);   // Laser measurement function is linear, thus this is constant.
  Hj_ = MatrixXd(3,4);        // This is a linearization of radar h(x) which depend on x. As such will vary at each update step.  
  
  // Init values
  R_laser_ << 0.0225, 0,      
        0, 0.0225;
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  H_laser_ << 1, 0, 0, 0,
                 0, 1, 0, 0;
  Hj_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0; 
  F_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
  P_ << 1, 0, 0, 0, 
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
  Q_ <<  0, 0, 0, 0,  
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  
  // Process model noise
  noise_ax_ = 9;
  noise_ay_ = 9;

  // These are taken by the input dataset. Correct initial values for the filter can significantly reduce the RMSE
  i_vx_ = 5.199937;
  i_vy_ = 0;
}

FusionEKF::~FusionEKF(){}


void FusionEKF::Init(const MeasurementPackage &measurement_pack){
    if (is_radar_measurement_) {
      InitFromRadar(measurement_pack);
    }
    else {
      InitFromLidar(measurement_pack);
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
}


void FusionEKF::InitFromRadar(const MeasurementPackage &measurement_pack){
  VectorXd x = VectorXd(4);
  float rho,phi,px,py;
  cout << "First measurement = radar" << endl;
  rho = measurement_pack.raw_measurements_[0];
  phi = measurement_pack.raw_measurements_[1];
  px = rho*cos(phi);
  py = rho*sin(phi);
  x << px, py, i_vx_, i_vy_;
  Hj_ = CalculateJacobian(x);
  InitKFVariables(x,P_,F_,Hj_,R_radar_,Q_);
}
void FusionEKF::InitFromLidar(const MeasurementPackage &measurement_pack){
  VectorXd x = VectorXd(4);
  float px,py;
  cout << "First measurement = laser" << endl;
  px = measurement_pack.raw_measurements_[0];
  py = measurement_pack.raw_measurements_[1];
  x << px, py, i_vx_, i_vy_;
  InitKFVariables(x,P_,F_,H_laser_,R_laser_,Q_);
}


void FusionEKF::InitKFVariables(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in){
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}


MatrixXd FusionEKF::CalculateJacobian(const VectorXd& x_state){
  MatrixXd Hj(3,4);
  float px,py,vx,vy,c1,c2,c3;
  // recover state parameters
  px = x_state(0);
  py = x_state(1);
  vx = x_state(2);
  vy = x_state(3);
  // compute a set of terms to avoid repeated calculation
  c1 = px*px+py*py;
  c2 = sqrt(c1);
  c3 = (c1*c2);

  //check division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }
  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
  
}

void FusionEKF::UpdateMatricesForPrediction(const MeasurementPackage &measurement_pack){
  float dt,dt_2,dt_3,dt_4;
  
  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;     //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  
  
  dt_2 = dt * dt;
  dt_3 = dt_2 * dt;
  dt_4 = dt_3 * dt;


  //Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  
  //set the process covariance matrix Q
  Q_ <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
         0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
         dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
         0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;
  return;
}

void FusionEKF::UpdateMatricesForUpdate(){
  if(is_radar_measurement_){
    Hj_ = CalculateJacobian(x_);
    H_ = Hj_;
    R_ = R_radar_;
  }
  else{
    H_ = H_laser_;
    R_ = R_laser_;
  }
}
/**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
void FusionEKF::Predict(const MeasurementPackage &measurement_pack){
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
void FusionEKF::Update(const MeasurementPackage &measurement_pack){
  VectorXd y,z,z_pred;
  if(is_radar_measurement_){      // We are using EKF, Predict the measurement using non linear function 
      cout << "UpdateRadar" << endl;
      float px, py, vx, vy, rho, phi, rhodot;
      z = VectorXd(3);
      z_pred = VectorXd(3);
      z << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1],measurement_pack.raw_measurements_[2];
      px = x_(0);
      py = x_(1);
      vx = x_(2);
      vy = x_(3);
      rho = sqrt(px*px+py*py);
      phi = atan2f(py,px);
      rhodot = (px*vx+py*vy)/rho;
      if(fabs(rho) < 0.0001){
        std::cout << "Rho very close to zero - Error - Division by Zero - Skip UpdateEKF" << endl;
        return;
      }
      z_pred << rho, phi, rhodot;
      y = z - z_pred;
      // EKF filter works with small angles, given the linearization. Thus, we have to normalize phi, to be included between -PI and PI
      y(1) = y(1) > M_PI ? (y(1)-2*M_PI) : (y(1) < - M_PI ? y(1) + 2*M_PI : y(1)); 
    }
    else{                 // Laser. Standard KF equation. 
      cout << "UpdateLaser" << endl;
      z = VectorXd(2);
      z_pred = VectorXd(2);
      z << measurement_pack.raw_measurements_[0],measurement_pack.raw_measurements_[1];
      z_pred = H_ * x_;
      y = z - z_pred;
    }

    // This equations are common to both EKF and KF. In case of Radar measurement, H_ and R_ are already properly set. 
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
  }


VectorXd FusionEKF::GetEstimate(const MeasurementPackage &measurement_pack){
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      is_radar_measurement_ = true;
      if(skip_radar_){
        VectorXd special(1);
        special(0) = std::numeric_limits<float>::max();   // escape value
        return special;
      }
    }
  else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      is_radar_measurement_ = false;
      if(skip_laser_){
        VectorXd special(1);
        special(0) = std::numeric_limits<float>::max();   // escape value
        return special;
      }
    }
  if (!is_initialized_) {
    Init(measurement_pack);
    return x_;
  }
  UpdateMatricesForPrediction(measurement_pack);
  Predict(measurement_pack);
  UpdateMatricesForUpdate();
  Update(measurement_pack);
  // print the output
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
  return x_;
  }