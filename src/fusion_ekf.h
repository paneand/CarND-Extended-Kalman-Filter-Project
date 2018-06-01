#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_
#include "Eigen/Dense"
#include "fusion_estimator.h"
#include "kalman_filter.h"

using namespace std;


/**
This class inherits from KalmanFilter and FusionEstimator to provide an interface to estimate the position of an object
leveraging on the EKF equations, combining Lidar and Radar measurements. 
*/
class FusionEKF : public KalmanFilter, public FusionEstimator {
public:
  /**
   * Constructor
   */
  FusionEKF();

  /**
   * Destructor
   */
  ~FusionEKF();

  void Init(const MeasurementPackage &measurement_pack);

  void Predict(const MeasurementPackage &measurement_pack);

  void Update(const MeasurementPackage &measurement_pack);
  VectorXd GetEstimate(const MeasurementPackage &measurement_pack);


private:

  void InitFromRadar(const MeasurementPackage &measurement_pack);
  void InitFromLidar(const MeasurementPackage &measurement_pack);
  void InitKFVariables(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
      Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);
  void UpdateMatricesForPrediction(const MeasurementPackage &measurement_pack);
  void UpdateMatricesForUpdate();
  MatrixXd CalculateJacobian(const VectorXd& x_state);
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;
  bool skip_radar_;
  bool skip_laser_;
  bool is_radar_measurement_;   // True means last input measure is radar, false means Lidar measurament
  // previous timestamp
  long long previous_timestamp_;
  float i_vx_, i_vy_, noise_ax_, noise_ay_;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  Eigen::VectorXd x_;   // state vector
  Eigen::MatrixXd P_;   // state covariance matrix
  Eigen::MatrixXd F_;   // state transition matrix
  Eigen::MatrixXd Q_;   // process covariance matrix
  Eigen::MatrixXd H_;   // measurement matrix
  Eigen::MatrixXd R_;   // measurement covariance matrix



};

#endif /* FUSION_EKF_H_ */