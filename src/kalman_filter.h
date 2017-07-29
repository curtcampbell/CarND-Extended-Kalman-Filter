#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

struct KalmanFilter {

  // state vector
  Eigen::VectorXd _x;

  // state covariance matrix
  Eigen::MatrixXd _P;

  // state transition matrix
  Eigen::MatrixXd _F;

  // process covariance matrix
  Eigen::MatrixXd _Q;

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param Q_in Process covariance matrix
   */
  void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
            Eigen::MatrixXd &Q_in);
};

#endif /* KALMAN_FILTER_H_ */
