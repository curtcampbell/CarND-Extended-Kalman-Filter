#include "update_policy.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UpdatePolicy::UpdatePolicy(KalmanFilter& kalmanFilter, float noise_ax, float noise_ay):
    _ekf(kalmanFilter),
    _x(kalmanFilter._x),
    _F(kalmanFilter._F),
    _Q(kalmanFilter._Q),
    _P(kalmanFilter._P),
    _noise_ax(noise_ax),
    _noise_ay(noise_ay)
{
}

void UpdatePolicy::Predict(float delta_t) {    
    // Update the transition matrix with the elapsed time.
    _F(0, 2) = delta_t;
    _F(1, 3) = delta_t;

    // Update the process covariance matrix
    float dt_4 = pow(delta_t, 4) / 4.0f;
    float dt_3 = pow(delta_t, 3) / 2.0f;
    float dt_2 = pow(delta_t, 2);
    
    _Q << dt_4 * _noise_ax, 0, dt_3 * _noise_ax, 0,
        0, dt_4 * _noise_ay, 0, dt_3 * _noise_ay,
        dt_3 * _noise_ax, 0, dt_2 * _noise_ax, 0,
        0, dt_3 * _noise_ay, 0, dt_2 * _noise_ay;

    //Predict the new state.
    _x = _F * _x;
    _P = _F * _P * _F.transpose() + _Q;
}


LaserUpdatePolicy::LaserUpdatePolicy(KalmanFilter& kalmanFilter, float noise_ax, float noise_ay):
    UpdatePolicy(kalmanFilter, noise_ax, noise_ay),
    _H(2, 4),
    _R(MatrixXd(2,2))
{
    //Setup the laser measurement matrix
    _H << 1, 0, 0, 0,
          0, 1, 0, 0;

    //Setup the measurement covariance matrix
    _R << 0.0225, 0,
          0, 0.0225;
}

void LaserUpdatePolicy::Update(const Eigen::VectorXd &z)
{
    //Update the state by using Kalman Filter equations

    VectorXd z_pred = _H * _x;
    VectorXd y = z - z_pred;
    MatrixXd Ht = _H.transpose();
    MatrixXd S = _H * _P * Ht + _R;
    MatrixXd K = _P * Ht * S.inverse();

    //new estimate
    _x = _x + (K * y);

    long x_size = _x.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    _P = (I - K * _H) * _P;
}


RadarUpdatePolicy::RadarUpdatePolicy(KalmanFilter & kalmanFilter, float noise_ax, float noise_ay):
    UpdatePolicy(kalmanFilter, noise_ax, noise_ay),
    _R(MatrixXd(3, 3))
{
    //measurement covariance matrix - radar
    _R << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
}

void RadarUpdatePolicy::Update(const Eigen::VectorXd & z)
{
    //update the state by using Extended Kalman Filter equations

    VectorXd z_pred = MapToPolar(_x);
    VectorXd y = z - z_pred;

    y(1) = NormalizeAngle(y(1));

    MatrixXd Hj = Tools::CalculateJacobian(_x);
    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj * _P * Hjt + _R;
    MatrixXd K = _P * Hjt *  S.inverse();

    _x = _x + (K * y);

    long x_size = _x.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    _P = (I - K * Hj) * _P;
}

Eigen::VectorXd RadarUpdatePolicy::MapToPolar(const Eigen::VectorXd & x)
{
    float px = x(0),
        py = x(1),
        vx = x(2),
        vy = x(3),
        rho = sqrt(px*px + py*py);

    VectorXd p(3);
    p << rho,
        atan2(py, px),
        (px*vx + py*vy) / rho;

    return p;
}

float RadarUpdatePolicy::NormalizeAngle(float theta)
{
    if (theta > Tools::Pi)
    {
        theta -= Tools::Pi * 2.0f;
    }
    else if (theta < -Tools::Pi)
    {
        theta += Tools::Pi * 2.0f;
    }
    return theta;
}


