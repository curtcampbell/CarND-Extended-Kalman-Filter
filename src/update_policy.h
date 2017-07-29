#pragma once

#include "kalman_filter.h"

class UpdatePolicy
{
public:
    UpdatePolicy(KalmanFilter& kalmanFilter, float noise_ax, float noise_ay);
    virtual ~UpdatePolicy() {}

    void Predict(float delta_t);
    virtual void Update(const Eigen::VectorXd &z) = 0;

protected:
    KalmanFilter& _ekf;

    // state vector
    Eigen::VectorXd& _x;
    Eigen::MatrixXd& _F;
    Eigen::MatrixXd& _Q;
    Eigen::MatrixXd& _P;

    float _noise_ax;
    float _noise_ay;
};

class LaserUpdatePolicy : public virtual UpdatePolicy
{
public:
    LaserUpdatePolicy(KalmanFilter& kalmanFilter, float noise_ax, float noise_ay);
    virtual ~LaserUpdatePolicy() {}

    virtual void Update(const Eigen::VectorXd &z);

private:
    // laser measurement matrix
    Eigen::MatrixXd _H;

    // laser measurement covariance matrix
    Eigen::MatrixXd _R;

};

class RadarUpdatePolicy : public virtual UpdatePolicy
{
public:
    RadarUpdatePolicy(KalmanFilter& kalmanFilter, float noise_ax, float noise_ay);
    virtual ~RadarUpdatePolicy(){ }

    virtual void Update(const Eigen::VectorXd &z);

private:
    // Radar measurement covariance matrix
    Eigen::MatrixXd _R;
    Eigen::VectorXd MapToPolar(const Eigen::VectorXd &x);
    float NormalizeAngle(float theta);
};

