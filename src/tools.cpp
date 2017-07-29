#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

const double Tools::Pi = 2 * acos(0.0);

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//compute the Jacobian matrix
	float m1 = px*px + py*py;
	float m1_sqrt = sqrt(m1);
	float m2 = px * vx + py * vy;
	float m3 = pow(m1, 3 / 2);
	float i = (vx*py - vy * px) / pow(m1, 3 / 2);
	float m31 = py*(vx*py - vy * px) / m3;
	float m32 = px*(vy*px - vx*py) / m3;

	//check division by zero
    if (fabs(m1) < 0.0001) {
        cout << "CalculateJacobian () - Error - Division by Zero" << endl;
        return Hj;
    }

	Hj << px / m1_sqrt, py / m1_sqrt,            0,            0,
		      -py / m1,      px / m1,            0,            0,
		           m31,          m32, px / m1_sqrt, py / m1_sqrt;

    return Hj;
}
