#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include "update_policy.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF():
	is_initialized_(false),
	previous_timestamp_(0),
    _laserUpdatePolicy(0),
    _radarUpdatePolicy(0)
{

    //The original comment in the code said to "Pass in noise_ax = 9 and noise_ay = 9 for your Q matrix."
    //In practice however we needed to increase these values. This is because our motion model is linear.
    //Any non-linearities are accounted for as noise.  Increasing these values gave us better results.
    float noise_ax = 50.0f;
    float noise_ay = 50.0f;

    //Both update policies operate on the same reference to the KalmanFilter 
    //class instance which maintains ownership of the state variables and matacies.
    //
    //Q is computed in the Predict() method of the UpdatePolicy base class.
    _laserUpdatePolicy = new LaserUpdatePolicy(_ekf, noise_ax, noise_ay);
    _radarUpdatePolicy = new RadarUpdatePolicy(_ekf, noise_ax, noise_ay);


  //Initialize shared state variables.
  // state vector
  Eigen::VectorXd x(4);
  x << 0, 0, 0, 0;

  // state covariance matrix
  Eigen::MatrixXd P(4,4);
  P << 1000.0f, 0, 0, 0,
       0, 1000.0f, 0, 0,
       0, 0, 1000.0f, 0,
       0, 0, 0, 1000.0f;

  // state transition matrix
  Eigen::MatrixXd F = MatrixXd::Identity(4,4);

  // process covariance matrix
  Eigen::MatrixXd Q(4,4);

  _ekf.Init(x, P, F, Q);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() 
{
    delete _laserUpdatePolicy;
    delete _radarUpdatePolicy;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

    /*****************************************************************************
    *  Initialization
    ****************************************************************************/
  float px, py;

  if (!is_initialized_) {

    // first measurement
    cout << "EKF: " << endl;
    _ekf._x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		//rho
		float rho = measurement_pack.raw_measurements_(0);
		float theta = measurement_pack.raw_measurements_(1);
		float rho_dot = measurement_pack.raw_measurements_(2);

		py = rho * sin(theta);
		px = rho * cos(theta);

        _ekf._P << 1000.0f, 0, 0, 0,
                   0, 1000.0f, 0, 0,
                   0, 0, 10000.0f, 0,
                   0, 0, 0, 10000.0f;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        px = measurement_pack.raw_measurements_(0);
        py = measurement_pack.raw_measurements_(1);

        _ekf._P << 1.0f, 0, 0, 0,
                    0, 1.0f, 0, 0,
                    0, 0, 10000.0f, 0,
                    0, 0, 0, 10000.0f;
	}

    /**
    Initialize state with the first sensor reading.
    */
    _ekf._x << px, py, 0, 0;

	previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
 
  //////////////////////////
  //Prediction
  if (previous_timestamp_ != measurement_pack.timestamp_)
  {
      // Predict a new state only if the timestamp changed.  
      // Same timestamp means we have simultanious sensor readings and don't need to 
      // calculate a new predicted state.
      float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0f;
      previous_timestamp_ = measurement_pack.timestamp_;

      measurement_pack.updatePolicy_->Predict(dt);
  }

  /////////////////////////////
  //Update
  //The concrete type determines that type of update that happens here.
  measurement_pack.updatePolicy_->Update(measurement_pack.raw_measurements_);

  // print the output
  cout << "x_ = " << _ekf._x << endl;
  cout << "P_ = " << _ekf._P << endl;
}
