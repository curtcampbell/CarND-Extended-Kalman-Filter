#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class UpdatePolicy;

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter state variables live here.
  */
  KalmanFilter _ekf;

  //Returns an object used to update state variables when there is a Laser reading
  UpdatePolicy* LaserUpdate() { return _laserUpdatePolicy; }

  //Returns an object used to update state variables when there is a Radar reading
  UpdatePolicy* RadarUpdate() { return _radarUpdatePolicy; }

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  UpdatePolicy* _laserUpdatePolicy;
  UpdatePolicy* _radarUpdatePolicy;
};

#endif /* FusionEKF_H_ */
