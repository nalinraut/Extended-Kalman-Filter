#ifndef Fusion_H_
#define Fusion_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include "Eigen/Dense"
#include "extended_kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class Fusion {
public:
    /**
     * Constructor.
     */
    Fusion();

    /**
     * Destructor.
     */
    virtual ~Fusion();

    /**
     * Run the whole flow of the Kalman Filter from here.
     */
    void processMeasurement(const MeasurementPackage &measurementPack);
    /**
     * Kalman Filter update and prediction math lives in here.
     */
    std::shared_ptr<ExtendedKalmanFilter> ekf = std::make_shared<ExtendedKalmanFilter>();

    private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool isInitialized;

    // previous timestamp
    long long prevTimestamp;
    // tool object used to compute Jacobian and RMSE
    Tools tools;
    Eigen::MatrixXd laserR;
    Eigen::MatrixXd radarR;
    Eigen::MatrixXd laserH;
    Eigen::MatrixXd radarH;
    Eigen::MatrixXd stateTransitionF;
    Eigen::MatrixXd covarianceP;
};

#endif // Fusion_H_
