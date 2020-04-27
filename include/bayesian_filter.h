#ifndef BAYESIAN_FILTER_H_
#define BAYESIAN_FILTER_H_

#include <iostream>

#include "Eigen/Dense"
#include "measurement_package.h"

enum FilterType
{
    EKF,
    UKF
};

class BayesianFilter {
public:
    /**
     * Constructor
     */
    BayesianFilter();
    BayesianFilter(FilterType const);

    /**
     * Destructor
     */
    virtual ~BayesianFilter();

    /**
     * Init Initializes filter
     */
    virtual void initialize();

    /**
     * Getter and setter for State variable mX 
     */ 
    Eigen::VectorXd getState() const;
    void setState(const Eigen::VectorXd state);

    /**
     * Getter and setter for Process Covariance variable mP 
     */ 
    Eigen::MatrixXd getProcessCovariance() const;
    void setProcessCovariance(const Eigen::MatrixXd processCov);
    
    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * Time between k and k+1 in s
     */
    virtual void predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    virtual void update(const Eigen::VectorXd &z, 
                        const SensorType sensorType 
                        );

protected:
    /**
     * Filter type
     */
    FilterType mFilterType;
    
    /** 
     * State vector
     */
    Eigen::VectorXd mX;

    /**
     * State covariance matrix
     */
    Eigen::MatrixXd mP;
};

#endif // BAYESIAN_FILTER_H_