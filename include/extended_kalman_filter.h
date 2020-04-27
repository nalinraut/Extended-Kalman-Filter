#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "bayesian_filter.h"

class ExtendedKalmanFilter : public BayesianFilter
{
public:
    /**
     * Constructor
     */
    ExtendedKalmanFilter();

    /**
     * Destructor
     */
    virtual ~ExtendedKalmanFilter();

    /**
     * Getter and Setter for Measurement model matrix
     */
    Eigen::MatrixXd getMeasurementModelMatrix() const;
    void setMeasurementModelMatrix(const Eigen::MatrixXd measurementModelMatrix);
    
    /**
     * Getter and Setter for Process model matrix
     */
    Eigen::MatrixXd getStateTransitionMatrix() const;
    void setStateTransitionMatrix(const Eigen::MatrixXd processModelMatrix);
    

    /**
     * Getter and Setter for Process noise matrix
     */
    Eigen::MatrixXd getProcessNoiseMatrix() const;
    void setProcessNoiseMatrix(const Eigen::MatrixXd processNoiseMatrix);
    
    /**
     * Getter and Setter for Measurement noise matrix
     */
    Eigen::MatrixXd getMeasurementNoiseMatrix() const;
    void setMeasurementNoiseMatrix(const Eigen::MatrixXd measurementNoiseMatrix);
    
    /**
     * Init Initializes filter
     */
    void initialize();

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param timeStep Time between k and k+1 in s
     */
    void predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void update(const Eigen::VectorXd &z,
                const SensorType sensorType);

private:

    /**
     * State transition matrix
     */ 
    Eigen::MatrixXd mF;

    /**
     * Process noise matrix
     */ 
    Eigen::MatrixXd mQ;

    /**
     * Measurement matrix
     */ 
    Eigen::MatrixXd mH;

    /** 
     * Measurement noise matrix
     */ 
    Eigen::MatrixXd mR;
};

#endif // KALMAN_FILTER_H_
