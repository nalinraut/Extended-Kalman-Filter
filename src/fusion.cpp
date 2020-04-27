#include "fusion.h"

Fusion::Fusion() 
{

    isInitialized = false;
    prevTimestamp = 0;

    // initializing matrices
    laserR = Eigen::MatrixXd(2, 2);
    radarR = Eigen::MatrixXd(3, 3);
    laserH = Eigen::MatrixXd(2, 4);
    radarH = Eigen::MatrixXd(3, 4);

    //measurement covariance matrix - laser
    laserR << 0.0225, 0.0000,
              0.0000, 0.0225;

    //measurement covariance matrix - radar
    radarR << 0.0900, 0.0000, 0.0000,
              0.0000, 0.0009, 0.0000,
              0.0000, 0.0000, 0.0900;

    // Measurment Matrix for laser      
    laserH << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;


    //Jacobian measurment for Radar
    radarH << 1.0, 1.0, 0.0, 0.0,
              1.0, 1.0, 0.0, 0.0,
              1.0, 1.0, 1.0, 1.0;           

    //The transition matrix F
    stateTransitionF = Eigen::MatrixXd(4,4);
    stateTransitionF << 1.0, 0.0, 1.0, 0.0,
                        0.0, 1.0, 0.0, 1.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0;

    ekf->setStateTransitionMatrix(stateTransitionF);


    //The covariance matrix P
    covarianceP = Eigen::MatrixXd(4,4);
    covarianceP << 1.0000, 0.00000, 0.0000, 0.0000,
                   0.0000, 1.00000, 0.0000, 0.0000,
                   0.0000, 0.00000, 1000.0, 0.0000,
                   0.0000, 0.00000, 0.0000, 1000.0;

    ekf->setProcessCovariance(covarianceP);


}

Fusion::~Fusion() {}


void Fusion::processMeasurement(const MeasurementPackage &measurementPack) 
{
    if (!isInitialized) 
    {
        //Initializing the state ekf_.x_ with the first measurement.
        std::cout << "EKF: " << std::endl;
        Eigen::VectorXd X = Eigen::VectorXd(4);
        X << 1, 1, 1, 1;
        ekf->setState(X);

        if (measurementPack.sensor_type_ == SensorType::RADAR) 
        {
            //Converting Polar coordinates to cartesian
            // The x = rho.cos(theta) y = rho.sin(theta)
            double rho = measurementPack.raw_measurements_(0);
            double phi = measurementPack.raw_measurements_(1);
            double ro_dot = measurementPack.raw_measurements_(2);
            double x = rho*cos(phi);
            X << rho*cos(phi) , rho*sin(phi) , ro_dot * cos( phi ) , ro_dot * sin( phi );
            ekf->setState(X);
        }
        else if (measurementPack.sensor_type_ == SensorType::LASER) 
        {
            //Laser Update
            X << measurementPack.raw_measurements_(0),measurementPack.raw_measurements_(1),0,0;
            ekf->setState(X);
        }

        prevTimestamp = measurementPack.timestamp_;
        isInitialized = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    //To convert timestamp to from microsecond to second 
    float dt = ( measurementPack.timestamp_ - prevTimestamp) / 1000000.0 ;  
    prevTimestamp = measurementPack.timestamp_;

    /**The state transition matrix should be time dependent.
     * x' = F x + ν		
    */
    stateTransitionF(0,2) = dt;
    stateTransitionF(1,3) = dt;
    ekf->setStateTransitionMatrix(stateTransitionF);

    /**Process Covariance matrix...
     * Q is used for random acceleration vector, 
     * as we don't know the acceleration we add it to the noise part
     * Q is the expectation value of the noise vector and its transpose.
     * Q = G * Qv * G' 
    */			 

    float dt_2 = pow(dt,2);
    float dt_3 = pow(dt,3);
    float dt_4 = pow(dt,4);
    float noise_ax  = 9 , noise_ay = 9;


    Eigen::MatrixXd processNoiseQ = Eigen::MatrixXd(4, 4);
    processNoiseQ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
                     0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
                     dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
                     0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
    ekf->setProcessNoiseMatrix(processNoiseQ);


    //Predicting the new position            
    ekf->predict();


    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     * We have to choose the measurement Function (H) and noise (R) for the update
     * for two types of sensors and then call the update function 
    */

    if (measurementPack.sensor_type_ == SensorType::RADAR) 
    {
        // Radar update..
        Tools tools;
        ekf->setMeasurementModelMatrix(tools.CalculateJacobian(ekf->getState()));
        ekf->setMeasurementNoiseMatrix (radarR);
        ekf->update(measurementPack.raw_measurements_, SensorType::RADAR);
        std::cout<<"Measurement Matrix: "<<ekf->getMeasurementModelMatrix()<<std::endl;
    } 
    else 
    {
        // Laser updates
        ekf->setMeasurementModelMatrix(laserH); 
        ekf->setMeasurementNoiseMatrix(laserR);
        ekf->update(measurementPack.raw_measurements_, SensorType::LASER);
    }

    // print the output
    std::cout << "State x: " << ekf->getState() << std::endl;
    std::cout << "Covariance P: " << ekf->getProcessCovariance() << std::endl;

}