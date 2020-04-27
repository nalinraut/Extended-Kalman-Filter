#include "extended_kalman_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
    this->initialize();
}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){}

 

void ExtendedKalmanFilter::initialize()
{

}

Eigen::MatrixXd ExtendedKalmanFilter::getMeasurementModelMatrix() const
{
    return mH;
}

void ExtendedKalmanFilter::setMeasurementModelMatrix(const Eigen::MatrixXd measurementModelMatrix)
{
    mH = measurementModelMatrix;
}

Eigen::MatrixXd ExtendedKalmanFilter::getStateTransitionMatrix() const
{
    return mF;
}

void ExtendedKalmanFilter::setStateTransitionMatrix(const Eigen::MatrixXd processModelMatrix)
{
    mF = processModelMatrix;
}

Eigen::MatrixXd ExtendedKalmanFilter::getProcessNoiseMatrix() const
{
    return mQ;
}

void ExtendedKalmanFilter::setProcessNoiseMatrix(const Eigen::MatrixXd processNoiseMatrix)
{
    mQ = processNoiseMatrix;
}

Eigen::MatrixXd ExtendedKalmanFilter::getMeasurementNoiseMatrix() const
{
    return mR;
}

void ExtendedKalmanFilter::setMeasurementNoiseMatrix(const Eigen::MatrixXd measurementNoiseMatrix)
{
    mR = measurementNoiseMatrix;
}

void ExtendedKalmanFilter::predict() {
	
	/** 
     * The motion noise is taken as 0.
     * Predict the new state 
     * x = F x + u 
     * P = F P F' + Q  
    */

	mX = mF * mX;
	mP = mF * mP * mF.transpose() + mQ;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd &z,
                                  const SensorType sensorType) {


    Eigen::VectorXd y;

    if (sensorType == SensorType::LASER)
    {
        /** 
         * Update the state using real world measurement 
         * y = z − H x'
         * S = H P H' + R 
         * K = P H' S-1
         * x = x + K y 
         * P = (I − K H ) P 
         */
        y    = z  - mH * mX;
    }

    else
    {
        /** Update the state using real world measurement 
		z' = h(x') + ω
		y = z − z' 	S = H P H' + R 
		K = P H' S-1
		x = x + K y 
		P = (I − K H ) P 
        */

        //Cartesian to polar coordinates	
        double rho = sqrt(pow(mX(0),2) + pow(mX(1),2));
        double phi = atan2(mX(1) , mX(0));
        double rhoDot = 0 ;

        if ( fabs(rho)  > 0.0001)
		    rhoDot = ( mX(0) * mX(2) + mX(1) * mX(3) ) / rho ;
        
        Eigen::VectorXd zPred(3);
        zPred << rho, phi, rhoDot;

        y = z - zPred;
        while ( y(1) > M_PI || y(1) < - M_PI){
		    if( y(1) > M_PI)
			    y(1) -= M_PI;
		    else
			    y(1) += M_PI;	 
	    }
    }

    Eigen::MatrixXd S   = mH * mP * mH.transpose() + mR ;
    Eigen::MatrixXd K   = mP * mH.transpose() * S.inverse(); //Kalman gain
        
    // Update
    mX = mX + K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(mX.size(), mX.size());
    mP = (I - K * mH) * mP;
    
}
