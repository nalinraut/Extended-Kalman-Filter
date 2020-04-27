#include "bayesian_filter.h"

BayesianFilter::BayesianFilter(){}

BayesianFilter::BayesianFilter(const FilterType filterType)
:mFilterType(filterType)
{}

BayesianFilter::~BayesianFilter(){}

void BayesianFilter::initialize(){}

Eigen::VectorXd BayesianFilter::getState() const
{
    return mX;
}

void BayesianFilter::setState(const Eigen::VectorXd state)
{
    mX = state;
}

Eigen::MatrixXd BayesianFilter::getProcessCovariance() const
{
    return mP;
}

void BayesianFilter::setProcessCovariance(const Eigen::MatrixXd processCov)
{
    mP = processCov;
}

void BayesianFilter::predict()
{}

void BayesianFilter::update(const Eigen::VectorXd &z, 
                    const SensorType sensorType)
{}

