//
// Created by Zhangjun on 2020/2/19.
//

#ifndef EKF_DATAPACKAGE_H
#define EKF_DATAPACKAGE_H

#include <Eigen>

class GroundTruthPackage {
public:
    long long int timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd gt_values_;

};

class MeasurementPackage {
public:
    long long int timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;
};

#endif //EKF_DATAPACKAGE_H
