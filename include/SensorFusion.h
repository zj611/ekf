//
// Created by Zhangjun on 2020/2/19.
//

#ifndef EKF_SENSORFUSION_H
#define EKF_SENSORFUSION_H

#include <Eigen>
#include "DataPackage.h"
#include "ExtendedKalmanFilter.h"

class SensorFusion {
public:
    SensorFusion();
    ~SensorFusion(){};
    void Process(MeasurementPackage &measurementPackage);

    ExtendedKalmanFilter kf;

private:
    bool is_initialized;
    long long int last_timestamp;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_lidar_;
};


#endif //EKF_SENSORFUSION_H
