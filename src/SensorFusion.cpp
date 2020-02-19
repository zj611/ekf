//
// Created by Zhangjun on 2020/2/19.
//

#include "SensorFusion.h"

SensorFusion::SensorFusion() {
    is_initialized = false;
    last_timestamp = 0.0;

    //初始化激光雷达的测量矩阵
    H_lidar_ = Eigen::MatrixXd(2, 4);
    H_lidar_ << 1,0,0,0,
            0,1,0,0;
    // 设置传感器的测量噪声矩阵，一般由传感器厂商提供，如未提供，也可通过有经验的工程师调试得到
    R_lidar_ = Eigen::MatrixXd(2, 2);
    R_lidar_ << 0.0225, 0,
            0,      0.0225;
    R_radar_ = Eigen::MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;
}

void SensorFusion::Process(MeasurementPackage &measurementPackage)
{
    // 第一帧数据用于初始化 Kalman 滤波器
    if (!is_initialized) {
        Eigen::Vector4d x;
        if (measurementPackage.sensor_type_ == MeasurementPackage::LASER) {
            // 如果第一帧数据是激光雷达数据，没有速度信息，因此初始化时只能传入位置，速度设置为0
            x << measurementPackage.raw_measurements_[0], measurementPackage.raw_measurements_[1], 0, 0;
        } else if (measurementPackage.sensor_type_ == MeasurementPackage::RADAR) {
            // 如果第一帧数据是毫米波雷达，可以通过三角函数算出x-y坐标系下的位置和速度
            float rho = measurementPackage.raw_measurements_[0];
            float phi = measurementPackage.raw_measurements_[1];
            float rho_dot = measurementPackage.raw_measurements_[2];
            float position_x = rho * cos(phi);
            if (position_x < 0.0001) {
                position_x = 0.0001;
            }
            float position_y = rho * sin(phi);
            if (position_y < 0.0001) {
                position_y = 0.0001;
            }
            float velocity_x = rho_dot * cos(phi);
            float velocity_y = rho_dot * sin(phi);
            x << position_x, position_y, velocity_x , velocity_y;
        }

        // 避免运算时，0作为被除数
        if (fabs(x(0)) < 0.001) {
            x(0) = 0.001;
        }
        if (fabs(x(1)) < 0.001) {
            x(1) = 0.001;
        }
        // 初始化Kalman滤波器
        kf.Initialization(x);

        // 设置协方差矩阵P
        Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
        P << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1000.0, 0.0,
                0.0, 0.0, 0.0, 1000.0;
        kf.SetP(P);

        // 设置过程噪声Q
        Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
        Q << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
        kf.SetQ(Q);

        // 存储第一帧的时间戳，供下一帧数据使用
        last_timestamp = measurementPackage.timestamp_;
        is_initialized = true;
        return;
    }

    // 求前后两帧的时间差，数据包中的时间戳单位为微秒，处以1e6，转换为秒
    //double tt = double(measurementPackage.timestamp_ - last_timestamp);
    double delta_t =  (measurementPackage.timestamp_ - last_timestamp) / 1000000.0; // unit : s
    last_timestamp = measurementPackage.timestamp_;

    // 设置状态转移矩阵F
    Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
    F << 1.0, 0.0, delta_t, 0.0,
            0.0, 1.0, 0.0, delta_t,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    kf.SetF(F);

    // 预测
    kf.Prediction();

    // 更新
    if (measurementPackage.sensor_type_ == MeasurementPackage::LASER) {
        kf.SetH(H_lidar_);
        kf.SetR(R_lidar_);
        kf.KF_MeasurementUpdate(measurementPackage.raw_measurements_);
    } else if (measurementPackage.sensor_type_ == MeasurementPackage::RADAR) {
        kf.SetR(R_radar_);
        // Jocobian矩阵Hj的运算已包含在EKFUpdate中
        kf.EKF_MeasurementUpdate(measurementPackage.raw_measurements_);
    }
}