//
// Created by Zhangjun on 2020/2/19.
//

#ifndef EKF_EXTENDEDKALMANFILTER_H
#define EKF_EXTENDEDKALMANFILTER_H

#include <Eigen>


class ExtendedKalmanFilter{
public:
    ExtendedKalmanFilter();

    ~ExtendedKalmanFilter(){}

    Eigen::VectorXd Get_X() { return x_; }
    void Initialization(Eigen::VectorXd x_in){
        x_ = x_in;
        is_initialized_ = true;
    }
    bool IsInitialized() {  return is_initialized_; }
    // 预测模块代码
    void SetF(Eigen::MatrixXd F_in) { F_ = F_in; }
    void SetP(Eigen::MatrixXd P_in) { P_ = P_in; }
    void SetQ(Eigen::MatrixXd Q_in) { Q_ = Q_in; }
    void SetH(Eigen::MatrixXd H_in) { H_ = H_in; }
    void SetR(Eigen::MatrixXd R_in) { R_ = R_in; }

    void Prediction(){
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    // 观测过程（measurement）
    void KF_MeasurementUpdate(Eigen::VectorXd z);
    void EKF_MeasurementUpdate(Eigen::VectorXd z);

private:
    bool is_initialized_;   //初始化标志
    Eigen::VectorXd x_;     //状态变量
    Eigen::MatrixXd F_;     //状态转移矩阵
    Eigen::MatrixXd P_;     //状态协方差矩阵
    Eigen::MatrixXd Q_;     //过程协方差矩阵
    Eigen::MatrixXd H_;     //雅克比测量矩阵
    Eigen::MatrixXd R_;     //测量协方差矩阵

    //雅克比矩阵计算
    void CalculateJacobianMatrix();
};

#endif //EKF_EXTENDEDKALMANFILTER_H
