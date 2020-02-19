//
// Created by Zhangjun on 2020/2/19.
//

#include "ExtendedKalmanFilter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    is_initialized_ = false;
}

void ExtendedKalmanFilter::KF_MeasurementUpdate(Eigen::VectorXd z)
{
    Eigen::VectorXd y = z - H_ * x_;
    Eigen::MatrixXd Ht = H_.transpose();
    Eigen::MatrixXd S = H_ * P_ * Ht + R_;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd K =  P_ * Ht * Si;
    x_ = x_ + (K * y);
    //int x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
}

void ExtendedKalmanFilter::EKF_MeasurementUpdate(Eigen::VectorXd z){
    double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    double theta = atan2(x_(1),x_(0));
    double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
    Eigen::VectorXd h = Eigen::VectorXd(3);
    h << rho,theta,rho_dot;
    Eigen::VectorXd y = z - h;

    CalculateJacobianMatrix();

    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
    x_ = x_ + (K * y);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(),x_.size());
    P_ = (I - K * H_) * P_;
}

void ExtendedKalmanFilter::CalculateJacobianMatrix(){
    Eigen::MatrixXd Hj(3,4);
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    if(fabs(c1) < 0.0001){
        H_ = Hj;
        return;
    }
    //计算雅克比矩阵
    Hj << (px/c2),(py/c2),0,0,
            -(py/c1),(px/c1),0,0,
            py*(vx*py-vy*px)/c3,px*(px*vy - py*vx)/c3,px/c2,py/c2;
    H_ = Hj;
}