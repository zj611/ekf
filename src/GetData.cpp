//
// Created by Zhangjun on 2020/2/19.
//

#include <fstream>
#include "GetData.h"

void GetData(std::string input_file_name,
             std::vector<MeasurementPackage> &measurement_pack_list,
             std::vector<GroundTruthPackage> &groundtruth_pack_list){
    // 打开数据，若失败则输出失败信息，返回并终止程序
    // Open file. if failed return -1 & end program
    std::ifstream input_file(input_file_name.c_str(), std::ifstream::in);
    if (!input_file.is_open()) {
        std::cout << "Failed to open file named : " << input_file_name << std::endl;
        return;
    }
    // measurement_pack_list：毫米波雷达/激光雷达实际测得的数据。
    // groundtruth_pack_list：每次测量时，障碍物位置的真值。
    // 通过while循环将雷达测量值和真值全部读入内存，存入measurement_pack_list和groundtruth_pack_list中
    std::string line;
    while (getline(input_file, line)) {
        std::string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        std::istringstream iss(line);
        long long int timestamp;

        // 读取当前行的第一个元素，L代表Lidar数据，R代表Radar数据
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) {
            // 该行第二个元素为测量值x，第三个元素为测量值y，第四个元素为时间戳(纳秒）
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type.compare("R") == 0) {
            // 该行第二个元素为距离pho，第三个元素为角度phi，第四个元素为径向速度pho_dot，第五个元素为时间戳(纳秒）
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            float rho;
            float phi;
            float rho_dot;
            iss >> rho;
            iss >> phi;
            iss >> rho_dot;
            meas_package.raw_measurements_ << rho, phi, rho_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }

        // 当前行的最后四个元素分别是x方向上的距离真值，y方向上的距离真值，x方向上的速度真值，y方向上的速度真值
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = Eigen::VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        groundtruth_pack_list.push_back(gt_package);
    }

    std::cout << "Success to load data." << std::endl;
}

