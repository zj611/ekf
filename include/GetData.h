//
// Created by Zhangjun on 2020/2/19.
//

#ifndef EKF_GET_DATA_H
#define EKF_GET_DATA_H

#include <iostream>
#include <vector>
#include "DataPackage.h"

void GetData(std::string input_file_name,
             std::vector<MeasurementPackage> &measurement_pack_list,
             std::vector<GroundTruthPackage> &groundtruth_pack_list);


#endif //EKF_GET_DATA_H
