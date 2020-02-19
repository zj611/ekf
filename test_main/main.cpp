#include <iostream>
#include <Eigen>
#include <math.h>
#include "DataPackage.h"
#include "GetData.h"
#include "SensorFusion.h"

//using namespace std;
//using namespace Eigen;

int main() {
    std::vector<MeasurementPackage> measurement_pack_list;
    std::vector<GroundTruthPackage> groundtruth_pack_list;
    std::string input_file_name ="../data/sample-laser-radar-measurement-data-2.txt";//注意clion的自己编译路径cmake-build-debug
    GetData(input_file_name,measurement_pack_list,groundtruth_pack_list);

    SensorFusion sensorFusion;

    //计算均方根误差（RMSE）
    double x_sum = 0 , y_sum = 0;

    for(int i = 0; i < measurement_pack_list.size(); i++)
    {
        sensorFusion.Process(measurement_pack_list[i]);
        Eigen::Vector4d x_out = sensorFusion.kf.Get_X();
        Eigen::Vector4d x_ground = groundtruth_pack_list[i].gt_values_;

        std::cout<<"   x: "<<x_out[0]<<" - " << x_ground[0];
        std::cout<<"   y: "<<x_out[1]<<" - " << x_ground[1];
        std::cout<<"   vx: "<<x_out[2]<<" - " << x_ground[2];
        std::cout<<"   vy: "<<x_out[3]<<" - " << x_ground[3]<<std::endl;

        x_sum += pow(x_out[0]-x_ground[0],2);
        y_sum += pow(x_out[1]-x_ground[1],2);
    }

    double RMSE_x = sqrt(x_sum / measurement_pack_list.size()) ;
    double RMSE_y = sqrt(y_sum / measurement_pack_list.size()) ;

    std::cout << "RMSE_x = "<<RMSE_x << std::endl;
    std::cout << "RMSE_y = "<<RMSE_y << std::endl;

    return 0;
}
