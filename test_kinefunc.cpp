#include "helper_func.hpp"
#include "robot_model.hpp"
#include "robot_arm.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace Eigen;
constexpr double maenohou_len = 22.5 + 190 + 66.5;
int main(int argc, char const *argv[])
{
    Eigen::Vector3d target_pos{0,0,0};
    if(argc == 3){
        std::string y(argv[1]);
        std::string z(argv[2]);
        target_pos(1) = std::stod(y);
        target_pos(2) = std::stod(z);
    }
    else if(argc == 2){
        auto rand = getRandomJointDegree(2);
        target_pos(1) = rand[0] * 360.f / M_PI;
        target_pos(2) = 30;
    }
    else{
        target_pos(0) = 0;
        target_pos(1) = maenohou_len + 70;
        target_pos(2) = 20;
    }
    auto ptr = createRobotArm();
    auto res_opt = solveCCDINV(target_pos);
    if(res_opt.has_value()){
        auto result_degs = res_opt.value();
    }
    else{
        std::cerr << "<<<<< Cannot solve problem!!! >>>>> " << std::endl;
        return 1;
    }
    ResultPosition result = calcForwardKinematics();
    // std::vector<double> degs;
    // size_t yarinaoshi_count = 0;
    // constexpr size_t yarinaoshi_threshould = 10000;
    // auto checkYZisnotMinus = [](const ResultPosition& pos) -> bool {
    //     for (size_t ind = 0; ind < pos.z.size(); ind++)
    //     {
    //         if (pos.z[ind] < -0.1) //誤差考慮
    //         {
    //             // std::cout << "[Error]: z is minus on link " << ind << " !!!!!" << std::endl;
    //             return false;
    //         }
    //         if(pos.y[ind] < -0.1){ //誤差考慮
    //             return false;
    //         }
    //         ind++;
    //     }
    //     return true;
    // };
    // while (true)
    // {
    //     auto joi = getRandomJointDegree(ptr->size());
    //     joi[0] = 0;//根元の回転はしてほしくないので0にする
    //     setAllJointAngle(joi); //ランダムな初期姿勢を設定
    //     auto solution = calcInverseKinematics(target_pos);
    //     if(solution.has_value()){
    //         result = calcForwardKinematics();
    //         if(checkYZisnotMinus(result)){
    //             break;
    //         }
    //     }
    //     yarinaoshi_count++;
    //     if(yarinaoshi_count > yarinaoshi_threshould){
    //         std::cerr << "<<<<< Cannot solve problem!!! >>>>> " << std::endl;
    //         return 1;
    //     }
    // }
    // std::cout << "------------- [Calculation is finished] -------------" << std::endl;
    // std::cout << "--- End position ---" << std::endl;
    // std::cout << result.getEndPositionVec().transpose() << std::endl;
    // result.print();
    getCalcJointAngle();
    // std::cout << "[yarinaoshi_count] :" << yarinaoshi_count << std::endl;
    // plotArm(result);
    plot2darm();
    return 0;
}
