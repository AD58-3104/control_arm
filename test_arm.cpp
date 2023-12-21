#include "robot_model.hpp"
#include "robot_arm.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>
#include "helper_func.hpp"

using namespace sciplot;
using namespace Eigen;
int main(int argc, char const *argv[])
{
    MagnetController magnet;
    uint8_t mag = MagnetController::magnet_state::DISABLE_UP;
    magnet.control(mag);
    Eigen::Vector3d target_pos{15, 180 + 80, 100};
    double nemoto_rad = calcAngleXY(target_pos) - M_PI / 2.0f;
    dxl_motor motors;
    motors.addMotorId(1);
    motors.addMotorId(2);
    motors.addMotorId(2 + clone_id_offset); // 2のクローン
    motors.addMotorId(3);
    motors.addMotorId(4);
    motors.addMotorId(5);
    motors.setAllSpeed(30); // 速度を大分遅くした
    motors.allTorqueEnable(true);
    motors.setGoalPosition(5, 75);
    auto ptr = createRobotArm();
    sleep(5);
    ptr->at(0).setJointAngle(nemoto_rad); // 手先の回転
    ptr->at(1).setJointAngle(0);          // 手先の回転
    ptr->at(2).setJointAngle(0);          // 手先の回転
    ptr->at(3).setJointAngle(0);          // 手先の回転
    ptr->at(4).setJointAngle(0);          // 手先の回転
    moveArm(motors);


    motors.setGoalPosition(5, 150);
    mag = MagnetController::magnet_state::DISABLE_DOWN;
    magnet.control(mag);
    sleep(1);
    motors.setGoalPosition(5, 75);

    // int ok = -100;
    // Eigen::Vector3d target_height_and_distance{0, calcDistanceXY(target_pos), target_pos(2)};
    // Eigen::Vector3d tmp_target = target_height_and_distance;
    // const std::vector<double> height_list{180,100};
    // tmp_target(2) = height_list[0];
    // // auto solution = solveCCDINV(target_height_and_distance);
    // size_t count = 0;
    // while (true)
    // {
    //     auto result = calcForwardKinematics();
    //     // ptr->at(0).setJointAngle(calcAngleXY(target_pos)); //根元の回転
    //     ptr->at(0).setJointAngle(M_PI / 6); // 根元の回転
    //     if (count + 1 >= height_list.size())
    //     {
    //         tmp_target = target_height_and_distance;
    //     }
    //     solveCCDINV(tmp_target);
    //     plot2darm(true);
    //     std::cout << "if you want to move arm, input number > 765" << std::endl;
    //     std::cin >> ok;
    //     if (ok > 765)
    //     {
    //         count++;
    //         ptr->at(0).setJointAngle(nemoto_rad); // 根元の回転
    //         moveArm(motors);
    //         tmp_target(2) = height_list[count];
    //         if (count + 1 >= height_list.size())
    //         {
    //             break;
    //         }
    //     }
    //     else
    //     {
    //         continue;
    //     }
    //     sleep(1);
    // }
    sleep(5);
    return 0;
}

//: 0, -28.9323, -45.2895, -73.3168, -146.674, 141.434,