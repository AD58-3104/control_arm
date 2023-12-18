#include "robot_model.hpp"
#include "robot_arm.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace Eigen;
int main(int argc, char const *argv[])
{
    dxl_motor motors;
    motors.addMotorId(1);
    motors.addMotorId(2);
    motors.addMotorId(2 + clone_id_offset);//2のクローン
    motors.addMotorId(3);
    motors.addMotorId(4);
    motors.addMotorId(5);
    motors.setAllSpeed(30);   //速度を大分遅くした
    motors.allTorqueEnable(true);
    auto ptr = createRobotArm();
    sleep(1);
    ptr->at(0).setJointAngle(0); //手先の回転
    ptr->at(1).setJointAngle(0); //手先の回転
    ptr->at(2).setJointAngle(0); //手先の回転
    ptr->at(3).setJointAngle(0); //手先の回転
    ptr->at(4).setJointAngle(0); //手先の回転
    moveArm(motors);

    Eigen::Vector3d target_pos{0,22.5 + 190 + 66.5,200};
    auto solution = solveCCDINV(target_pos);
    auto result = calcForwardKinematics();
    int ok = -100;
    std::cout << "if you want to move arm, input number > 765" << std::endl;
    std::cin >> ok;
    if(ok > 765){
        // setAllJointAngle(solution.value());
        moveArm(motors);
    }
    return 0;
}


//:0, -28.9323, -45.2895, -73.3168, -146.674, 141.434,