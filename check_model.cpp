#include "robot_arm.hpp"

int main(int argc, char const *argv[])
{
    //PC上の処理では+だと上に回転する。
    auto ptr = createRobotArm();
    // ptr->at(1).setJointAngle(M_PI/4);
    ptr->at(1).setJointAngle(M_PI/8);
    plot2darm();
    ptr->at(2).setJointAngle(M_PI/8);
    plot2darm();
    ptr->at(3).setJointAngle(M_PI/8);
    plot2darm();
    ptr->at(4).setJointAngle(M_PI/8);
    plot2darm();
    auto res = calcForwardKinematics(); 
    return 0;
}

/**
 * 回転方向メモ
 * id 1 = 下     id1は一番最初のx軸回転
 * id 2 = 上
 * id 3 = 下
 * id 4 = 下
 */