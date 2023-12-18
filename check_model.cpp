#include "robot_arm.hpp"

void printContainer(const std::vector<double>& vec,std::string message = ""){
    std::cout << message;
    for  (const auto& v : vec)
    {
        std::cout << v << ", ";
    }
    std::cout << std::endl;
}

int main(int argc, char const *argv[])
{
    //PC上の処理では+だと上に回転する。
    // auto ptr = createRobotArm();
    // ptr->at(1).setJointAngle(M_PI/4);
    // ptr->at(1).setJointAngle(M_PI/8.0f);
    // plot2darm(true);
    auto ptr = createRobotArm();
    ptr->at(2).setRelativeJointAngle(0);
    ptr->at(2).setJointAngle(0);
    ptr->at(2).setRelativeJointAngle(0);
    ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    ptr->at(2).setJointAngle(M_PI/8.0f);
    // ptr->at(2).setJointAngle(0);
    // ptr->at(2).setJointAngle(0);
    std::cout << "offset" <<ptr->at(2).joint_angle_offset_ << std::endl;;
    plot2darm(true);
    // ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    // plot2darm(true);
    // ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    // plot2darm(true);
    // ptr->at(2).setRelativeJointAngle(M_PI/8.0f);
    // plot2darm(true);
    // ptr = createRobotArm();
    // ptr->at(3).setJointAngle(M_PI/8.0f);
    // plot2darm(true);
    // ptr = createRobotArm();
    // ptr->at(4).setJointAngle(M_PI/8.0f);
    // plot2darm(true);
    auto res = calcForwardKinematics();
    auto degs = getJointAngleParam();
    printContainer(degs,"[joint param degs] ");
    return 0;
}

/**
 * 回転方向メモ
 * id 1 = そもそも軸が違うので今回関係なし
 * id 2 = 下     id2は一番最初のx軸回転
 * id 3 = 下     90度が足せていないのが原因だった！！回転方向を同じにできた！
 * id 4 = 下
 * id 5 = ? //不明
 * 
 * もう一つの問題！！！
 *  setJointAngleは絶対角度指定だが、CCDでは相対角度が指令になっている。ここを変えないとマズイ
 * この時リバース設定は結構難しいぞどうしよう...
 */