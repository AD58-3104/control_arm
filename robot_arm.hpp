#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP
#include "dxl_motor.hpp"
#include "robot_model.hpp"
#include <list>
#include <memory>

using arm_shared_ptr_t = std::shared_ptr<std::list<StraightChainRobotModel>>;

static arm_shared_ptr_t robot_arm;

arm_shared_ptr_t createRobotArm()
{
    robot_arm = std::make_shared<std::list<StraightChainRobotModel>>();
    //ロボットモデルはここで定義する
    robot_arm->push_back(StraightChainRobotModel(1, 0, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    robot_arm->push_back(StraightChainRobotModel(2, 0, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    robot_arm->push_back(StraightChainRobotModel(3, 0, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    robot_arm->push_back(StraightChainRobotModel(4, 0, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    return robot_arm;
}

/**
 * @brief アームを動かす
 * @param robot_arm
 * @param motors
 * @todo クローンのリバース考慮
 * @todo 角度の変換
 */
void moveArm(dxl_motor &motors)
{
    if(robot_arm == nullptr){
        throw std::runtime_error("robot_arm is not initialized!");
    }
    for (auto &motor : *robot_arm)
    {
        uint16_t position = motor.joint_angle_ * 180 / M_PI; // 取り敢えずラジアン
        if (motor.does_reverse_)
        {
            position *= -1.0f;
        }
        position = 150 + position;
        motors.setGoalPosition(motor.motor_id_, position);
        if (!motor.clone_motors.empty())
        {
            for (auto &clone_motor : motor.clone_motors)
            {
                uint16_t position = motor.joint_angle_ * 180 / M_PI; // 取り敢えずラジアン
                if (motor.does_reverse_)
                {
                    position *= -1.0f;
                }
                position = 150 + position;
                motors.setGoalPosition(clone_motor.motor_id_, position);
            }
        }
    }
    return;
}

#endif // !ROBOT_ARM_HPP