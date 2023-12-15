#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP
#include "dxl_motor.hpp"
#include "robot_model.hpp"
#include <list>
#include <barrier>
#include <memory>
#include <future>

using arm_shared_ptr_t = std::shared_ptr<std::list<StraightChainRobotModel>>;

static arm_shared_ptr_t robot_arm;

arm_shared_ptr_t createRobotArm()
{
    robot_arm = std::make_shared<std::list<StraightChainRobotModel>>();
    // ロボットモデルはここで定義する
    robot_arm->push_back(StraightChainRobotModel(1, 0, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    robot_arm->push_back(StraightChainRobotModel(2, 2, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    // robot_arm->push_back(StraightChainRobotModel(3, 2, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    // robot_arm->push_back(StraightChainRobotModel(4, 1, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    return robot_arm;
}

/**
 * @brief アームを動かす
 * @param robot_arm
 * @param motors 
 */
void moveArm(dxl_motor &motors)
{
    if (robot_arm == nullptr)
    {
        throw std::runtime_error("robot_arm is not initialized!");
    }
    for (auto &motor : *robot_arm)
    {
        uint16_t position = motor.joint_angle_ * 180 / M_PI; // 取り敢えずラジアン
        if (motor.does_reverse_)
        {
            position *= -1.0f;
        }
        position = DXL_CENTOR_POSITION + position;
        //クローン動作のモータがある場合
        if (!motor.clone_motors.empty())
        {
            //基のモータのデータを追加
            std::vector<dxl_motor_sync_moves> move_data;
            move_data.push_back({motor.motor_id_, position});
            for (auto &clone_motor : motor.clone_motors)
            {
                uint16_t position = motor.joint_angle_ * 180 / M_PI; // 取り敢えずラジアン
                if (motor.does_reverse_)
                {
                    position *= -1.0f;
                }
                position = DXL_CENTOR_POSITION + position;
                //クローン動作のモータを追加
                move_data.push_back({clone_motor.motor_id_, position});
            }
            motors.setSyncGoalPosition(move_data);
        }
        else
        {
            motors.setGoalPosition(motor.motor_id_, position);
        }
    }
    return;
}

/**
 * @brief アームのリンクそれぞれの位置の結果
 */
struct ResultPosition
{
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
};

ResultPosition calcForwardKinematics()
{
    if (robot_arm == nullptr)
    {
        throw std::runtime_error("robot_arm is not initialized!");
    }
    ResultPosition result;
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    for (auto &motor : *robot_arm)
    {
        res *= motor.getTransformMatrix();
        result.x.push_back(res(0, 3));
        result.y.push_back(res(1, 3));
        result.z.push_back(res(2, 3));
    }
    std::cout << res << std::endl;
    return result;
}

#endif // !ROBOT_ARM_HPP