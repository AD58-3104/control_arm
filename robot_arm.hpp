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
    // 根元の回転軸をzとした左手系で定義。xは右、yは上

    // origin
    robot_arm->push_back(StraightChainRobotModel(1, 0, Eigen::Vector3d::Zero(), 24, Eigen::Vector3d::Zero()));
    // 根元回転
    robot_arm->push_back(StraightChainRobotModel(2, 0, Eigen::Vector3d(0, -M_PI / 2.0, 0), 0, Eigen::Vector3d::Zero()));
    // id2のクローン. リバースがtrue
    robot_arm->back().clone_motors.push_back(StraightChainRobotModel(2 + clone_id_offset, true));
    // 次のリンク
    // ここyへの22.5も足さなくてはいけない...。どうしよう
    robot_arm->push_back(StraightChainRobotModel(3, 195, Eigen::Vector3d(0, 0, 0), 0, Eigen::Vector3d(0, 0, M_PI / 2)));
    // 次のリンク
    robot_arm->push_back(StraightChainRobotModel(4, 190, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    // 手先のリンク
    robot_arm->push_back(StraightChainRobotModel(5, 66.5, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
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
        // クローン動作のモータがある場合
        if (!motor.clone_motors.empty())
        {
            // 基のモータのデータを追加
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
                // クローン動作のモータを追加
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
    result.x.push_back(0); //原点
    result.y.push_back(0); //原点
    result.z.push_back(0); //原点
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    for (auto &motor : *robot_arm)
    {
        auto tf = motor.getTransformMatrix();
        res *= tf;
        result.x.push_back(res(0, 3));
        result.y.push_back(res(1, 3));
        result.z.push_back(res(2, 3));
        std::cout << "*** motor id " << (uint16_t)(motor.motor_id_) << " ***" << std::endl;
        std::cout << tf << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << res << std::endl;
    }
    std::cout << "^^^ result ^^^" << std::endl;
    std::cout << res << std::endl;
    std::cout << "||| octave data |||" << std::endl;
    std::cout << "x = [";
    for (auto &x : result.x)
    {
        std::cout << x << " ";
    }
    std::cout << "];" << std::endl;
    std::cout << "y = [";
    for (auto &y : result.y)
    {
        std::cout << y << " ";
    }
    std::cout << "];" << std::endl;
    std::cout << "z = [";
    for (auto &z : result.z)
    {
        std::cout << z << " ";
    }
    std::cout << "];" << std::endl;
    return result;
}

#endif // !ROBOT_ARM_HPP