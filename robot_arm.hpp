#ifndef ROBOT_ARM_HPP
#define ROBOT_ARM_HPP
#include "dxl_motor.hpp"
#include "robot_model.hpp"
#include <list>
#include <barrier>
#include <memory>
#include <future>

using arm_shared_ptr_t = std::shared_ptr<std::vector<StraightChainRobotModel>>;

static arm_shared_ptr_t robot_arm;

arm_shared_ptr_t createRobotArm()
{
    robot_arm = std::make_shared<std::vector<StraightChainRobotModel>>();
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
    robot_arm->back().setYoffset(22.5); // だいぶ小手先だが仕方ない...
    // 次のリンク
    robot_arm->push_back(StraightChainRobotModel(4, 190, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    // 手先のリンク
    robot_arm->push_back(StraightChainRobotModel(5, 66.5, Eigen::Vector3d::Zero(), 0, Eigen::Vector3d::Zero()));
    return robot_arm;
}

arm_shared_ptr_t getRobotArm()
{
    assert(robot_arm != nullptr); // robot_armが初期化されていない
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
    auto getEndPositionVec()
    {
        return Eigen::Vector3d(x.back(), y.back(), z.back());
    }
    auto getPositionVec(const size_t index)
    {
        return Eigen::Vector3d(x[index], y[index], z[index]);
    }
    void print(){
        std::cout << "x: ";
        for(auto &data: x){
            std::cout << data << ", ";
        }
        std::cout << std::endl;
        std::cout << "y: ";
        for(auto &data: y){
            std::cout << data << ", ";
        }
        std::cout << std::endl;
        std::cout << "z: ";
        for(auto &data: z){
            std::cout << data << ", ";
        }
        std::cout << std::endl;
    
    }
};

ResultPosition calcForwardKinematics(bool print = false)
{
    if (robot_arm == nullptr)
    {
        throw std::runtime_error("robot_arm is not initialized!");
    }
    ResultPosition result;
    result.x.push_back(0); // 原点
    result.y.push_back(0); // 原点
    result.z.push_back(0); // 原点
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();
    for (auto &motor : *robot_arm)
    {
        auto tf = motor.getTransformMatrix();
        res *= tf;
        result.x.push_back(res(0, 3));
        result.y.push_back(res(1, 3));
        result.z.push_back(res(2, 3));
        if (print)
        {
            std::cout << "*** motor id " << (uint16_t)(motor.motor_id_) << " ***" << std::endl;
            std::cout << tf << std::endl;
            std::cout << "--------------------------" << std::endl;
            std::cout << res << std::endl;
        }
    }
    if (print)
    {
        std::cout << "^^^ result ^^^" << std::endl;
        std::cout << res << std::endl;
    }
    return result;
}

void setAllJointAngle(const std::vector<double> joint_angles)
{
    assert(robot_arm != nullptr); // robot_armが初期化されていない
    if (joint_angles.size() != robot_arm->size())
    {
        throw std::runtime_error("joint_angles size is not equal to robot_arm size!");
    }
    auto itr = joint_angles.begin();
    for (auto &motor : *robot_arm)
    {
        motor.setJointAngle(*itr);
        itr++;
    }
    return;
}

/**
 * @brief 与えられた目標位置に対するアームの逆運動学を解く
 * @param target_position 目標位置(x,y)
 * @return std::vector<double> 各関節角度(degree)
 * 多分本番と回転の向きが逆だ！！！！！
 */
constexpr double error_threshold = 5.0f;
std::vector<double> calcInverseKinematics(const Eigen::Vector3d target_position)
{
    if (robot_arm == nullptr)
    {
        throw std::runtime_error("robot_arm is not initialized!");
    }
    const size_t arm_size = getRobotArm()->size();
    auto convertToDegreeContainer = [](const std::vector<double> rad_data) {
        std::vector<double> result;
        for (auto &data : rad_data)
        {
            result.push_back(data * 180 / M_PI);
        }
        return result;
    };
    auto convertToDegree = [](const double rad_data) {
        return rad_data * 180 / M_PI;
    };
    std::vector<double> result(arm_size, 0.0f);
    for (size_t iterate = 0; iterate < 100000; iterate++)
    {
        for (size_t index = arm_size; 1 < index; index--) //根元は計算しないので、1までしか対象に入れない.ここは1 < である必要あり。何故かというと、Fkineが原点の0,0,0を返してしまうからそれを計算しないようにするため。
        {
            auto res = calcForwardKinematics();
            auto link_end = res.getEndPositionVec();
            auto target_link = res.getPositionVec(index - 1);
            auto target_link_vec = link_end - target_link;
            auto origin_link_vec = target_position - target_link;
            double target_angle = acos(target_link_vec.dot(origin_link_vec) / (target_link_vec.norm() * origin_link_vec.norm()));
            auto muki = target_link_vec.cross(origin_link_vec).normalized();
            // std::cout << "muki: " << muki << std::endl;
            //回転の軸を決める
            if(muki(0) >= 0)
            {
                target_angle *= -1.0f;  //zの回転軸が自分が最初設定したつもりのものと逆だったのでx方向へのプラスのベクトルで逆回転になる。
            }
            getRobotArm()->at(index -1).setJointAngle(convertToDegree(target_angle));
            result[index - 1] = target_angle;
            // std::cout << "index: " << index - 1 << std::endl;
            // std::cout << "target_angle: " << target_angle * 180 /M_PI << std::endl;
            // setAllJointAngle(convertToDegree(result));
        }
        auto res = calcForwardKinematics();
        auto link_end = res.getEndPositionVec();
        auto error = target_position - link_end;
        if(error.norm() < error_threshold)
        {
            std::cout << "Loop count: " << iterate << std::endl;
            std::cout << "OK error is small!!  " << std::endl;
            break;
        }
    }
    for (auto &data : result)
    {
        data = data * 180 / M_PI; // degreeに変換してから返す
    }
    return result;
}

#endif // !ROBOT_ARM_HPP