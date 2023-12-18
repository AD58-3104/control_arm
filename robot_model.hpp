#ifndef ROBOT_MODEL_HPP_
#define ROBOT_MODEL_HPP_

#include <list>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <iomanip>

/**
 * @brief ZYXオイラー角から回転行列を生成する
 * @param rotate ZYXオイラー角[rad]。渡すのはx,y,zの順番で良い。内部でZYXの順に掛ける
 * @return Eigen::Matrix3d 回転行列
 */
Eigen::Matrix4d get3DRotationMatrix(const Eigen::Vector3d &rotate)
{
    Eigen::Matrix4d rotate_matrix = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d tmp;
    tmp = Eigen::AngleAxisd(rotate[2], Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rotate[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rotate[0], Eigen::Vector3d::UnitX());
    rotate_matrix.block<3, 3>(0, 0) = tmp;
    return rotate_matrix;
}

constexpr uint8_t clone_id_offset = 8; // クローンのidは元のid+8にする
struct StraightChainRobotModel
{
    // DH記法で表現.
    uint8_t motor_id_;
    Eigen::Vector3d x_link_length_; // x,y,z [mm]
    Eigen::Matrix4d x_link_rotate_;       // ZYXオイラー角 [rad]
    Eigen::Vector3d z_link_length_;
    Eigen::Matrix4d z_link_rotate_;
    double joint_angle_; // [rad]
    double joint_angle_offset_; // [rad] //構造上でz方向の回転が発生している時のオフセット
    bool does_reverse_;  // 1 or -1
    std::list<StraightChainRobotModel> clone_motors;
    StraightChainRobotModel(const uint8_t motor_id, const bool does_reverse) : motor_id_(motor_id), does_reverse_(does_reverse_)
    {
    }
    StraightChainRobotModel(
        const uint8_t motor_id,
        const double x_link_length,
        const Eigen::Vector3d x_link_rotate,
        const double z_link_length,
        const Eigen::Vector3d z_link_rotate) : motor_id_(motor_id),
                                               x_link_length_(x_link_length, 0, 0),
                                               x_link_rotate_(get3DRotationMatrix(x_link_rotate)),
                                               z_link_length_(0, 0, z_link_length),
                                               z_link_rotate_(get3DRotationMatrix(z_link_rotate)),
                                               joint_angle_(0),
                                               joint_angle_offset_(0),
                                               does_reverse_(false),
                                               clone_motors()
    {
        joint_angle_offset_ = z_link_rotate_(0, 2);
        joint_angle_ = joint_angle_offset_;
    }
    void printLinkState()
    {
        std::cout << "------- motor id <" << motor_id_ << "> --------" << std::endl;
        std::cout << "------- reverse <" << std::boolalpha << does_reverse_ << "> --------" << std::endl;
        std::cout << "*** x_link_length ***\n"
                  << x_link_length_ << std::endl;
        std::cout << "*** x_link_rotate ***\n"
                  << x_link_rotate_ << std::endl;
        std::cout << "*** z_link_length ***\n"
                  << z_link_length_ << std::endl;
        std::cout << "*** z_link_rotate ***\n"
                  << z_link_rotate_ << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;
    }

    void setCloneMotor(StraightChainRobotModel motor)
    {
        assert(motor.motor_id_ != motor_id_ + clone_id_offset); // idが間違っている
        clone_motors.push_back(motor);
        return;
    }

    void setYoffset(const double y_offset)
    {
        x_link_length_(1,0) = y_offset;
        return;
    }

    void setReverse(const bool does_reverse)
    {
        does_reverse_ = does_reverse;
        return;
    }

    /**
     * @brief z軸関節角度を設定する。z軸の回転に関する同時変換行列も更新される。
     * @param joint_angle 絶対角度 [rad]
     */
    void setJointAngle(const double joint_angle)
    {
        joint_angle_ *= (does_reverse_ ? -1.0f : 1.0f);
        z_link_rotate_ = get3DRotationMatrix(Eigen::Vector3d(0, 0, joint_angle_ + joint_angle_offset_ * (does_reverse_ ? -1.0f : 1.0f)));
        std::cout << "joint_angle: " << joint_angle_ << std::endl;
        std::cout << "joint_angle_offset: " << joint_angle_offset_ << std::endl;
        std::cout << z_link_rotate_ << std::endl;
        if (clone_motors.size() != 0)
        {
            for (auto &clone_motor : clone_motors)
            {
                clone_motor.setJointAngle(joint_angle); //ここはオフセット足す前のにしておかないと、またオフセットが足されてしまう。
            }
        }
        return;
    }

    Eigen::Matrix4d getTransformMatrix()
    {
        Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
        transform_matrix.block<3, 1>(0, 3) = x_link_length_.transpose();
        transform_matrix *= x_link_rotate_;
        Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
        tmp.block<3, 1>(0, 3) = z_link_length_.transpose();
        transform_matrix *= tmp;
        transform_matrix *= z_link_rotate_;
        return transform_matrix;
    }

    // /**
    //  * @brief DH記法におけるzの回転角を更新する
    //  * @param angle 回転角度[rad]
    //  */
    // void updateJointAngle(const double angle)
    // {
    //     z_link_rotate_(0, 2) = angle;
    //     return;
    // }

    /**
     * @brief 逆運動学を解く
     * @param target_position 目標位置(x,y,z) [mm]
     * @todo 逆運動学の実装 向きを決めて、sin,cosを解くのやつを実装する。
     */
    void solveInverseKinematicsGeometry(const Eigen::Vector3d &target_position)
    {
    }
};

#endif // !ROBOT_MODEL_HPP_
