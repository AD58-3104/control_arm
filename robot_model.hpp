#ifndef ROBOT_MODEL_HPP_
#define ROBOT_MODEL_HPP_

#include <list>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

/**
 * @brief ZYXオイラー角から回転行列を生成する
 * @param rotate ZYXオイラー角 [rad]
 * @return Eigen::Matrix3d 回転行列
 */
Eigen::Matrix3d get3DRotationMatrix(const Eigen::Vector3d &rotate)
{
    Eigen::Matrix3d rotate_matrix;
    rotate_matrix = Eigen::AngleAxisd(rotate[0], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rotate[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rotate[2], Eigen::Vector3d::UnitX());
    return rotate_matrix;
}

struct StraightChainRobotModel
{
    std::list<StraightChainRobotModel> clone_motors;
    // DH記法で表現
    const Eigen::Vector3d x_link_length_; // x,y,z [mm]
    Eigen::Matrix3d x_link_rotate_;       // ZYXオイラー角 [rad]
    const Eigen::Vector3d z_link_length_;
    Eigen::Matrix3d z_link_rotate_;
    double robote_direction; // 1 or -1
    StraightChainRobotModel(
        const double x_link_length,
        const Eigen::Vector3d x_link_rotate,
        const double z_link_length,
        const Eigen::Vector3d z_link_rotate) : x_link_length_(x_link_length, 0, 0),
                                               x_link_rotate_(get3DRotationMatrix(x_link_rotate)),
                                               z_link_length_(0,0,z_link_length),
                                               z_link_rotate_(get3DRotationMatrix(x_link_rotate))
    {
    }
    void printLinkState(){
        std::cout << "------- link of StraightChainRobotModel --------" << std::endl;
        std::cout << "*** x_link_length ***\n" << x_link_length_ << std::endl;
        std::cout << "*** x_link_rotate ***\n" << x_link_rotate_ << std::endl;
        std::cout << "*** z_link_length ***\n" << z_link_length_ << std::endl;
        std::cout << "*** z_link_rotate ***\n" << z_link_rotate_ << std::endl;
        std::cout << "-------------------------------------------------" << std::endl;
    }
    void setCloneMotor(StraightChainRobotModel motor){
        clone_motors.push_back(motor);
        return;
    }
};


#endif // !ROBOT_MODEL_HPP_
