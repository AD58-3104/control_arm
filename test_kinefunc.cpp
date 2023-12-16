#include "helper_func.hpp"
#include "robot_model.hpp"
#include "robot_arm.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace Eigen;
int main(int argc, char const *argv[])
{
    auto ptr = createRobotArm();
    ptr->at(1).setJointAngle( M_PI / 12); //id2の回転
    ptr->at(2).setJointAngle( M_PI / 12);  //id3の回転
    auto result = calcForwardKinematics();
    plotArm(result);
    auto degs = calcInverseKinematics({0, 22.5+190+66.5, 24});
    setAllJointAngle(degs);
    result = calcForwardKinematics();
    plotArm(result);
    std::cout << "--- End position ---" << std::endl;
    std::cout << result.getEndPositionVec().transpose() << std::endl;
    size_t index = 0;
    for (const auto &deg : degs)
    {
        std::cout << "index " << index<< "::"<< deg << std::endl;
        index++;
    }
    return 0;
}
