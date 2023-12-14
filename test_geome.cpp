#include "robot_model.hpp"
#include <iostream>

using namespace Eigen;
int main(int argc, char const *argv[])
{
    Vector3d link_rotate(M_PI / 2, 0, 0);
    auto mat = get3DRotationMatrix(link_rotate);
    StraightChainRobotModel robot_model(10, link_rotate, 10, link_rotate);
    robot_model.printLinkState();
    return 0;
}
