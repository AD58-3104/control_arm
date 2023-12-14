#include "robot_model.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace Eigen;
int main(int argc, char const *argv[])
{
    Vector3d link_rotate(0, 0, M_PI / 5);
    auto mat = get3DRotationMatrix(link_rotate);
    StraightChainRobotModel origin(0, 0, Vector3d::Zero(), 0, Vector3d(0,0,M_PI/3));
    StraightChainRobotModel link0(1, 3, Vector3d::Zero(), 0, Vector3d(0,0,-M_PI/1.5f));
    StraightChainRobotModel link1(1, 3, Vector3d::Zero(), 0, link_rotate);
    // robot_model.printLinkState();

    std::vector<float> result_x{};
    std::vector<float> result_y{};

    auto res = origin.getTransformMatrix(); //-------- 動かす
    std::cout << res << std::endl;
    result_x.push_back(res(0, 3));
    result_y.push_back(res(1, 3));
    res *= link0.getTransformMatrix(); //-------- 動かす
    std::cout << res << std::endl;
    result_x.push_back(res(0, 3));
    result_y.push_back(res(1, 3));
    res *= link1.getTransformMatrix(); //-------- 動かす
    std::cout << res << std::endl;
    result_x.push_back(res(0, 3));
    result_y.push_back(res(1, 3));

    Plot2D plot;
    plot.legend().hide();
    plot.xlabel("x");
    plot.ylabel("y");
    plot.yrange(-3, 7);
    plot.xrange(0.0f, 7);
    plot.drawWithVecs("linespoints", result_x, result_y).lineColor("red");
    Figure fig = {{plot}};
    Canvas canvas = {{fig}};
    canvas.show();
    return 0;
}
