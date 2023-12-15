#include "robot_model.hpp"
#include "robot_arm.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace Eigen;
int main(int argc, char const *argv[])
{
    // dxl_motor motors;
    // motors.addMotorId(1);
    // motors.addMotorId(2);
    // motors.addMotorId(2 + clone_id_offset);//2のクローン
    // motors.addMotorId(3);
    // motors.addMotorId(4);
    // motors.addMotorId(5);
    // motors.setAllSpeed(250);
    // motors.allTorqueEnable(true);

    auto ptr = createRobotArm();
    auto result = calcForwardKinematics();
    Plot3D plot;
    // plot.legend().hide();
    plot.xlabel("x");
    plot.ylabel("y");
    plot.zlabel("z");
    // plot.yrange(-3, 7);
    // plot.xrange(0.0f, 7);
    plot.drawWithVecs("linespoints", result.x, result.y,result.z).lineColor("red");
    Figure fig = {{plot}};
    Canvas canvas = {{fig}};
    // canvas.show();

    Plot2D plot2d;
    plot2d.drawWithVecs("linespoints", result.x, result.y).lineColor("red");
    Figure fig2d = {{plot2d}};
    Canvas canvas2d = {{fig2d}};
    // canvas2d.show();
    return 0;
}
