#include "robot_model.hpp"
#include "robot_arm.hpp"
#include <iostream>
#include <sciplot/sciplot.hpp>

using namespace sciplot;
using namespace Eigen;
int main(int argc, char const *argv[])
{
    dxl_motor motors;
    motors.addMotorId(1);
    motors.addMotorId(2);
    motors.addMotorId(2 + clone_id_offset);//2のクローン
    motors.addMotorId(3);
    motors.addMotorId(4);
    motors.addMotorId(5);
    motors.setAllSpeed(250);
    motors.allTorqueEnable(true);
    sleep(2);
    auto ptr = createRobotArm();
    auto itr = ptr->begin();
    itr->setJointAngle(M_PI/10); //根元回転
    itr++;
    itr->setJointAngle(M_PI/4); //根元からのリンク回転
    // itr++;
    // itr->setJointAngle(-M_PI/8); //id3の回転
    // itr++;
    // itr->setJointAngle(-M_PI/3); //id4の回転
    itr = ptr->end();
    itr--;
    itr->setJointAngle(-M_PI/6); //手先の回転
    moveArm(motors);
    itr->setJointAngle(-M_PI/12); //手先の回転
    moveArm(motors);
    auto result = calcForwardKinematics();
    sleep(2);
    Plot3D plot;
    plot.legend().hide();
    plot.xlabel("x");
    plot.ylabel("y");
    plot.zlabel("z");
    plot.yrange(-300.f, 300.f);
    plot.xrange(-100.f, 100.f);
    plot.zrange(0.0f, 400.f);
    plot.drawWithVecs("linespoints", result.x, result.y,result.z).lineColor("red");
    Figure fig = {{plot}};
    Canvas canvas = {{fig}};
    // canvas.show();

    // Plot2D plot2d;
    // plot2d.drawWithVecs("linespoints", result.x, result.y).lineColor("red");
    // Figure fig2d = {{plot2d}};
    // Canvas canvas2d = {{fig2d}};
    // canvas2d.show();
    return 0;
}
