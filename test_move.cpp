#include "dxl_motor.hpp"

int main(int argc, char const *argv[])
{
    dxl_motor motors;
    motors.addMotorId(1);
    motors.addMotorId(2);
    motors.addMotorId(3);
    motors.addMotorId(4);
    motors.allTorqueEnable(true);
    std::cout << "id1 is moving to 0 degree... " << std::endl;
    motors.setGoalPosition(1, 150); // 0度までid1を動かす
    std::cout << "id1 is moving to  degree... " << std::endl;
    sleep(2);
    motors.setGoalPosition(1, 180); // 中央から30度までid1を動かす
    sleep(5);
    std::cout << "Torque of motors will be disable... " << std::endl;
    sleep(1);
    return 0;
}
