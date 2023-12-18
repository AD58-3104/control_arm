#include "dxl_motor.hpp"
#include "robot_arm.hpp"

int main(int argc, char const *argv[])
{
    dxl_motor motors;
    motors.addMotorId(1);
    motors.addMotorId(2);
    motors.addMotorId(3);
    motors.addMotorId(4);
    motors.addMotorId(5);
    motors.addMotorId(10);
    motors.allTorqueEnable(true);
    motors.setAllSpeed(100);
    // 2,3は同時に逆に動かす必要あり！！！！危険です！！！
    std::cout << "id1 is moving to 0 degree... " << std::endl;
    motors.setGoalPosition(1, 150);
    sleep(1);
    motors.setGoalPosition(3, 150);
    sleep(1);
    motors.setGoalPosition(4, 150);
    sleep(1);
    motors.setGoalPosition(5, 150);
    sleep(1);
    //2つのモータを同時に逆方向に動かす
    std::vector<dxl_motor_sync_moves> move_data;
    move_data.push_back({2, 150});
    move_data.push_back({10, 150});
    motors.setSyncGoalPosition(move_data);
    std::cout << "Torque of motors will be disable... " << std::endl;
    sleep(2);
    motors.allTorqueEnable(false);
    return 0;
}
