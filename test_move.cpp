#include "dxl_motor.hpp"
#include "robot_arm.hpp"

int main(int argc, char const *argv[])
{
    dxl_motor motors;
    motors.addMotorId(1);
    motors.addMotorId(2);
    motors.addMotorId(10);
    motors.allTorqueEnable(true);
    // 2,3は同時に逆に動かす必要あり！！！！危険です！！！
    std::cout << "id1 is moving to 0 degree... " << std::endl;
    motors.setGoalPosition(1, 150);  // 0度までid1を動かす
    motors.setGoalPosition(2, 150);  // 中央から10度までid1を動かす
    motors.setGoalPosition(10, 150); // 中央から10度までid1を動かす
    std::cout << "id1 is moving to  degree... " << std::endl;
    sleep(1);
    //2つのモータを同時に逆方向に動かす
    std::vector<dxl_motor_sync_moves> move_data;
    move_data.push_back({2, 150 + 70});
    move_data.push_back({10, 150 - 70});
    motors.setSyncGoalPosition(move_data);
    sleep(1);
    std::cout << "Torque of motors will be disable... " << std::endl;
    sleep(1);
    motors.allTorqueEnable(false);
    return 0;
}
