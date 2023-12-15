#ifndef DXL_MOTORCLASS_HPP_
#define DXL_MOTORCLASS_HPP_
#include "dynamixel_sdk.h"
#include <unistd.h>
#include <cmath>
#include <set>
#include <iostream>
#include <stdexcept>

// AX12用のアドレス
constexpr uint8_t ADDR_MX_TORQUE_ENABLE = 24; // Control table address is different in Dynamixel model
constexpr uint8_t ADDR_MX_GOAL_POSITION = 30;
constexpr uint8_t ADDR_MX_PRESENT_POSITION = 36;
constexpr uint8_t ADDR_MX_MOVING_APEED = 32; // 動くスピード。しかしjointモードとwheelモードで違うらしい。

constexpr uint8_t PROTOCOL_VERSION = 1.0;
constexpr uint32_t BAUDRATE = 1000000;
const char *DEVICENAME = "/dev/ttyUSB0";

constexpr uint8_t TORQUE_ENABLE = 1;                  // Value for enabling the torque
constexpr uint8_t TORQUE_DISABLE = 0;                 // Value for disabling the torque
constexpr uint16_t DXL_MINIMUM_POSITION_VALUE = 100;  // 稼働範囲の下限 //事故りそうなので最初は小さくする
constexpr uint16_t DXL_MAXIMUM_POSITION_VALUE = 1000; // 可動範囲の上限
constexpr uint8_t DXL_MOVING_STATUS_THRESHOLD = 10;   // 位置の誤差の閾値

constexpr double DXL_POSITION_RESOLUTION = 1024.0f / 300.0f;

struct dxl_motor_sync_moves
{
    const uint8_t id;
    const uint16_t goal_position;
};

// TODO 速度を遅めに設定しておく

class dxl_motor
{
private:
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    std::set<uint8_t> motor_id_set_; // モータidの集合。これは単に管理対象を示すのみで、ここの順番をアプリケーション側が期待してはいけない。

public:
    dxl_motor();
    ~dxl_motor();
    void setTorqueEnable(const uint8_t id, const bool is_enable);
    void allTorqueEnable(const bool is_enable);
    void addMotorId(const uint8_t id);
    uint16_t setGoalPosition(const uint8_t id, const uint16_t goal_position);
    uint16_t setSyncGoalPosition(const std::vector<dxl_motor_sync_moves> &move_data);
};

dxl_motor::dxl_motor()
{
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (portHandler_->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        throw std::runtime_error("Failed to open the port!\n");
    }
    // Set port baudrate
    if (portHandler_->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        throw std::runtime_error("Failed to change the baudrate!\n");
    }
    std::cout << "dxl_motor class is initialized!" << std::endl;
    std::cout << "Torque of motors will be enable... " << std::endl;
}

dxl_motor::~dxl_motor()
{
    // Disable Dynamixel Torque
    allTorqueEnable(false);
    portHandler_->closePort();
}

void dxl_motor::allTorqueEnable(const bool is_enable)
{
    for (const auto &id : motor_id_set_)
    {
        setTorqueEnable(id, is_enable);
    }
    return;
}

void dxl_motor::setTorqueEnable(const uint8_t id, const bool is_enable)
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error
    if (is_enable)
    {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    }
    else
    {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }

    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
        throw std::runtime_error("Failed to set torque enable!\n");
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler_->getRxPacketError(dxl_error));
        throw std::runtime_error("Failed to set torque enable!\n");
    }
    return;
}

void dxl_motor::addMotorId(const uint8_t id)
{
    motor_id_set_.emplace(id);
    return;
}

/**
 * @brief モータを動かす。モータが目標位置に近くなるまでブロッキングする。
 * @param id モータのid
 * @param goal_position　目標位置(中央からの角度) [degree](0~300)
 * @return 最終的な現在位置
 */
uint16_t dxl_motor::setGoalPosition(const uint8_t id, const uint16_t goal_position)
{
    if (!motor_id_set_.contains(id))
    {
        throw std::runtime_error("This id is not registered!\n");
    }

    if (goal_position < DXL_MINIMUM_POSITION_VALUE || goal_position > DXL_MAXIMUM_POSITION_VALUE)
    {
        throw std::runtime_error("Goal position is out of movable range!\n");
    }

    const uint16_t goal_position_raw = goal_position * DXL_POSITION_RESOLUTION;
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0;              // Dynamixel error
    // 位置指令
    std::cout << "id" << (int)id << " is moving to " << goal_position_raw << " degree... " << std::endl;
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, ADDR_MX_GOAL_POSITION, goal_position_raw, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler_->getRxPacketError(dxl_error));
    }

    uint16_t dxl_present_position = 0; // Present position
    do
    {
        // Read present position
        dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler_->getRxPacketError(dxl_error));
        }

        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", id, goal_position_raw, dxl_present_position);
        usleep(5000);
    } while ((std::abs(goal_position_raw - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    return dxl_present_position;
}


/**
 * @brief モータを動かす。モータが目標位置に近くなるまでブロッキングする。
 * @param id モータのid
 * @param goal_position　目標位置(中央からの角度) [degree](0~300)
 * @return 最終的な現在位置
 */
uint16_t dxl_motor::setSyncGoalPosition(const std::vector<dxl_motor_sync_moves> &move_data)
{
    if (move_data.empty())
    {
        throw std::runtime_error("move_data is empty!");
    }
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, 2);
    for (const auto &data : move_data)
    {
        uint8_t param[2];
        uint16_t goal_position_raw = data.goal_position * DXL_POSITION_RESOLUTION;
        param[0] = DXL_LOBYTE(goal_position_raw);
        param[1] = DXL_HIBYTE(goal_position_raw);
        if (groupSyncWrite.addParam(data.id, param) != true)
        {
            throw std::runtime_error("Failed to add param!");
        }
    }
    if (groupSyncWrite.txPacket() != COMM_SUCCESS)
    {
        throw std::runtime_error("Failed to send packet!");
    }
    groupSyncWrite.clearParam();
    uint16_t dxl_present_position = 0; // Present position
    do
    {
        // Read present position
        int dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, move_data[0].id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, nullptr);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
        }
        printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", move_data[0].id, (uint16_t)(move_data[0].goal_position * DXL_POSITION_RESOLUTION), dxl_present_position);
        usleep(10000); // 10ms待つ
    } while ((std::abs(move_data[0].goal_position * DXL_POSITION_RESOLUTION - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    return dxl_present_position;
}

#endif // !DXL_MOTORCLASS_HPP_