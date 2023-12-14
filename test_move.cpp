#include "dynamixel_sdk.h"
#include <cmath>

// Control table address
constexpr uint8_t ADDR_MX_TORQUE_ENABLE = 24; // Control table address is different in Dynamixel model
constexpr uint8_t ADDR_MX_GOAL_POSITION = 30;
constexpr uint8_t ADDR_MX_PRESENT_POSITION = 36;

// Protocol version
constexpr uint8_t PROTOCOL_VERSION = 1.0; // See which protocol version is used in the Dynamixel

// Default setting
constexpr uint8_t DXL_ID = 3; // Dynamixel ID: 1
constexpr uint32_t BAUDRATE = 1000000;
const char *DEVICENAME = "/dev/ttyUSB0"; // Check which port is being used on your controller
                                         // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

constexpr uint8_t TORQUE_ENABLE = 1;                // Value for enabling the torque
constexpr uint8_t TORQUE_DISABLE = 0;               // Value for disabling the torque
constexpr uint16_t DXL_MINIMUM_POSITION_VALUE = 280; // Dynamixel will rotate between this value
constexpr uint16_t DXL_MAXIMUM_POSITION_VALUE = 300; // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
constexpr uint8_t DXL_MOVING_STATUS_THRESHOLD = 10; // Dynamixel moving status threshold

int main(int argc, char const *argv[])
{
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        return 0;
    }
    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        return 0;
    }
    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;                                                  // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE}; // Goal position

    uint8_t dxl_error = 0;             // Dynamixel error
    uint16_t dxl_present_position = 0; // Present position
    while (1)
    {
        // Write goal position
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }
        else if (dxl_error != 0)
        {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        }

        do
        {
            // Read present position
            dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS)
            {
                printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            }

            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

        } while ((std::abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

        // Change goal position
        if (index == 0)
        {
            index = 1;
        }
        else
        {
            index = 0;
        }
    }

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    portHandler->closePort();
    return 0;
}
