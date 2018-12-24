#include "DxlControl.h"

DxlControl::DxlControl()
{
}

DxlControl::~DxlControl()
{
}

void DxlControl::init(){
    dxl_comm_result = COMM_TX_FAIL;
    dxl_error = 0;	// Dynamixel error
    moving_threshold = 0;
    current_limit = 0;

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    }
    else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        return;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    }
    else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        return;
    }
}

int DxlControl::dxl_init(uint8_t ID)
{
    // Check Dynamixel Torque on or off
    int torque = 0;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, reinterpret_cast<uint8_t*>(&torque), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    else {
        printf("Dynamixel has been successfully connected\n");
    }

//    cout << "torque enable : " << torque << endl;
    if (torque == TORQUE_ENABLE) {
        // Disable Dynamixel Torque
        packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }

    // Write Dynamixel Baud rate (1M bps)
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_BAUD_RATE, ADDR_BR_1M, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED_RED, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED_GREEN, 255, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED_BLUE, 0, &dxl_error);

    // Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, OPERATING_POSITION_CONTROL, &dxl_error);

    //    int32_t goal_velocity = 2900;
    //    int32_t goal_acceleration = 2000;
    //    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_VELOCITY, static_cast<uint32_t>(goal_velocity), &dxl_error);
    //    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_VELOCITY, static_cast<uint32_t>(goal_velocity), &dxl_error);
    //    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_PROFILE_ACCELERATION, static_cast<uint32_t>(goal_acceleration), &dxl_error);

    //    uint16_t positionPGain = 200;
    //    uint16_t positionDGain = 50;
    //    uint16_t positionIGain = 10;
    //    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_PGAIN, positionPGain, &dxl_error);
    //    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_DGAIN, positionDGain, &dxl_error);
    //    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_POSITION_IGAIN, positionIGain, &dxl_error);

    // Enable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    int16_t present_current = 0;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_CURRENT, (uint16_t*)(&present_current), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("present current state : %s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0) {
        printf("present current state : %s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
    else {
        printf("present current state : Dynamixel has been successfully connected\n");
    }

    return 1;
}

void DxlControl::dxl_deinit()
{
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_1, ADDR_LED_RED, 255, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_1, ADDR_LED_GREEN, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_1, ADDR_LED_BLUE, 0, &dxl_error);
    // Reset home position
    int32_t goal_position = (int)(0*262143/360.0);
    int32_t pos = 0;
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID_1, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID_1, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
        printf("Goal pos : %d, Current pos : %d\n", 0, pos);
    } while (abs(goal_position - pos) > 200);
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, DXL_ID_2, ADDR_LED_RED, 255, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_2, ADDR_LED_GREEN, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_2, ADDR_LED_BLUE, 0, &dxl_error);
    // Reset home position
    pos = 0;
    goal_position = -22500 + (int)(-15*262143/360.0);
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID_2, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID_2, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
        printf("Goal pos : %d, Current pos : %d\n", goal_position, pos);
    } while (abs(goal_position - pos) > 200);
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, DXL_ID_3, ADDR_LED_RED, 255, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_3, ADDR_LED_GREEN, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_3, ADDR_LED_BLUE, 0, &dxl_error);
    // Reset home position
    pos = 0;
    goal_position = -86500;
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID_3, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID_3, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
        printf("Goal pos : %d, Current pos : %d\n", goal_position, pos);
    } while (abs(goal_position - pos) > 200);
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, DXL_ID_4, ADDR_LED_RED, 255, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_4, ADDR_LED_GREEN, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL_ID_4, ADDR_LED_BLUE, 0, &dxl_error);
    // Reset home position
    pos = 0;
    goal_position = 53950;
    do {
        packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, 4, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
        printf("Goal pos : %d, Current pos : %d\n", goal_position, pos);
    } while (abs(goal_position - pos) > 200);
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Close port
    portHandler->closePort();

    printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::reset(){
    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

uint8_t DxlControl::reset2(){
    packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    uint8_t state[4] = {0,};
    packetHandler->read1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, &state[0], &dxl_error);
    packetHandler->read1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, &state[1], &dxl_error);
    packetHandler->read1ByteTxRx(portHandler, 3, ADDR_TORQUE_ENABLE, &state[2], &dxl_error);
    packetHandler->read1ByteTxRx(portHandler, 4, ADDR_TORQUE_ENABLE, &state[3], &dxl_error);

    if(state[0] == 0 && state[1] == 0 && state[2] == 0 && state[3] == 0){
        return 1;
    }
    else {
        return 0;
    }
}

void DxlControl::setLEDon(int addr, uint8_t ID)
{
    setLEDoff(ID);
    packetHandler->write1ByteTxRx(portHandler, ID, (uint16_t)(addr), 255, &dxl_error);
}

void DxlControl::setLEDoff(uint8_t ID)
{
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED_RED, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED_GREEN, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_LED_BLUE, 0, &dxl_error);
}

void DxlControl::setInputTorque(int16_t input_torque, uint8_t ID)
{
    packetHandler->write2ByteTxRx(portHandler, ID, ADDR_GOAL_TORQUE, (uint16_t)(input_torque), &dxl_error);
}

void DxlControl::setPosition(int32_t goal_position, uint8_t ID)
{
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, (uint32_t)(goal_position), &dxl_error);
//    int32_t pos = 0;
//    do {
//        packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, goal_position, &dxl_error);
//        packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, (uint32_t*)(&pos), &dxl_error);
//        printf("Initial pos : %d, Current pos : %d\n", goal_position, pos);
//    } while (abs(pos - goal_position) > 100);
}

void DxlControl::setVelocity(int32_t goal_velocity, uint8_t ID)
{
    packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_VELOCITY, (uint32_t)(goal_velocity), &dxl_error);
}

void DxlControl::setOperateMode(uint8_t mode, uint8_t ID)
{
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, mode, &dxl_error);

    // Reset home position
    packetHandler->write1ByteTxRx(portHandler, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

void DxlControl::setHomePosition(uint8_t ID)
{
    int32_t pos = 0;
    do {
        packetHandler->write4ByteTxRx(portHandler, ID, ADDR_GOAL_POSITION, 0, &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, (uint32_t*)(&pos), &dxl_error);
        printf("Initial pos : %d, Current pos : %d\n", 0, pos);
    } while (abs(pos) > 200);
}

double DxlControl::getDelayTime(uint8_t ID)
{
    uint8_t delay_time = 0;
    packetHandler->read1ByteTxRx(portHandler, ID, ADDR_DELAY_TIME, &delay_time, &dxl_error);
    return delay_time * 0.002;
}

int32_t DxlControl::getPresentPosition(uint8_t ID)
{
    int32_t present_position = 0;
    packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_POSITION, (uint32_t*)(&present_position), &dxl_error);
    return present_position/* * 360.0 / RESOLUTION * M_PI/180.0*/;
}

int32_t DxlControl::getPresentVelocity(uint8_t ID)
{
    int32_t present_velocity = 0;
    packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_VELOCITY, (uint32_t*)(&present_velocity), &dxl_error);
    //    return static_cast<double>(present_velocity) * 0.01*6.0 * M_PI / 180.0;
    return present_velocity;
}

int16_t DxlControl::getPresentCurrent(uint8_t ID)
{
    int16_t present_current = 0;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, ADDR_PRESENT_CURRENT, (uint16_t*)(&present_current), &dxl_error);
//    if (dxl_comm_result != COMM_SUCCESS) {
//        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//        return 0;
//    }
//    else if (dxl_error != 0) {
//        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
//        return 0;
//    }
//    else {
//        printf("Dynamixel has been successfully connected\n");
//    }
    return present_current;// * 0.001;
}

double DxlControl::getPresentVoltage(uint8_t ID)
{
    uint16_t present_velocity = 0;
    packetHandler->read2ByteTxRx(portHandler, ID, ADDR_INPUT_VOLTAGE, &present_velocity, &dxl_error);
    return present_velocity * 0.1;
}

uint8_t DxlControl::getOperateMode(uint8_t ID)
{
    uint8_t operating_mode = 0;
    packetHandler->read1ByteTxRx(portHandler, ID, ADDR_OPERATING_MODE, &operating_mode, &dxl_error);
    cout << "operating mode : " << (int)operating_mode << endl;

    return operating_mode;
}

int32_t DxlControl::getPresentEncoder1(uint8_t ID){
    int32_t present_endcoder = 0;
    packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_ENC1, (uint32_t*)(&present_endcoder), &dxl_error);
    //    return static_cast<double>(present_velocity) * 0.01*6.0 * M_PI / 180.0;
    return present_endcoder;
}
int32_t DxlControl::getPresentEncoder2(uint8_t ID){
    int32_t present_endcoder = 0;
    packetHandler->read4ByteTxRx(portHandler, ID, ADDR_PRESENT_ENC2, (uint32_t*)(&present_endcoder), &dxl_error);
    //    return static_cast<double>(present_velocity) * 0.01*6.0 * M_PI / 180.0;
    return present_endcoder;
}
