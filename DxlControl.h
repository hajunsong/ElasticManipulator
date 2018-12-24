#pragma once

#include <iostream>
#include <QtMath>
#include "dynamixel_sdk.h"

using namespace std;

enum{Red=513, Green, Blue};
enum{torque_mode=0, velocity_mode, position_mode=3, extended_position_mode, pwm_mode=16};

// Control table address
#define ADDR_TORQUE_ENABLE      512
#define ADDR_GOAL_POSITION      564
#define ADDR_GOAL_VELOCITY      552
#define ADDR_GOAL_TORQUE        550
#define ADDR_PRESENT_POSITION   580
#define ADDR_PRESENT_VELOCITY   576
#define ADDR_PRESENT_CURRENT    574
#define ADDR_INPUT_VOLTAGE      592
#define ADDR_OPERATING_MODE     11
#define ADDR_MOVING_THRESHOLD   24
#define ADDR_DELAY_TIME         9
#define ADDR_LED_RED            513
#define ADDR_LED_GREEN          514
#define ADDR_LED_BLUE           515
#define ADDR_CURRENT_LIMIT      38
#define ADDR_VELOCITY_PGAIN     526
#define ADDR_VELOCITY_IGAIN     524
#define ADDR_PROFILE_VEL        560
#define ADDR_PROFILE_ACC        556
#define ADDR_REALTIME_TICK      568
#define ADDR_POSITION_DGAIN     528
#define ADDR_POSITION_IGAIN     530
#define ADDR_POSITION_PGAIN     532
#define ADDR_BAUD_RATE          8
#define ADDR_BR_57600           1
#define ADDR_BR_1M              3
#define ADDR_BR_4M              6
#define ADDR_PRESENT_ENC1       770
#define ADDR_PRESENT_ENC2       774

// Protocol version
#define PROTOCOL_VERSION    2.0

#define DXL_ID_1    1
#define DXL_ID_2    2
#define DXL_ID_3    3
#define DXL_ID_4    4
#define BAUDRATE    1000000
#define DEVICENAME  "/dev/ttyUSB0"	// Is different in PC

#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0
#define OPERATING_TORQUE_CONTROL    0
#define OPERATING_VELOCITY_CONTROL  1
#define OPERATING_POSITION_CONTROL  3
#define VELOCITY_SCALE_FACTOR   0.00329218
#define TORQUE_CONSTANT     0.03187304
#define RESOLUTION  262143
#define GEAR_RATIO  303.75

class DxlControl
{
public:
    DxlControl();
    ~DxlControl();

    int dxl_comm_result;// = COMM_TX_FAIL;
    uint8_t dxl_error;// = 0;	// Dynamixel error
    uint32_t moving_threshold;// = 0;
    int16_t current_limit;

    void init();
    int dxl_init(uint8_t ID);
    void dxl_deinit();

    void setLEDon(int addr, uint8_t ID);
    void setLEDoff(uint8_t ID);
    void setInputTorque(int16_t input_torque, uint8_t ID);
    void setPosition(int32_t goal_position, uint8_t ID);
    void setVelocity(int32_t goal_velocity, uint8_t ID);
    void setOperateMode(uint8_t mode, uint8_t ID);
    void setHomePosition(uint8_t ID);

    double getDelayTime(uint8_t ID);
    int32_t getPresentPosition(uint8_t ID);
    int32_t getPresentVelocity(uint8_t ID);
    int16_t getPresentCurrent(uint8_t ID);
    double getPresentVoltage(uint8_t ID);
    uint8_t getOperateMode(uint8_t ID);
    int32_t getPresentEncoder1(uint8_t ID);
    int32_t getPresentEncoder2(uint8_t ID);

    void reset();
    uint8_t reset2();

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

private:

};

