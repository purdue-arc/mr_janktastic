#include <ros/ros.h>

#include "std_msgs/String.h"
#include <dynamixel_sdk/dynamixel_sdk.h>

#pragma once

using namespace dynamixel;
// AX-12A 
// Control table address of 
#define ADDR_TORQUE_ENABLE    24
#define ADDR_PRESENT_LED      25

// Stall torque of AX-12A In N-m
#define MAX_TORQUE 1.5 

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE              1000000           // Default Baudrate of AX-12A 
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

// 
#define DEG_PER_UNIT_VAL 0.29
#define RPM_PER_UNIT_VAL 0.111

#define CTRL_DIM 7

const uint8_t JNT_IDS[CTRL_DIM] = { 1,2,3,4,5,6,7 };

enum WriteAddr {
   ADDR_GOAL_POSITION=30,
   MOVING_SPEED=32
};

enum ReadAddr {
   ADDR_PRESENT_POSITION=36,
   ADDR_PRESENT_SPEED=38,
   ADDR_PRESENT_LOAD=40,
   ADDR_PRESENT_TEMP=43,
   ADDR_PRESENT_VOLTAGE=43,
};


class MrJankInterface {
   public:
      MrJankInterface();

      std::vector<double> read_arm_pos();
      std::vector<double> read_arm_vel();
      bool read_arm_effort();

   private:
      PortHandler * portHandler_;
      PacketHandler * packetHandler_;

      bool read_(int id, uint16_t* val_addr, ReadAddr addr);
      bool write_(int id, uint16_t val, WriteAddr addr);


}; 