#include <mrjank_control/control.h>

MrJankInterface::MrJankInterface() : 
  portHandler_{PortHandler::getPortHandler(DEVICE_NAME)},
  packetHandler_{PacketHandler::getPacketHandler(PROTOCOL_VERSION)} {}

bool MrJankInterface::read_(int id, uint16_t* data, ReadAddr addr) {

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_,id,addr,data);
  if (dxl_comm_result == COMM_SUCCESS) {
    std::cout << *data << "\n";
    return true;
  }
  else {
    ROS_ERROR("Failed to get ctrl table addr %d! Result: %d", addr, dxl_comm_result);
    return false;
  }
}

bool MrJankInterface::write_(int id, uint16_t data, WriteAddr addr) {

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_,id,addr,data);
  if (dxl_comm_result == COMM_SUCCESS) {
    return true;
  }
  else {
    ROS_ERROR("Failed to set ctrl table addr %d! Result: %d", addr, dxl_comm_result);
    return false;
  }
}

std::vector<double> MrJankInterface::read_arm_pos() {
  std::vector<double> jnts;
  for(int i = 0; i < CTRL_DIM; i++) {
    uint16_t data = 0;
    bool err = read_(JNT_IDS[i],&data,ADDR_PRESENT_POSITION);
    ROS_ASSERT_MSG(err,"Read position failed for id %d", i);
    jnts.push_back(data * DEG_PER_UNIT_VAL);
  }
  return jnts;
}

std::vector<double> MrJankInterface::read_arm_vel() {
  std::vector<double> jnts;
  for(int i = 0; i < CTRL_DIM; i++) {
    uint16_t data = 0;
    bool err = read_(JNT_IDS[i],&data,ADDR_PRESENT_SPEED);
    ROS_ASSERT_MSG(err,"Read velocity failed for id %d", i);
    jnts.push_back(data * RPM_PER_UNIT_VAL);
  }
  return jnts;
}