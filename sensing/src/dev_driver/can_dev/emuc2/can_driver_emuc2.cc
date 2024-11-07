//
#include <string.h>
#include <stdio.h>
#include "can_dev/emuc2/can_driver_emuc2.h"
#include "utils/log.h"
#if (ENABLE_CAN_DEV_EMUC2)
#include "lib_emuc_2.h"
#endif


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_EMUC2)

bool CanDriverEmuc2::device_is_open_ = false;
bool CanDriverEmuc2::channel_is_open_ = false;
int CanDriverEmuc2::device_port_ = 0;

CanDriverEmuc2::CanDriverEmuc2() {
}

CanDriverEmuc2::~CanDriverEmuc2() {
  CloseChannel();
}

bool CanDriverEmuc2::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  device_port_ = -1;
  for (int i = 0; i < MAX_COM_NUM; i++) {
    int ret = EMUCOpenDevice(i);
    if (0 == ret) {
      device_port_ = i;
      break;
    }
  }
  if (device_port_ < 0) {
    LOG_ERR << "No EMUC port available.";
    return (false);
  }

  device_is_open_ = true;

  EMUCInitCAN(device_port_, EMUC_INACTIVE, EMUC_INACTIVE);
  EMUCResetCAN(device_port_);
  EMUCClearFilter(device_port_, EMUC_CAN_1);
  EMUCClearFilter(device_port_, EMUC_CAN_2);
  EMUCSetErrorType(device_port_, EMUC_DIS_ALL);
  EMUCSetMode(device_port_, EMUC_NORMAL, EMUC_NORMAL);

  CFG_INFO cfg_info;
  int ret = EMUCGetCfg(device_port_, &cfg_info);
  if(ret) {
    LOG_ERR << "EMUC get config, failed!";
  } else {
    LOG_ERR << "EMUC get config, error set = " << cfg_info.err_set;
  }

  LOG_INFO(3) << "EMUC port = " << device_port_;
  LOG_INFO(3) << "--------- Open EMUC CAN device --------> [OK]";

  return (true);
}

bool CanDriverEmuc2::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;

    EMUCInitCAN(device_port_, EMUC_INACTIVE, EMUC_INACTIVE);
    int ret = EMUCCloseDevice(device_port_);
    LOG_INFO(3) << "EMUC close device, ret = " << ret;
  }

  return (true);
}

bool CanDriverEmuc2::OpenChannel(const CanChannelParam& param) {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }
  if (param.channel < 0) {
    LOG_ERR << "Invalid CAN channel.";
    return (false);
  }
  if (channel_is_open_) {
    return (true);
  }

  int can_baud_rate = 0;
  switch (param.bit_rate) {
  case (CAN_BIT_RATE_250K):
    can_baud_rate = EMUC_BAUDRATE_250K;
    break;
  case (CAN_BIT_RATE_500K):
    can_baud_rate = EMUC_BAUDRATE_500K;
    break;
  default:
    LOG_ERR << "Invalid CAN Bit Rate.";
    return (false);
  }

  EMUCSetBaudRate(device_port_, can_baud_rate, can_baud_rate);

  EMUCInitCAN(device_port_, EMUC_ACTIVE, EMUC_ACTIVE);

  channel_ = param.channel;

  channel_is_open_ = true;

  return (true);
}

bool CanDriverEmuc2::CloseChannel() {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }

  EMUCInitCAN(device_port_, EMUC_INACTIVE, EMUC_INACTIVE);

  channel_is_open_ = false;

  return (true);
}

Int32_t CanDriverEmuc2::Send(const CanFrame* frame, Int32_t frame_num) {
  if (!device_is_open_) {
    LOG_ERR << "This channel is not open";
    return (false);
  }

  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    CAN_FRAME_INFO can_frame_info;
    memset(&can_frame_info, 0, sizeof(can_frame_info));

    if (0 == channel_) {
      can_frame_info.CAN_port = EMUC_CAN_1;
    } else if (1 == channel_) {
      can_frame_info.CAN_port = EMUC_CAN_2;
    } else {
      can_frame_info.CAN_port = EMUC_CAN_1;
    }
    if (frame[i].RTR) {
      can_frame_info.rtr = EMUC_EN_RTR;
    } else {
      can_frame_info.rtr = EMUC_DIS_RTR;
    }
    if (frame[i].EXT) {
      can_frame_info.id_type = EMUC_EID;
    } else {
      can_frame_info.id_type = EMUC_SID;
    }
    can_frame_info.dlc = frame[i].data_len;
    can_frame_info.id = frame[i].id;
    for (Int32_t i = 0; i < DATA_LEN; ++i) {
      can_frame_info.data[i] = frame[i].data[i];
    }

    int ret = EMUCSend(device_port_, &can_frame_info);
    if (0 != ret) {
      LOG_ERR << "Send can failed, port=" << device_port_
              << ",channel=" << channel_
              << ",err="
              << (Char_t*)(&(can_frame_info.data_err[can_frame_info.CAN_port][0]));
      continue;
    }
    send_count++;
  }

  return (send_count);
}

Int32_t CanDriverEmuc2::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout) {
  if (!device_is_open_ || (max_frame_num < 1)) {
    LOG_ERR << "This channel is closed or invalid parameter.";
    return (-1);
  }

  CAN_FRAME_INFO can_frame_info;
  memset(&can_frame_info, 0, sizeof(can_frame_info));

  int ret = EMUCReceive(device_port_, &can_frame_info);
  if (1 != ret) {
    return (-1);
  }
  bool msg_ok = false;
  switch (can_frame_info.msg_type) {
  case (EMUC_DATA_TYPE):
    msg_ok = true;
    break;
  case (EMUC_EEERR_TYPE):
    LOG_ERR << "EMUC Receive, EEPROM Error !";
    break;
  case (EMUC_BUSERR_TYPE):
    LOG_ERR << "EMUC Receive, Bus error, err="
            << (Char_t*)(&(can_frame_info.data_err[can_frame_info.CAN_port][0]));
    break;
  default:
    LOG_ERR << "EMUC Receive, Invalid message type !";
    break;
  }

  if (msg_ok) {
    frame->EXT = false;
    if (EMUC_EID == can_frame_info.id_type) {
      frame->EXT = true;
    }
    frame->RTR = false;
    if (EMUC_EN_RTR == can_frame_info.rtr) {
      frame->RTR = true;
    }
    frame->id = can_frame_info.id;
    frame->data_len = can_frame_info.dlc;
    for (Int32_t i = 0; i < DATA_LEN; ++i) {
      frame->data[i] = can_frame_info.data[i];
    }
  } else {
    return (-1);
  }

  return (1);
}

#endif  // #if (ENABLE_CAN_DEV_EMUC2)


}  // namespace can_dev
}  // namespace phoenix
