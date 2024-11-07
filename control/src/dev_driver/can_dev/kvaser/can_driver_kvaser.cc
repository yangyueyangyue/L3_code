//
#include <string.h>
#include <stdio.h>
#include "can_dev/kvaser/can_driver_kvaser.h"
#include "utils/log.h"
#if (ENABLE_CAN_DEV_KVASER)
#include "canlib.h"
#endif


namespace phoenix {
namespace can_dev {


#if (ENABLE_CAN_DEV_KVASER)

static void Check(const char* id, canStatus stat) {
  if (canOK != stat) {
    char buf[64];
    buf[0] = '\0';
    canGetErrorText(stat, buf, sizeof(buf));
    LOG_ERR << id <<": failed, stat="
            << static_cast<int>(stat) << " (" << buf << ")";
  }
}

bool CanDriverKvaser::device_is_open_ = false;

CanDriverKvaser::CanDriverKvaser() {
  handle_read_ = -1;
  handle_write_ = -1;
}

CanDriverKvaser::~CanDriverKvaser() {
  CloseChannel();
}

bool CanDriverKvaser::OpenDevice() {
  if (device_is_open_) {
    return (true);
  }

  canInitializeLibrary();

  device_is_open_ = true;

  LOG_INFO(3) << "--------- Open Kvaser CAN device -------->";

  int beta = canGetVersionEx(canVERSION_CANLIB32_BETA);
  unsigned int can_lib_version = canGetVersionEx(canVERSION_CANLIB32_PRODVER);
  if (beta) {
    LOG_INFO(3) << "CANlib version " << (can_lib_version >> 8)
                << "." << (can_lib_version & 0xff)
                << " BETA";
  } else {
    LOG_INFO(3) << "CANlib version " << (can_lib_version >> 8)
                << "." << (can_lib_version & 0xff);
  }

  int chan_count = 0;
  canStatus stat = canGetNumberOfChannels(&chan_count);
  if (canOK != stat) {
    Check("canGetNumberOfChannels", stat);
    device_is_open_ = false;
    canUnloadLibrary();
    return (false);
  }
  LOG_INFO(3) << "CANlib found " << chan_count << " channel(s).";

  char name[256] = {0};
  char driver_name[256] = {0};
  char cust_chan_name[40] = {0};
  unsigned int ean[2] = {0};
  unsigned int fw[2] = {0};
  unsigned int serial[2] = {0};
  uint16_t file_version[4] = {0};
  bool ret = true;
  for (int i = 0; i < chan_count; ++i) {
    stat = canGetChannelData(
        i, canCHANNELDATA_DRIVER_NAME, &driver_name, sizeof(driver_name));
    if (stat != canOK) {
      Check("canGetChannelData: DRIVER_NAME", stat);
      ret = false;
      break;
    }
    stat = canGetChannelData(
        i, canCHANNELDATA_DLL_FILE_VERSION, &file_version,
        sizeof(file_version));
    if (stat != canOK) {
      Check("canGetChannelData: DLL_FILE_VERSION", stat);
      ret = false;
      break;
    }
    stat = canGetChannelData(
        i, canCHANNELDATA_DEVDESCR_ASCII, &name, sizeof(name));
    if (stat != canOK) {
      Check("canGetChannelData: DEVDESCR_ASCII", stat);
      ret = false;
      break;
    }
    if (0 == strcmp(name, "Kvaser Unknown")) {
      stat = canGetChannelData(
          i, canCHANNELDATA_CHANNEL_NAME, &name, sizeof(name));
      if (stat != canOK) {
        Check("canGetChannelData: CHANNEL_NAME", stat);
        ret = false;
        break;
      }
    }
    stat = canGetChannelData(i, canCHANNELDATA_CARD_UPC_NO, &ean, sizeof(ean));
    if (stat != canOK) {
      Check("canGetChannelData: CARD_UPC_NO", stat);
      ret = false;
      break;
    }
    stat = canGetChannelData(
        i, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial));
    if (stat != canOK) {
      Check("canGetChannelData: CARD_SERIAL_NO", stat);
      ret = false;
      break;
    }
    stat = canGetChannelData(
        i, canCHANNELDATA_CARD_FIRMWARE_REV, &fw, sizeof(fw));
    if (stat != canOK) {
      Check("canGetChannelData: CARD_FIRMWARE_REV", stat);
      ret = false;
      break;
    }
    canGetChannelData(i, canCHANNELDATA_CUST_CHANNEL_NAME,
        cust_chan_name, sizeof(cust_chan_name));

    LOG_INFO(3) << "ch[" << i << "]: name=\"" << name
                << "\" ean=\"" << (ean[1] >> 12)
                << "-" << (((ean[1] & 0xfff) << 8) | ((ean[0] >> 24) & 0xff))
                << "-" << ((ean[0] >> 4) & 0xfffff) << "-" << (ean[0] & 0x0f)
                << "\" s/n=\"" << serial[0]
                << "\" fw=\"" << (fw[1] >> 16) << "."
                << (fw[1] & 0xffff) << "." << (fw[0] >> 16)
                << "." << (fw[0] & 0xffff)
                << "\" cust_chan_name=\"" << cust_chan_name
                << "\" driver_name=\"" << driver_name
                << "\" file_version=\"" << file_version[3]
                << "." << file_version[2] << "." << file_version[1]
                << "\" ";
  }

  LOG_INFO(3) << "<-------- Open Kvaser CAN device ---------";

  if (false == ret) {
    device_is_open_ = false;
    canUnloadLibrary();
    return (false);
  }
  if (chan_count <= 0) {
    LOG_INFO(3) << "No divece have been found, please check Kvaser CAN Device.";
    device_is_open_ = false;
    canUnloadLibrary();
    return (false);
  }

  return (true);
}

bool CanDriverKvaser::CloseDevice() {
  if (device_is_open_) {
    device_is_open_ = false;
    canStatus stat = canUnloadLibrary();
    if (canOK != stat) {
      Check("canUnloadLIbrary", stat);
      return (false);
    }
  }

  return (true);
}

bool CanDriverKvaser::OpenChannel(const CanChannelParam& param) {
  if (!CreateCanHandle(param, handle_read_)) {
    LOG_ERR << "Failed to create CAN Device Handle for reading.";
    return false;
  }

  if (!CreateCanHandle(param, handle_write_)) {
    DestroyCanHandle(handle_read_);
    LOG_ERR << "Failed to create CAN Device Handle for writing.";
    return false;
  }

  return true;
}

bool CanDriverKvaser::CloseChannel() {
  DestroyCanHandle(handle_read_);
  DestroyCanHandle(handle_write_);

  return (true);
}

Int32_t CanDriverKvaser::Send(const CanFrame* frame, Int32_t frame_num) {
  if ((!device_is_open_) || (handle_write_ < 0)) {
    LOG_ERR << "This channel is not open";
    return (-1);
  }

  Int32_t send_count = 0;
  for (Int32_t i = 0; i < frame_num; ++i) {
    CanFrame can_frame = frame[i];

    unsigned int flag = 0;
    if (can_frame.RTR) {
      flag |= canMSG_RTR;
    }
    if (can_frame.EXT) {
      flag |= canMSG_EXT;
    } else {
      flag |= canMSG_STD;
    }
    canStatus stat = canWrite(
          handle_write_, can_frame.id, can_frame.data, can_frame.data_len, flag);
    if (canOK != stat) {
      Check("canWrite", stat);
      continue;
    }

    send_count++;
  }

  return (send_count);
}

Int32_t CanDriverKvaser::ReadWait(
    CanFrame* frame, Int32_t max_frame_num, Uint32_t timeout_ms) {
  if ((!device_is_open_) || (handle_read_ < 0) || (max_frame_num < 1)) {
    LOG_ERR << "This channel is closed or invalid parameter.";
    return (-1);
  }
  frame->data_len = 0;
  long id = 0;
  unsigned int  dlc = 0;
  unsigned int  flag = 0;
  unsigned long time = 0;
  canStatus stat = canReadWait(handle_read_, &id,
                               &(frame->data), &dlc,
                               &flag, &time, timeout_ms);
  frame->id = id;
  frame->data_len = dlc;
  frame->time_stamp = time;
  frame->EXT = false;
  if (flag & canMSG_EXT) {
    frame->EXT = true;
  }
  frame->RTR = false;
  if (flag & canMSG_RTR) {
    frame->RTR = true;
  }
  if (canOK != stat) {
    if (canERR_NOMSG == stat) {
      //
    } else {
      Check("canReadWait", stat);
    }
    return (-1);
  }

  return (1);
}

bool CanDriverKvaser::CreateCanHandle(
    const CanChannelParam& param, canHandle& handle) {
  if (!device_is_open_) {
    LOG_ERR << "CAN Divece has not been open.";
    return (false);
  }
  if (param.channel < 0) {
    LOG_ERR << "Invalid CAN channel.";
    return (false);
  }
  if (handle >= 0) {
    LOG_WARN << "This handle has already been open.";
    return (true);
  }

  handle = canOpenChannel(param.channel, canOPEN_ACCEPT_VIRTUAL);
  if (handle < 0) {
    Check("canOpenChannel", (canStatus)handle);
    return (false);
  }

  canStatus stat = canERR_PARAM;
  switch (param.bit_rate) {
  case (CAN_BIT_RATE_250K):
    stat = canSetBusParams(handle, canBITRATE_250K, 0, 0, 0, 0, 0);
    break;
  case (CAN_BIT_RATE_500K):
    stat = canSetBusParams(handle, canBITRATE_500K, 0, 0, 0, 0, 0);
    break;
  default:
    LOG_ERR << "Invalid CAN Bit Rate.";
    stat = canClose(handle);
    Check("canClose", stat);
    return (false);
  }

  if (canOK != stat) {
    Check("canSetBusParams", stat);
    stat = canClose(handle);
    return (false);
  }

  stat = canBusOn(handle);
  if (canOK != stat) {
    Check("canBusOn", stat);
    stat = canClose(handle);
    return (false);
  }

  return (true);
}

bool CanDriverKvaser::DestroyCanHandle(canHandle& handle) {
  if (handle >= 0) {
    canStatus stat = canWriteSync(handle, 1000);
    Check("canWriteSync", stat);
    stat = canBusOff(handle);
    Check("canBusOff", stat);
    stat = canClose(handle);
    Check("canClose", stat);
    handle = -1;
  }

  return (true);
}

#endif  // #if (ENABLE_CAN_DEV_KVASER)


}  // namespace can_dev
}  // namespace phoenix
