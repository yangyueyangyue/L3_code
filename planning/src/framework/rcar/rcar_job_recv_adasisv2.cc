
#include "rcar/rcar_job_recv_adasisv2.h"
#include "pcc_map/adasisv2_msg_parser.h"
#include "communication/shared_data.h"


#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#ifdef __cplusplus
extern "C" {
#endif
#include "Sys_InfoCom.h"
#ifdef __cplusplus
}
#endif

#define ADASISV2_CANFRAME_LOG (0)

static phoenix::adasisv2::ADASISv2MsgParser s_adasisv2_parser;

void RCAR_Job_RecvADASISv2_DoJob() {
  phoenix::framework::SharedData *shared_data = phoenix::framework::SharedData::instance();

  // 处理POSITION CAN报文
  Tc397ToRcar_70DC data_70DC;
  sys_get_70DC(&data_70DC);

  can_dev::CanFrame frame;
  frame.data_len = 8;
  frame.id = ADASIS_V2_ADAS_POSN_FRAME_ID;
  frame.data[0] = data_70DC.Reserve_BYTE0;
  frame.data[1] = data_70DC.Reserve_BYTE1;
  frame.data[2] = data_70DC.Reserve_BYTE2;
  frame.data[3] = data_70DC.Reserve_BYTE3;
  frame.data[4] = data_70DC.Reserve_BYTE4;
  frame.data[5] = data_70DC.Reserve_BYTE5;
  frame.data[6] = data_70DC.Reserve_BYTE6;
  frame.data[7] = data_70DC.Reserve_BYTE7;

#ifdef ADASISV2_CANFRAME_LOG
  printf("[ADASIS] RAW POSN: timestamp[%ld], id[0x%08X], data[%02X %02X %02X %02X %02X %02X %02X %02X]\n", frame.time_stamp, 
            frame.id, frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#endif

  phoenix::adasisv2::MessageType msg_type = s_adasisv2_parser.ParseCanFrame(frame);
  if (phoenix::adasisv2::ADASIS_V2_MESSAGE_TYPE_POSITION == msg_type) {
    PositionMessage position = s_adasisv2_parser.getPosition();
    framework::MessageRecvADASISV2Position message(&position);
    shared_data->SetADASISV2Position(message);
#ifdef ADASISV2_CANFRAME_LOG
   printf("[ADASIS] POSITION : path = %d, offset = %d, slope = %f\n", position.path_index, position.offset, position.slope);
#endif
  }

  // 处理PROFILE SHORT CAN报文
  Tc397ToRcar_7258 data_7528;
  sys_get_7528(&data_7528);

  frame.id = ADASIS_V2_ADAS_PROFSHORT_FRAME_ID;
  frame.data[0] = data_7528.Reserve_BYTE0;
  frame.data[1] = data_7528.Reserve_BYTE1;
  frame.data[2] = data_7528.Reserve_BYTE2;
  frame.data[3] = data_7528.Reserve_BYTE3;
  frame.data[4] = data_7528.Reserve_BYTE4;
  frame.data[5] = data_7528.Reserve_BYTE5;
  frame.data[6] = data_7528.Reserve_BYTE6;
  frame.data[7] = data_7528.Reserve_BYTE7;

#ifdef ADASISV2_CANFRAME_LOG
  printf("[ADASIS] RAW PROFSHORT: timestamp[%ld], id[0x%08X], data[%02X %02X %02X %02X %02X %02X %02X %02X]\n", frame.time_stamp, 
            frame.id, frame.data[0], frame.data[1], frame.data[2], frame.data[3],
        frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
#endif

  phoenix::adasisv2::MessageType msg_type = s_adasisv2_parser.ParseCanFrame(frame);
  if (phoenix::adasisv2::ADASIS_V2_MESSAGE_TYPE_PROFILE_SHORT == msg_type) {
    ProfileShortMessage profile_short = s_adasisv2_parser.getProfileShort();
    framework::MessageRecvADASISV2ProfileShort message(&profile_short);
    shared_data->SetADASISV2ProfileShort(message);

#ifdef ADASISV2_CANFRAME_LOG
   printf("[ADASIS] PROFILE SHORT : type(1:curvature, 4 : slope) = %d, path = %d, offset = %d, value0 =  %f\n", profile_short.profile_type, profile_short.path_index, profile_short.offset, profile_short.value0);
#endif
  }
}

#endif