//
#include "msg_common_c.h"


void Phoenix_AdMsg_ClearMsgHead(MsgHead_t* const msg_head) {
  msg_head->valid = 0;
  msg_head->sequence = 0;
  msg_head->timestamp = 0;
  msg_head->src_module_id = MODULE_ID_INVALID;
  msg_head->dst_module_id = MODULE_ID_INVALID;
}

void Phoenix_AdMsg_UpdateSequenceNum(MsgHead_t* const msg_head, Uint32_t step) {
  msg_head->sequence += step;
  if (msg_head->sequence >= AD_MSG_SEQUENCE_NUM_MODULO) {
    msg_head->sequence -= AD_MSG_SEQUENCE_NUM_MODULO;
  }
}

Uint32_t Phoenix_AdMsg_CalcSequenceDiff(Uint32_t prev, Uint32_t next) {
  Uint32_t diff = 0;
  if (next >= prev) {
    diff = next - prev;
  } else {
    diff = AD_MSG_SEQUENCE_NUM_MODULO - prev + next;
  }

  return (diff);
}

void Phoenix_AdMsg_ClearModuleStatus(ModuleStatus_t* const msg) {
  msg->sub_module_id = MODULE_ID_INVALID;
  msg->status = MODULE_STATUS_OK;
  phoenix_com_memset(msg->param, 0, sizeof(msg->param));
  msg->timeout = 0;
}

void Phoenix_AdMsg_ClearModuleStatusList(ModuleStatusList_t* const msg) {
  Phoenix_AdMsg_ClearMsgHead(&(msg->msg_head));
  msg->module_status_num = 0;
  for (Int32_t i = 0; i < AD_MSG_MAX_MODULE_STATUS_NUM; ++i) {
    Phoenix_AdMsg_ClearModuleStatus(&(msg->module_status_list[i]));
  }
}

Int32_t Phoenix_AdMsg_PushBackModuleStausToList(
    const ModuleStatus_t* status, ModuleStatusList_t* const list) {
  Int32_t ret = -1;
  if (list->module_status_num < AD_MSG_MAX_MODULE_STATUS_NUM) {
    phoenix_com_memcpy(&(list->module_status_list[list->module_status_num]),
                       status, sizeof(ModuleStatus_t));
    list->module_status_num++;

    ret = 0;
  }

  return (ret);
}
