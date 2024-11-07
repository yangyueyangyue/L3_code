//
#include "serial_msg_common_c.h"

#include "utils/log_c.h"
#include "utils/com_utils_c.h"
#include "utils/serialization_utils_c.h"


Int32_t Phoenix_Common_EncodeMsgHeadArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const MsgHead_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"valid\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeUint32Array(
          buf, offset + pos, maxlen - pos, &(p[i].sequence), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"sequence\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt64Array(
          buf, offset + pos, maxlen - pos, &(p[i].timestamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"timestamp\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeUint32Array(
          buf, offset + pos, maxlen - pos, &(p[i].src_module_id), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"src_module_id\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeUint32Array(
          buf, offset + pos, maxlen - pos, &(p[i].dst_module_id), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"dst_module_id\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeMsgHeadArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    MsgHead_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].valid), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"valid\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    Uint32_t sequence = 0;
    thislen = Phoenix_Common_DecodeUint32Array(
          buf, offset + pos, maxlen - pos, &(sequence), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"sequence\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
    /// Don't set value to "sequence" in "MsgHead".
    // p[i].sequence = sequence;

    thislen = Phoenix_Common_DecodeInt64Array(
          buf, offset + pos, maxlen - pos, &(p[i].timestamp), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"timestamp\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeUint32Array(
          buf, offset + pos, maxlen - pos, &(p[i].src_module_id), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"src_module_id\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeUint32Array(
          buf, offset + pos, maxlen - pos, &(p[i].dst_module_id), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"dst_module_id\" of MsgHead[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


/// 子模块ID
//Int32_t sub_module_id;
/// 模块状态类型
//Int32_t status;
/// 模块是否超时
//Int8_t timeout;
/// 自定义参数
//Int32_t param[4];
Int32_t Phoenix_Common_EncodeModuleStatusArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ModuleStatus_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].sub_module_id), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"sub_module_id\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"status\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].timeout), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"timeout\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].param[0]), 4);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"param\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeModuleStatusArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ModuleStatus_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].sub_module_id), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"sub_module_id\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].status), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"status\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt8Array(
          buf, offset + pos, maxlen - pos, &(p[i].timeout), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"timeout\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].param[0]), 4);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"param\" of ModuleStatus[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


/// 消息头
//MsgHead msg_head;
/// 模块状态的数量
//Int32_t module_status_num;
/// 模块状态列表
//ModuleStatus module_status_list[MAX_MODULE_STATUS_NUM];
Int32_t Phoenix_Common_EncodeModuleStatusListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ModuleStatusList_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_EncodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"msg_head\" of ModuleStatusList[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].module_status_num), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"module_status_num\" "
                "of ModuleStatusList[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_EncodeModuleStatusArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].module_status_list[0]), p[i].module_status_num);
    if (thislen < 0) {
      LOG_ERR_C("Failed to encode \"module_status_list\" "
                "of ModuleStatusList[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t Phoenix_Common_DecodeModuleStatusListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ModuleStatusList_t* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = Phoenix_Common_DecodeMsgHeadArray(
          buf, offset + pos, maxlen - pos, &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"msg_head\" of ModuleStatusList[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeInt32Array(
          buf, offset + pos, maxlen - pos, &(p[i].module_status_num), 1);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"module_status_num\" "
                "of ModuleStatusList[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = Phoenix_Common_DecodeModuleStatusArray(
          buf, offset + pos, maxlen - pos,
          &(p[i].module_status_list[0]), p[i].module_status_num);
    if (thislen < 0) {
      LOG_ERR_C("Failed to decode \"module_status_list\" "
                "of ModuleStatusList[%d].", i);
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


