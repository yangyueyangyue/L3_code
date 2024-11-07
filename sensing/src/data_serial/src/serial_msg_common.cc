//
#include "serial_msg_common.h"

#include "utils/log.h"
#include "utils/serialization_utils.h"


namespace phoenix {
namespace data_serial {


Int32_t EncodeMsgHeadArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::MsgHead* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int8_t valid = p[i].valid;
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"valid\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].sequence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"sequence\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].timestamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"timestamp\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].src_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"src_module_id\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dst_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"dst_module_id\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeMsgHeadArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::MsgHead* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    Int8_t valid = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(valid), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"valid\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    p[i].valid = valid;

    Uint32_t sequence = 0;
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(sequence), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"sequence\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
    /// Don't set value to "sequence" in "MsgHead".
    // p[i].sequence = sequence;

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].timestamp), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"timestamp\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].src_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"src_module_id\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].dst_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"dst_module_id\" of MsgHead["
              << i << "].";
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
Int32_t EncodeModuleStatusArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ModuleStatus* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].sub_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"sub_module_id\" of ModuleStatus["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"status\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].timeout), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"timeout\" of ModuleStatus["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].param[0]), 4);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"param\" of ModuleStatus["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeModuleStatusArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ModuleStatus* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].sub_module_id), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"sub_module_id\" of ModuleStatus["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].status), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"status\" of MsgHead["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].timeout), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"timeout\" of ModuleStatus["
              << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].param[0]), 4);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"param\" of ModuleStatus["
              << i << "].";
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
Int32_t EncodeModuleStatusListArray(
    void* buf, Int32_t offset, Int32_t maxlen,
    const ad_msg::ModuleStatusList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = EncodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"msg_head\" of ModuleStatusList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::EncodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].module_status_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"module_status_num\" of ModuleStatusList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = EncodeModuleStatusArray(buf, offset + pos, maxlen - pos,
                                      &(p[i].module_status_list[0]),
                                      p[i].module_status_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"module_status_list\" of ModuleStatusList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}

Int32_t DecodeModuleStatusListArray(
    const void* buf, Int32_t offset, Int32_t maxlen,
    ad_msg::ModuleStatusList* p, Int32_t elements) {
  Int32_t pos = 0;
  Int32_t thislen = 0;

  for (Int32_t i = 0; i < elements; ++i) {
    thislen = DecodeMsgHeadArray(buf, offset + pos, maxlen - pos,
                                 &(p[i].msg_head), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"msg_head\" of ModuleStatusList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = common::DecodeValueArray(buf, offset + pos, maxlen - pos,
                                       &(p[i].module_status_num), 1);
    if (thislen < 0) {
      LOG_ERR << "Failed to encode \"module_status_num\" of ModuleStatusList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }

    thislen = DecodeModuleStatusArray(buf, offset + pos, maxlen - pos,
                                      &(p[i].module_status_list[0]),
                                      p[i].module_status_num);
    if (thislen < 0) {
      LOG_ERR << "Failed to decode \"module_status_list\" of ModuleStatusList[" << i << "].";
      return (thislen);
    } else {
      pos += thislen;
    }
  }

  return pos;
}


}  // namespace data_serial
}  // namespace phoenix


