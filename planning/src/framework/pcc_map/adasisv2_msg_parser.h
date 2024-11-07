/**
 * @file adasisv2_msg_parser.h
 * @author wangwh
 * @brief ADASISv2 CAN/ETH 消息解析为标准的结构体，方便PC及ADCU复用
 * @date 2023-08-14
 */

#ifndef PHOENIX_MAP_ADASISV2_MSG_PARSER_H_
#define PHOENIX_MAP_ADASISV2_MSG_PARSER_H_

#include "utils/macros.h"
#if (PROJECT_PLATFORM == PROJECT_PLATFORM_RCAR_M3N)
#include "pcc_map/adasis_v2.h"
#else
#include "can_dev/can_driver.h"
#include "pcc_map/adasis_v2.h"
#endif

namespace phoenix {
namespace adasisv2 {

#define USING_RAW_FRAME_ID (0)

/**
 * @brief ADASIS v2 CAN message parser
 * 参考cantools自动生成代码
 * 原始TBOX CAN ID改为ADECU的空闲ID
 * 
 */
/* Frame ids. */
#if USING_RAW_FRAME_ID
#define ADASIS_V2_ADAS_POSN_FRAME_ID (0x18F0F69Fu)
#define ADASIS_V2_ADAS_PROFSHORT_FRAME_ID (0x18F0F79Fu)
#else
#define ADASIS_V2_ADAS_POSN_FRAME_ID (0x18FF70DCu)
#define ADASIS_V2_ADAS_PROFSHORT_FRAME_ID (0x18FF7528u)
#endif

/* Frame lengths in bytes. */
#define ADASIS_V2_ADAS_PROFSHORT_LENGTH (8u)
#define ADASIS_V2_ADAS_POSN_LENGTH (8u)
/**
 * Signals in message ADAS_POSN.
 *
 * All signal values are as on the CAN bus.
 */
struct adasis_v2_adas_posn_t {
    /**
     * Range: -
     * Scale: 1.417
     * Offset: -179.959
     */
    uint8_t adas_posn_rel_head;

    /**
     * Range: 0..1023 (-51.1..51.2 %)
     * Scale: 0.1
     * Offset: -51.1
     */
    uint16_t adas_posn_slope;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_posn_cur_lane;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_posn_pos_confdc;

    /**
     * Range: -
     * Scale: 5
     * Offset: 0
     */
    uint16_t adas_posn_age;

    /**
     * Range: -
     * Scale: 3.333
     * Offset: 0
     */
    uint8_t adas_posn_pos_probb;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_posn_idx;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_posn_path_idx;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_posn_cyc_cnt;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t adas_posn_offset;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_posn_msg_typ;
};

/**
 * Signals in message ADAS_PROFSHORT.
 *
 * All signal values are as on the CAN bus.
 */
struct adasis_v2_adas_profshort_t {
    /**
     * Range: -
     * Scale: 0.1
     * Offset: -51.1
     */
    uint16_t adas_profshort_value1;

    /**
     * Range: -
     * Scale: 0.1
     * Offset: -51.1
     */
    uint16_t adas_profshort_value0;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t adas_profshort_dist1;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_accur_class;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_update;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_retr;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_ctrl_point;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_prof_type;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_path_idx;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_cyc_cnt;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint16_t adas_profshort_offset;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t adas_profshort_msg_typ;
};


class ADASISv2MsgParser {
public:
  ADASISv2MsgParser();
  ~ADASISv2MsgParser();

  MessageType ParseCanFrame(const can_dev::CanFrame& frame);

  const can_dev::CanFrame& getRawMsg() const { return can_frame_;}

  //TODO: 需要建立一个类体系？
  const PositionMessage& getPosition() const { return position_;}
  const ProfileShortMessage& getProfileShort() const { return profile_short_;}
  
private:

  // ADASIS v2 原始CAN数据
  phoenix::can_dev::CanFrame can_frame_;

  // CAN消息解析后的语义数据
  PositionMessage position_;
  ProfileShortMessage profile_short_;
};

}  // namespace map
}  // namespace phoenix


#endif  // PHOENIX_MAP_TASK_RECV_ADASISV2_MSG_H_
