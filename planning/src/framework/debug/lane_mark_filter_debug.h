#ifndef PHOENIX_FRAMEWORK_LANE_MARK_FILTER_DEBUG_H_
#define PHOENIX_FRAMEWORK_LANE_MARK_FILTER_DEBUG_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "common/message.h"


namespace phoenix {
namespace framework {


class LaneMarkFilterDebug {
public:
  void Debug();

private:
  void Debug_Case_01();
  void Debug_Case_02();
  void Debug_Case_03();
  void Debug_Case_04();
  void Debug_Case_05();
  void Debug_Case_06();
  void Debug_Case_07();
  void Debug_Case_08();
  void Debug_Case_09();

private:
  // chassis
  ad_msg::Chassis chassis_;
  // Lane mark
  ad_msg::LaneMarkCameraList lane_mark_camera_list_;
};


}  // namespace framework
}  // namespace framework


#endif  // PHOENIX_FRAMEWORK_LANE_MARK_FILTER_DEBUG_H_


