#ifndef PHOENIX_FRAMEWORK_CHANGING_LANE_DEBUG_H_
#define PHOENIX_FRAMEWORK_CHANGING_LANE_DEBUG_H_

#include "utils/macros.h"
#include "utils/com_utils.h"
#include "common/message.h"


namespace phoenix {
namespace framework {


class ChangingLaneDebug {
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
  void Debug_Case_11();
  void Debug_Case_12();
  void Debug_Case_13();
  void Debug_Case_14();
  void Debug_Case_15();
  void Debug_Case_16();
  void Debug_Case_17();
  void Debug_Case_18();
  void Debug_Case_19();
  void Debug_Case_21();
  void Debug_Case_22();
  void Debug_Case_23();
  void Debug_Case_24();
  void Debug_Case_25();
  void Debug_Case_26();
  void Debug_Case_27();
  void Debug_Case_28();
  void Debug_Case_30();
  void Debug_Case_31();
  void Debug_Case_32();
  void Debug_Case_33();
  void Debug_Case_34();
  void Debug_Case_35();
  void Debug_Case_36();
  void Debug_Case_37();
  void Debug_Case_38();
  void Debug_Case_41();
  void Debug_Case_42();
  void Debug_Case_43();
  void Debug_Case_44();
  void Debug_Case_45();
  void Debug_Case_46();
  void Debug_Case_47();
  void Debug_Case_51();
  void Debug_Case_52();
  void Debug_Case_53();
  void Debug_Case_54();
  void Debug_Case_55();
  void Debug_Case_56();
  void Debug_Case_57();
  void Debug_Case_58();
  void Debug_Case_61();



private:
  // chassis
  ad_msg::Chassis chassis_;
  // Lane mark
  ad_msg::LaneMarkCameraList lane_mark_camera_list_;
  // obstacles
  ad_msg::ObstacleList obstacle_list_;
};


}  // namespace framework
}  // namespace framework


#endif  // PHOENIX_FRAMEWORK_CHANGING_LANE_DEBUG_H_

