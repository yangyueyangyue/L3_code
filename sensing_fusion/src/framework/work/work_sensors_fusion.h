//
#ifndef PHOENIX_FRAMEWORK_WORK_SENSORS_FUSION_H_
#define PHOENIX_FRAMEWORK_WORK_SENSORS_FUSION_H_

#include "utils/macros.h"
#include "utils/com_utils.h"

#include "driving_map.h"
#include "driving_map_wrapper.h"
#include "common/message.h"
#include "ad_msg.h"

#include "pos_filter/include/pos_filter_wrapper.h"

#include "around_radar_handler.h"

#include "visual_control_handler.h"

#include "main_camera_handler.h"

#include "main_radar_handler.h"


#define  ENABLE_CARRYING_STANDARD_TRAILER    (1)

#define  DEBUG_RADAR_FUSION_DISAPPEAR_PROBLEM  (0) //调试ADECU单雷达融合结果消失的问题






namespace phoenix {
namespace perception {
namespace sensorsfusion{

class ISensorsFusionMainstream;


} // namespace sensors fusion
} // namespace perception
} // namespace phoenix

namespace phoenix {
namespace framework {


using namespace phoenix::perception::radar;

using namespace phoenix::perception::visual;

using namespace phoenix::perception::main_camera;

using namespace phoenix::perception::main_radar;



class WorkSensorsFusion {
public:
   WorkSensorsFusion();
   ~WorkSensorsFusion();

   void Initialize();

   Int32_t DoWork();

   const ad_msg::ObstacleList& GetSensorsFusionObjsResult();

   // Add percetpion debug BY wzf for Ros
   const ::sensing_fusion::perception_debug& GetPerceptionDebug() const {
      return (perception_debug_);
   }

   void ConstitutePerceptionDebug(ad_msg::ObstacleList &obstacle_list);
private:

   void handle_around_radar_debug_data(sensing_fusion::obstacle_radar_list & output_radar_list, ad_msg::ObstacleRadarList & input_radar_list,  
                              ad_msg::ObstacleRadarList & input_radar_list_preprocess, int target_id_before, int target_id_after);

   // Gnss
   ad_msg::Gnss gnss_info_;
   // Imu
   ad_msg::Imu imu_info_;
   // Relative pos list
   ad_msg::RelativePosList relative_pos_list_;
   // pos_filter info
   pos_filter::PosFilterInfo pos_filter_info_;
   // Gnss track list
   ad_msg::RelativePosList gnss_track_list_;
   // Gnss filtered track list
   ad_msg::RelativePosList gnss_filtered_track_list_;
   // Utm track list
   ad_msg::RelativePosList utm_track_list_;
   // Utm filtered track list
   ad_msg::RelativePosList utm_filtered_track_list_;
   // Gnss
   ad_msg::Gnss filtered_gnss_info_;
   // chassis
   ad_msg::Chassis chassis_;
    // chassisinfo
   ad_msg::SpecialChassisInfo special_chassis_info_;

   // Lane mark
   ad_msg::LaneMarkCameraList lane_mark_main_forward_camera_list_;
   // Lane curb
   ad_msg::LaneMarkCameraList lane_curb_main_forward_camera_list_;
   // obstacles from front camera
   ad_msg::ObstacleCameraList obstacle_main_forward_camera_list_;
   // obstacles from front radar
   ad_msg::ObstacleRadarList obstacle_main_forward_radar_list_;

   // obstacles from left fw radar
   ad_msg::ObstacleRadarList obstacle_left_forward_radar_list_;
   // obstacles from right fw radar
   ad_msg::ObstacleRadarList obstacle_right_forward_radar_list_;
   // obstacles from left bw radar
   ad_msg::ObstacleRadarList obstacle_left_backward_radar_list_;
   // obstacles from right bw radar
   ad_msg::ObstacleRadarList obstacle_right_backward_radar_list_; 

   // obstacles from center camera
   ad_msg::ObstacleCameraList obstacle_center_forward_camera_list_;
   // obstacles from left side camera
   ad_msg::ObstacleCameraList obstacle_left_side_camera_list_;
   // obstacles from right side camera
   ad_msg::ObstacleCameraList obstacle_right_side_camera_list_;
   // obstacle from left backward camera 
   ad_msg::ObstacleCameraList obstacle_left_backward_camera_list_;
   // obstacle from right backward camera
   ad_msg::ObstacleCameraList obstacle_right_backward_camera_list_;


   // obstacles from lidar
   ad_msg::ObstacleLidarList obstacle_main_forward_lidar_list_;

   // tracked obstacles
   ad_msg::ObstacleTrackedInfoList obstacle_tracked_info_list_;
   // obstacles
   ad_msg::ObstacleList obstacle_list_;
   // Traffic signal
   ad_msg::TrafficSignalList traffic_signal_list_;
   // Traffic light
   ad_msg::TrafficLightList traffic_light_list_;

   // pos Filter
   pos_filter::PosFilterWrapper pos_filter_;

   exp_map_t::stMapGeofenceInfo geofence_info_;

   // obj Filter
   std::unique_ptr<perception::sensorsfusion::ISensorsFusionMainstream> objs_sensors_fusion_ptr_;

   // Driving map
   driv_map::DrivingMapWrapper driving_map_;

   // map for debug
   driv_map::DrivingMapInfo driving_map_info_;

   AroundRadarHandler* around_radar_handler_;

   VisualControlHandler* visual_control_handler_;

   MainCameraHandler* main_camera_handler_;

   MainRadarHandler* main_radar_handler_;

  // Add Perception debug BY wzf for Ros
  ::sensing_fusion::perception_debug perception_debug_;

   framework::PerceptionModuleStatus perception_module_status_;

#if DEBUG_RADAR_FUSION_DISAPPEAR_PROBLEM   
   int32_t test_radar_count;

   int32_t test_flag;
#endif

};


} // namespace framwork
} // namespace phoenix

#endif
