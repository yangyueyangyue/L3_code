//
#ifndef PHOENIX_FRAMEWORK_WORK_PLANNING_H_
#define PHOENIX_FRAMEWORK_WORK_PLANNING_H_

#include "utils/macros.h"
#include "utils/com_utils.h"

#include "pos_filter_wrapper.h"
#include "obj_filter_wrapper.h"
#include "driving_map.h"
#include "driving_map_wrapper.h"
#include "motion_planning.h"
#include "action_planning_wrapper.h"
#include "trajectory_planning_wrapper.h"
#include "velocity_planning_wrapper.h"
#include "common/message.h"

#define ENABLE_WORK_PLANNING_DEBUGGING_FILE (0)

#if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
#include "debug/planning_debug.h"
#endif


#if (ENABLE_ROS_NODE)
// BY ZQ
#include "planning/plan_debug.h"
#include "planning/adecu_debug.h"
#endif

#ifdef ENABLE_PACC_ADASIS
#include "pcc_map/adasisv2_horizon_reconstructor.h"
#endif

namespace phoenix {
namespace framework {


class WorkPlanning {
public:
  WorkPlanning();
  ~WorkPlanning();

  void Initialize();

  void Configurate(const PlanningConfig& conf);
  const PlanningConfig& GetConfiguration() const {
    return (config_);
  }

  void SetPlanningStorys(const ad_msg::PlanningStoryList& storys) {
    planning_story_list_ = storys;
  }

  Int32_t DoWork();

  const ad_msg::PlanningResult& GetPlanningResult() const {
    return (planning_result_);
  }

  const ad_msg::ObstacleList& GetObstacleList() const {
    return (obstacle_list_);
  }

#if (ENABLE_ROS_NODE)  
  // Add Planning debug BY ZQ for Ros
  const ::planning::plan_debug& GetPlanningDebug() const {
    return (planning_debug_);
  }

  const ::planning::adecu_debug& GetAdecuPlanDebug() const {
    return (adecu_plan_debug_);
  }
#ifdef ENABLE_PACC_ADASIS
  const adasisv2::Horizon* GetADASISv2Horizon() const {
    return av2HR_.GetHorizon();
  }
#endif
#endif

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  bool IsLocalizationUpdated() const {
    return (localization_updated_flag_);
  }

  bool IsMapUpdated() const {
    return (map_updated_flag_);
  }

  const ad_msg::Gnss& GetGnssInfo() const {
    return (gnss_info_);
  }
#endif

private:
  class Button {
  public:
    Button() {
      Clear();
    }

    void Clear() {
      init_flag_ = false;
      prev_status_ = 0;
      button_on_count_ = 0;
      button_off_count_ = 0;
      button_filter_count_ = 0;
    }

    void Update(Int32_t status, bool* rising_edge, bool* falling_edge) {
      *rising_edge = false;
      *falling_edge = false;
      if (!init_flag_) {
        init_flag_ = true;
        prev_status_ = status;
        return;
      }

      if ((0 == prev_status_) && (1 == status)) {
        button_on_count_ = 1;
        button_filter_count_ = 0;
      }
      if ((1 == prev_status_) && (0 == status)) {
        button_off_count_ = 1;
        button_filter_count_ = 0;
      }
      if ((button_on_count_ > 0) && (1 == status)) {
        button_on_count_++;
        if (button_on_count_ > 1) {
          button_on_count_ = 0;
          *rising_edge = true;
        }
      }
      if ((button_off_count_ > 0) && (0 == status)) {
        button_off_count_++;
        if (button_off_count_ > 1) {
          button_off_count_ = 0;
          *falling_edge = true;
        }
      }

      button_filter_count_++;
      if (button_filter_count_ > 2) {
        button_filter_count_ = 2;
        if (button_on_count_ > 0) {
          button_on_count_ = 0;
        }
        if (button_off_count_ > 0) {
          button_off_count_ = 0;
        }
      }

      prev_status_ = status;
    }

  private:
    bool init_flag_;
    Int32_t prev_status_;
    Int32_t button_on_count_;
    Int32_t button_off_count_;
    Int32_t button_filter_count_;
  };

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  class HdMapBuffer {
  public:
    HdMapBuffer() { curr_buffer_idx_ = 0; }
    ~HdMapBuffer() {}

    const exp_map_t::stMapExpLocation& GetCurrLocalization() const {
      return (buffer_[curr_buffer_idx_].localization);
    }

    exp_map_t::stMapExpLocation& GetCurrLocalization() {
      return (buffer_[curr_buffer_idx_].localization);
    }

    const exp_map_t::stMapExpProfileInfo& GetCurrMap() const {
      return (buffer_[curr_buffer_idx_].map);
    }

    exp_map_t::stMapExpProfileInfo& GetCurrMap() {
      return (buffer_[curr_buffer_idx_].map);
    }

    const exp_map_t::stMapExpLocation& GetNextLocalization() const {
      Int32_t idx = curr_buffer_idx_ + 1;
      if (idx > 1) {
        idx = 0;
      }
      return (buffer_[idx].localization);
    }

    exp_map_t::stMapExpLocation& GetNextLocalization() {
      Int32_t idx = curr_buffer_idx_ + 1;
      if (idx > 1) {
        idx = 0;
      }
      return (buffer_[idx].localization);
    }

    const exp_map_t::stMapExpProfileInfo& GetNextMap() const {
      Int32_t idx = curr_buffer_idx_ + 1;
      if (idx > 1) {
        idx = 0;
      }
      return (buffer_[idx].map);
    }

    exp_map_t::stMapExpProfileInfo& GetNextMap() {
      Int32_t idx = curr_buffer_idx_ + 1;
      if (idx > 1) {
        idx = 0;
      }
      return (buffer_[idx].map);
    }

    void SwapBuffer() {
      curr_buffer_idx_++;
      if (curr_buffer_idx_ > 1) {
        curr_buffer_idx_ = 0;
      }
    }

    bool CheckConsistent(Int32_t buff_idx) const {
      const exp_map_t::stMapExpLocation* localization =
          &(buffer_[curr_buffer_idx_].localization);
      const exp_map_t::stMapExpProfileInfo* map =
          &(buffer_[curr_buffer_idx_].map);
      if (0 != buff_idx) {
        Int32_t idx = curr_buffer_idx_ + 1;
        if (idx > 1) {
          idx = 0;
        }
        localization = &(buffer_[idx].localization);
        map = &(buffer_[idx].map);
      }

      return ((localization->location_info.s_AbsolutePosition.m_datumTimestamp ==
              map->profile_datum_info.m_timestamp) &&
              (localization->location_info.s_AbsolutePosition.m_datumTimestamp != 0) &&
              (map->profile_datum_info.m_timestamp != 0));
    }

    bool IsCoorChanged() const {
      const exp_map_t::stMapExpProfileInfo& curr_map = GetCurrMap();
      const exp_map_t::stMapExpProfileInfo& next_map = GetNextMap();

      return ((curr_map.profile_datum_info.m_timestamp !=
               next_map.profile_datum_info.m_timestamp) &&
              (curr_map.profile_datum_info.m_timestamp != 0) &&
              (next_map.profile_datum_info.m_timestamp != 0));
    }

    bool IsLocationChanged() {
      const exp_map_t::stMapExpLocation& curr_location = GetCurrLocalization();
      const exp_map_t::stMapExpLocation& next_location = GetNextLocalization();

      return ((curr_location.location_info.s_AbsolutePosition.m_timestamp !=
               next_location.location_info.s_AbsolutePosition.m_timestamp) &&
              (next_location.location_info.s_AbsolutePosition.m_timestamp) != 0);
    }

    bool IsMapChanged() {
      const exp_map_t::stMapExpProfileInfo& curr_map = GetCurrMap();
      const exp_map_t::stMapExpProfileInfo& next_map = GetNextMap();

      return ((curr_map.profile_datum_info.m_profiletimestamp !=
               next_map.profile_datum_info.m_profiletimestamp) &&
              (next_map.profile_datum_info.m_profiletimestamp != 0));
    }

  private:
    struct Buffer {
      exp_map_t::stMapExpLocation localization;
      exp_map_t::stMapExpProfileInfo map;
    };

  private:
    Int32_t curr_buffer_idx_;
    Buffer buffer_[2];
  };
#endif

private:
  void UpdateVehicleParameter();

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  void GetMapDataFromEHR();
#endif

  void SetPlanningResult();
  void SetPlanningStatus();

  void GetPlanningSettings();
  void UpdatePlanningSettings(
      const ad_msg::PlanningSettings& new_settings,
      ad_msg::PlanningSettings* const cur_settings);
  void TmpFuncAddLidarObj(ad_msg::ObstacleList* obj_list);
  void TmpFuncAddLidarObjImp(
      const ad_msg::ObstacleLidarList& src_list,
      ad_msg::ObstacleList* dst_list);
#if (ENABLE_ROS_NODE)      
  void SetVelocityPlanningInternalDebug(const planning::VelocityPlanningInternal& data);
#endif

private:
  // Config
  PlanningConfig config_;
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
  // Odom track list
  ad_msg::RelativePosList odom_track_list_;
  // Odom filtered track list
  ad_msg::RelativePosList odom_filtered_track_list_;
  // Gnss
  ad_msg::Gnss filtered_gnss_info_;
  // chassis
  ad_msg::Chassis chassis_;
  // chassis control command
  ad_msg::ChassisCtlCmd chassis_ctl_cmd_;
  // chassisinfo
  ad_msg::SpecialChassisInfo special_chassis_info_;
  // hmisettings
  ad_msg::PlanningSettings curr_planning_settings_;
  ad_msg::PlanningSettings local_planning_settings_;
  ad_msg::PlanningSettings remote_planning_settings_;
  // Lane mark
  ad_msg::LaneMarkCameraList lane_mark_camera_list_;
  // obstacles from camera
  ad_msg::ObstacleCameraList obstacle_camera_list_;
  // obstacles from front esr
  ad_msg::ObstacleRadarList obstacle_radar_front_list_;
  // obstacles from rear esr
  ad_msg::ObstacleRadarList obstacle_radar_rear_list_;
  // obstacles from srr2
  ad_msg::ObstacleRadarList obstacle_radar_front_left_list_;
  ad_msg::ObstacleRadarList obstacle_radar_front_right_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_left_list_;
  ad_msg::ObstacleRadarList obstacle_radar_rear_right_list_;
  // obstacles from lidar
  ad_msg::ObstacleLidarList obstacle_lidar_list_0_;
  ad_msg::ObstacleLidarList obstacle_lidar_list_1_;
  // tracked obstacles
  ad_msg::ObstacleTrackedInfoList obstacle_tracked_info_list_;
  // obstacles
  ad_msg::ObstacleList obstacle_list_;
  // Traffic signal
  ad_msg::TrafficSignalList traffic_signal_list_;
  // Traffic light
  ad_msg::TrafficLightList traffic_light_list_;
  // Scene storys
  ad_msg::SceneStoryList scene_story_list_;
  // Map localization
  ad_msg::MapLocalization map_localization_;
  // adecu_vel_plan_debug
  planning::VelocityPlanningResult adecu_vel_plan_debug_;

  //log_file_pointXY:  For debugging
#if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
  Char_t str_buff_[1024*2];
  common::LogFile log_file_pointXY;
  bool open_log_file_pointXY= false;
#endif

#if HD_MAP_TYPE == HD_MAP_TYPE_D17
  bool localization_updated_flag_;
  bool map_updated_flag_;
  HdMapBuffer hd_map_buffer_;

  bool odom_coor_changed_flag_;
  Float32_t delta_heading_odom_coor_ = 0.0F;
  common::Matrix<Float64_t, 3, 3> mat_conv_odom_coor_;
  Int32_t is_valid_hd_map_ = 1;
#endif

  // pos Filter
  pos_filter::PosFilterWrapper pos_filter_;
  // obj Filter
  obj_filter::ObjFilterWrapper obj_filter_;
  // Driving map
  driv_map::DrivingMapWrapper driving_map_;
  // Action planner
  planning::ActionPlanningWrapper action_planner_;
  // Trajectory planner
  planning::TrajectoryPlanningWrapper trajectory_planner_;
  // Velocity planner
  planning::VelocityPlanningWrapper velocity_planner_;

  planning::ActionPlanningResult action_planning_result_;
  planning::TrajectoryPlanningResult trajectory_planning_result_;
  planning::VelocityPlanningResult velocity_planning_result_;

  planning::VelocityPlanningInternal velocity_planning_internal_;
  // map for debug
  driv_map::DrivingMapInfo driving_map_info_;
  // trajectory planning information for debug
  planning::TrajectoryPlanningInfo trj_planning_info_;

  ad_msg::PlanningResult planning_result_;
  common::Path target_trajectory_;
  common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum> target_trajectory_sample_points_;

#if (USING_SRR2_DETECTIONS)
  ad_msg::ObstacleRadarList srr2_detections_front_left_list_;
  ad_msg::ObstacleRadarList srr2_detections_front_right_list_;
  ad_msg::ObstacleRadarList srr2_detections_rear_left_list_;
  ad_msg::ObstacleRadarList srr2_detections_rear_right_list_;
  FilterSrr2Detections filter_srr2_detections_;
#endif

  // buttons for FT-Auman in TJA/HWA/I-Drive functions.
  Button button_start_hwa_;
  Button button_start_i_drive_;

  Button button_changing_lane_req_left_;
  Button button_changing_lane_req_right_;

  // Planning story list
  ad_msg::PlanningStoryList planning_story_list_;

  // Event reporting
  ad_msg::EventReporting event_reporting_list_[10];

  // Add Planning debug BY ZQ for Ros
  #if (ENABLE_ROS_NODE)
  ::planning::plan_debug planning_debug_;
  ::planning::adecu_debug adecu_plan_debug_;
  #endif

#if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
  // Debug
  PlanningDebug planning_info_debug_;
#endif
  
#if (ENABLE_WORK_PLANNING_DEBUGGING_FILE)
  Char_t log_file_buff_[1024*2];
  common::LogFile log_file_localization_;
#endif

#ifdef ENABLE_PACC_ADASIS
  adasisv2::HorizonReconstructor av2HR_;
#endif

};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_WORK_PLANNING_H_

