//
#ifndef PHOENIX_SENSOR_TASK_SAVE_DATA_TO_CSV_H_
#define PHOENIX_SENSOR_TASK_SAVE_DATA_TO_CSV_H_

#include <memory>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <iostream>
#include <fstream>
#include <ostream>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>

#include "math/math_utils.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"

#include "common/task.h"
#include "can_dev/can_driver.h"
#include "ad_msg.h"
#include <rosgraph_msgs/Clock.h>
#include "communication/shared_data.h"


namespace phoenix {
namespace sensor {


class TaskSaveDataToCsv : public framework::Task {
public:
  explicit TaskSaveDataToCsv(framework::Task* manager);
  ~TaskSaveDataToCsv();

  bool Start();
  bool Stop();

private:
  // 保存线程
  void ThreadSaveDataToCsv();

  // 获取build路径
  std::string GetProgramPath();

  // 判断目录是否存在
  void path_is_exist(std::string path);

  // 将时间戳转换为时间
  void TimeStampToLocalTime();
  
  // 在csv_data目录下创建csv文件
  void CreateCsvFile(Int32_t data_flag);

  // 保存数据到csv文件中
  void SaveDataCsvFile(Int32_t data_flag);

private:
  typedef boost::unique_lock<boost::mutex> Lock;

  boost::atomic_bool running_flag_save_data_to_csv_;
  boost::thread thread_save_data_to_csv_;

  struct TimeFlag {
    Uint64_t chassis_info_time[2];
    Uint64_t maxieye_lane_mark_time[2];
    Uint64_t maxieye_lane_curb_time[2];
    Uint64_t maxieye_objects_time[2];
    Uint64_t esr_front_objects_time[2];
    Uint64_t anngic_radar_front_left_objects_time[2];
    Uint64_t anngic_radar_front_right_objects_time[2];
    Uint64_t anngic_radar_rear_left_objects_time[2];
    Uint64_t anngic_radar_rear_right_objects_time[2];
    Uint64_t visual_control_front_objects_time[2];
    Uint64_t visual_control_front_left_objects_time[2];
    Uint64_t visual_control_front_right_objects_time[2];
    Uint64_t visual_control_rear_left_objects_time[2];
    Uint64_t visual_control_rear_right_objects_time[2];
    Uint64_t visual_control_rear_objects_time[2];
    Uint64_t visual_control_lane_mark_time[2];
    Uint64_t fusion_objects_time[2];
    Uint64_t adhmi_fusion_objects_time[2];
  }time_flag_;

  struct ObjectInfo {
    ad_msg::Gnss gnss_info_;
    ad_msg::Imu imu_info_;
    ad_msg::Chassis chassis_info;
    ad_msg::LaneMarkCameraList lane_mark_camera_list;
    ad_msg::LaneMarkCameraList lane_curb_camera_list;
    ad_msg::ObstacleCameraList obstacle_camera_list;
    ad_msg::ObstacleRadarList obstacle_esr_front_list;
    ad_msg::ObstacleCameraList obstacle_maxieye_camera_list;
    ad_msg::ObstacleLidarList obstacle_lidar_list_front;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_front_left_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_front_right_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_left_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_right_list;
    ad_msg::LaneMarkCameraList visual_control_lane_mark_camera_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_left_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_right_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_left_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_right_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_list;
    ad_msg::ObstacleList obstacle_fusion_list;
    ad_msg::ObstacleList adhmi_obstacle_fusion_list;
    rosgraph_msgs::Clock adhmi_clock;

    ObjectInfo() {
    }
  } object_info_;

  // 文件操作符
  struct OfstreamOutFile {
    std::ofstream chassis_info_out_file;
    std::ofstream maxieye_lane_mark_out_file;
    std::ofstream maxieye_lane_curb_out_file;
    std::ofstream maxieye_objects_out_file;
    std::ofstream esr_front_objects_out_file;
    std::ofstream anngic_radar_front_left_objects_out_file;
    std::ofstream anngic_radar_front_right_objects_out_file;
    std::ofstream anngic_radar_rear_left_objects_out_file;
    std::ofstream anngic_radar_rear_right_objects_out_file;
    std::ofstream visual_control_front_objects_out_file;
    std::ofstream visual_control_front_left_objects_out_file;
    std::ofstream visual_control_front_right_objects_out_file;
    std::ofstream visual_control_rear_left_objects_out_file;
    std::ofstream visual_control_rear_right_objects_out_file;
    std::ofstream visual_control_rear_objects_out_file;
    std::ofstream visual_control_lane_mark_out_file;
    std::ofstream fusion_objects_out_file;
    std::ofstream adhmi_fusion_objects_out_file;
  } out_files_;

  // 程序所在路径
  std::string current_program_path;

  // rosbag内部时间
  char local_time[120];

};


}  // namespace sensor
}  // namespace phoenix


#endif  //PHOENIX_SENSOR_TASK_SAVE_DATA_TO_CSV_H_
