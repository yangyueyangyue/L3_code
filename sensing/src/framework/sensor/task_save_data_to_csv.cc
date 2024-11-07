//
#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "sensor/task_save_data_to_csv.h"
#include "communication/shared_data.h"
#include "parse_sensor_data_common.h"

#include <cmath>
#include <deque>
#include <fstream>

#define CANFRAME_LOG_FLAG 0
#define LANE_LOG_FLAG 0
#define OBSTACLE_LOG_FLAG 0
#define TRAFFICE_LOG_FLAG 0
#define VISUAL_CONTROL_LOG_FLAG 0
#define VISUAL_CONTROL_LANE_LOG_FLAG 0

#if (ENABLE_CAN_DEV_ZLGCANNET)
#include "can_dev/zlg_can_net/can_driver_zlgcannet_fd.h"
#endif


namespace phoenix {
namespace sensor {

TaskSaveDataToCsv::TaskSaveDataToCsv(framework::Task *manager)
  : framework::Task(
      framework::TASK_ID_SAVE_DATA_TO_CSV, "Save Csv", manager) {
  running_flag_save_data_to_csv_ = false;

  time_t now;
  time(&now);
  tm* p = localtime(&now);
  //printf("%04d:%02d:%02d %02d:%02d:%02d\n", p->tm_year+1900, p->tm_mon+1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);

  current_program_path = GetProgramPath() + "/csv_data/";
  path_is_exist(current_program_path);

  char current_time[100];
  sprintf(current_time, "%04d_%02d_%02d_%02d_%02d_%02d", p->tm_year+1900, p->tm_mon+1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
  current_program_path += current_time;
  path_is_exist(current_program_path);

}

TaskSaveDataToCsv::~TaskSaveDataToCsv() {
  Stop();

}

bool TaskSaveDataToCsv::Start() {
  if (running_flag_save_data_to_csv_) {
    return (true);
  }

  LOG_INFO(3) << "Create thread of save data to csv...";
  running_flag_save_data_to_csv_ = true;
  thread_save_data_to_csv_ = boost::thread(boost::bind(&TaskSaveDataToCsv::ThreadSaveDataToCsv, this));

  LOG_INFO(3) << "Create thread of save data to csv... [OK]";

  return (true);
}

bool TaskSaveDataToCsv::Stop() {
  if (running_flag_save_data_to_csv_) {
    running_flag_save_data_to_csv_ = false;

    out_files_.chassis_info_out_file.close();
    out_files_.maxieye_lane_mark_out_file.close();
    out_files_.maxieye_lane_curb_out_file.close();
    out_files_.maxieye_objects_out_file.close();
    out_files_.esr_front_objects_out_file.close();
    out_files_.anngic_radar_front_left_objects_out_file.close();
    out_files_.anngic_radar_front_right_objects_out_file.close();
    out_files_.anngic_radar_rear_left_objects_out_file.close();
    out_files_.anngic_radar_rear_right_objects_out_file.close();
    out_files_.visual_control_front_objects_out_file.close();
    out_files_.visual_control_front_left_objects_out_file.close();
    out_files_.visual_control_front_right_objects_out_file.close();
    out_files_.visual_control_rear_left_objects_out_file.close();
    out_files_.visual_control_rear_right_objects_out_file.close();
    out_files_.visual_control_rear_objects_out_file.close();
    out_files_.visual_control_lane_mark_out_file.close();
    out_files_.fusion_objects_out_file.close();
    out_files_.adhmi_fusion_objects_out_file.close();

    LOG_INFO(3) << "Stop thread of save data to csv ...";
    bool ret = thread_save_data_to_csv_.timed_join(boost::posix_time::seconds(1));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread of save data to csv to stop.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop thread of save data to csv ... [OK]";
    } else {
      LOG_INFO(3) << "Stop thread of save data to csv ... [NG]";
    }
  }

  return (true);
}

void TaskSaveDataToCsv::ThreadSaveDataToCsv() {
  LOG_INFO(3) << "Thread of save data to csv ... [Started]";

  while (running_flag_save_data_to_csv_) { 
    phoenix::framework::SharedData* shared_data =
        phoenix::framework::SharedData::instance();
    shared_data->GetADHMIClock(&object_info_.adhmi_clock);
    TimeStampToLocalTime();

    shared_data->GetChassis(&object_info_.chassis_info);
    time_flag_.chassis_info_time[0] = object_info_.chassis_info.msg_head.timestamp;
    if (time_flag_.chassis_info_time[0] != time_flag_.chassis_info_time[1]) {
      time_flag_.chassis_info_time[1] = time_flag_.chassis_info_time[0];
      CreateCsvFile(framework::MSG_ID_CHASSIS);
      SaveDataCsvFile(framework::MSG_ID_CHASSIS);
    }

    shared_data->GetLaneMarkCameraList(&object_info_.lane_mark_camera_list);
    time_flag_.maxieye_lane_mark_time[0] = object_info_.lane_mark_camera_list.msg_head.timestamp;
    if (time_flag_.maxieye_lane_mark_time[0] != time_flag_.maxieye_lane_mark_time[1]) {
      time_flag_.maxieye_lane_mark_time[1] = time_flag_.maxieye_lane_mark_time[0];
      CreateCsvFile(framework::MSG_ID_LANE_MARK_CAMERA_LIST);
      SaveDataCsvFile(framework::MSG_ID_LANE_MARK_CAMERA_LIST);
    }

    shared_data->GetLaneCurbCameraList(&object_info_.lane_curb_camera_list);
    time_flag_.maxieye_lane_curb_time[0] = object_info_.lane_curb_camera_list.msg_head.timestamp;
    if (time_flag_.maxieye_lane_curb_time[0] != time_flag_.maxieye_lane_curb_time[1]) {
      time_flag_.maxieye_lane_curb_time[1] = time_flag_.maxieye_lane_curb_time[0];
      CreateCsvFile(framework::MSG_ID_LANE_CURB_CAMERA_LIST);
      SaveDataCsvFile(framework::MSG_ID_LANE_CURB_CAMERA_LIST);
    }


    shared_data->GetObstacleEsrFrontList(&object_info_.obstacle_esr_front_list);
    time_flag_.esr_front_objects_time[0] = object_info_.obstacle_esr_front_list.msg_head.timestamp;
    if (time_flag_.esr_front_objects_time[0] != time_flag_.esr_front_objects_time[1]) {
      time_flag_.esr_front_objects_time[1] = time_flag_.esr_front_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_ESR_DATA_FRONT);
      SaveDataCsvFile(framework::MSG_ID_RECV_ESR_DATA_FRONT);
    }

    shared_data->GetObstacleMaxieyeCameraList(&object_info_.obstacle_maxieye_camera_list);
    time_flag_.maxieye_objects_time[0] = object_info_.obstacle_maxieye_camera_list.msg_head.timestamp;
    if (time_flag_.maxieye_objects_time[0] != time_flag_.maxieye_objects_time[1]) {
      time_flag_.maxieye_objects_time[1] = time_flag_.maxieye_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT);
      SaveDataCsvFile(framework::MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT);
    }

    shared_data->GetVisualControlLaneMarkCameraList(&object_info_.visual_control_lane_mark_camera_list);
    time_flag_.visual_control_lane_mark_time[0] = object_info_.visual_control_lane_mark_camera_list.msg_head.timestamp;
    if (time_flag_.visual_control_lane_mark_time[0] != time_flag_.visual_control_lane_mark_time[1]) {
      time_flag_.visual_control_lane_mark_time[1] = time_flag_.visual_control_lane_mark_time[0];
      CreateCsvFile(framework::MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST);
      SaveDataCsvFile(framework::MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST);
    }
    
    shared_data->GetObstacleAnngicRadarFrontLeftList(&object_info_.obstacle_anngic_radar_front_left_list);
    time_flag_.anngic_radar_front_left_objects_time[0] = object_info_.obstacle_anngic_radar_front_left_list.msg_head.timestamp;
    if (time_flag_.anngic_radar_front_left_objects_time[0] != time_flag_.anngic_radar_front_left_objects_time[1]) {
      time_flag_.anngic_radar_front_left_objects_time[1] = time_flag_.anngic_radar_front_left_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT);
      SaveDataCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT);
    }

    shared_data->GetObstacleAnngicRadarFrontRightList(&object_info_.obstacle_anngic_radar_front_right_list);
    time_flag_.anngic_radar_front_right_objects_time[0] = object_info_.obstacle_anngic_radar_front_right_list.msg_head.timestamp;
    if (time_flag_.anngic_radar_front_right_objects_time[0] != time_flag_.anngic_radar_front_right_objects_time[1]) {
      time_flag_.anngic_radar_front_right_objects_time[1] = time_flag_.anngic_radar_front_right_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT);
      SaveDataCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT);
    }

    shared_data->GetObstacleAnngicRadarRearLeftList(&object_info_.obstacle_anngic_radar_rear_left_list);
    time_flag_.anngic_radar_rear_left_objects_time[0] = object_info_.obstacle_anngic_radar_rear_left_list.msg_head.timestamp;
    if (time_flag_.anngic_radar_rear_left_objects_time[0] != time_flag_.anngic_radar_rear_left_objects_time[1]) {
      time_flag_.anngic_radar_rear_left_objects_time[1] = time_flag_.anngic_radar_rear_left_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT);
      SaveDataCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT);
    }

    shared_data->GetObstacleAnngicRadarRearRightList(&object_info_.obstacle_anngic_radar_rear_right_list);
    time_flag_.anngic_radar_rear_right_objects_time[0] = object_info_.obstacle_anngic_radar_rear_right_list.msg_head.timestamp;
    if (time_flag_.anngic_radar_rear_right_objects_time[0] != time_flag_.anngic_radar_rear_right_objects_time[1]) {
      time_flag_.anngic_radar_rear_right_objects_time[1] = time_flag_.anngic_radar_rear_right_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT);
      SaveDataCsvFile(framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT);
    }

    shared_data->GetObstacleVisualControlFrontList(&object_info_.obstacle_visual_control_front_list);
    time_flag_.visual_control_front_objects_time[0] = object_info_.obstacle_visual_control_front_list.msg_head.timestamp;
    if (time_flag_.visual_control_front_objects_time[0] != time_flag_.visual_control_front_objects_time[1]) {
      time_flag_.visual_control_front_objects_time[1] = time_flag_.visual_control_front_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT);
      SaveDataCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT);
    }
    
    shared_data->GetObstacleVisualControlFrontLeftList(&object_info_.obstacle_visual_control_front_left_list);
    time_flag_.visual_control_front_left_objects_time[0] = object_info_.obstacle_visual_control_front_left_list.msg_head.timestamp;
    if (time_flag_.visual_control_front_left_objects_time[0] != time_flag_.visual_control_front_left_objects_time[1]) {
      time_flag_.visual_control_front_left_objects_time[1] = time_flag_.visual_control_front_left_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT);
      SaveDataCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT);
    }

    shared_data->GetObstacleVisualControlFrontRightList(&object_info_.obstacle_visual_control_front_right_list);
    time_flag_.visual_control_front_right_objects_time[0] = object_info_.obstacle_visual_control_front_right_list.msg_head.timestamp;
    if (time_flag_.visual_control_front_right_objects_time[0] != time_flag_.visual_control_front_right_objects_time[1]) {
      time_flag_.visual_control_front_right_objects_time[1] = time_flag_.visual_control_front_right_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT);
      SaveDataCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT);
    }

    shared_data->GetObstacleVisualControlRearLeftList(&object_info_.obstacle_visual_control_rear_left_list);
    time_flag_.visual_control_rear_left_objects_time[0] = object_info_.obstacle_visual_control_rear_left_list.msg_head.timestamp;
    if (time_flag_.visual_control_rear_left_objects_time[0] != time_flag_.visual_control_rear_left_objects_time[1]) {
      time_flag_.visual_control_rear_left_objects_time[1] = time_flag_.visual_control_rear_left_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT);
      SaveDataCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT);
    }

    shared_data->GetObstacleVisualControlRearRightList(&object_info_.obstacle_visual_control_rear_right_list);
    time_flag_.visual_control_rear_right_objects_time[0] = object_info_.obstacle_visual_control_rear_right_list.msg_head.timestamp;
    if (time_flag_.visual_control_rear_right_objects_time[0] != time_flag_.visual_control_rear_right_objects_time[1]) {
      time_flag_.visual_control_rear_right_objects_time[1] = time_flag_.visual_control_rear_right_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT);
      SaveDataCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT);
    }

    shared_data->GetObstacleVisualControlRearList(&object_info_.obstacle_visual_control_rear_list);
    time_flag_.visual_control_rear_objects_time[0] = object_info_.obstacle_visual_control_rear_list.msg_head.timestamp;
    if (time_flag_.visual_control_rear_objects_time[0] != time_flag_.visual_control_rear_objects_time[1]) {
      time_flag_.visual_control_rear_objects_time[1] = time_flag_.visual_control_rear_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR);
      SaveDataCsvFile(framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR);
    }

    shared_data->GetObstaclesFusionList(&object_info_.obstacle_fusion_list);
    time_flag_.fusion_objects_time[0] = object_info_.obstacle_fusion_list.msg_head.timestamp;
    if (time_flag_.fusion_objects_time[0] != time_flag_.fusion_objects_time[1]) {
      time_flag_.fusion_objects_time[1] = time_flag_.fusion_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_SENSORS_FUSION_DATA);
      SaveDataCsvFile(framework::MSG_ID_RECV_SENSORS_FUSION_DATA);
    }

    shared_data->GetADHMIObstaclesFusionList(&object_info_.adhmi_obstacle_fusion_list);
    time_flag_.adhmi_fusion_objects_time[0] = object_info_.adhmi_obstacle_fusion_list.msg_head.timestamp;
    if (time_flag_.adhmi_fusion_objects_time[0] != time_flag_.adhmi_fusion_objects_time[1]) {
      time_flag_.adhmi_fusion_objects_time[1] = time_flag_.adhmi_fusion_objects_time[0];
      CreateCsvFile(framework::MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA);
      SaveDataCsvFile(framework::MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA);
    }
    
    usleep(1000 * 10);
  }

  LOG_INFO(3) << "Thread of save data to csv ... [Stopped]";
}

std::string TaskSaveDataToCsv::GetProgramPath() {
  // 获取程序路径
  char path[1024] = { 0 };
  if (readlink("/proc/self/exe", path, sizeof(path)-2) < 1) {
    std::cout << "readlink failed!" << std::endl;
    //return (-1);
  }
  std::string current_path(path);
  // Set work space
  int separator_count = 0;
  std::string::reverse_iterator it_current_path = current_path.rbegin();
  for (; it_current_path != current_path.rend(); ++it_current_path) {
    if (*it_current_path == '/') {
      if (it_current_path == current_path.rbegin()) {
        continue;
      }
      ++it_current_path;
      ++separator_count;
      if (separator_count > 1) {
        break;
      }
    }
  }
  std::string work_space(current_path.begin(), it_current_path.base());
  std::cout << "work_space = " << work_space << std::endl;
  return work_space;
}

void TaskSaveDataToCsv::path_is_exist(std::string path) {
  // 判断路径是否存在
  if(access(path.c_str(),0) == -1) {
    // 创建目录
    mkdir(path.c_str(),0777);
  }
}

// 将时间戳转换为时间
void TaskSaveDataToCsv::TimeStampToLocalTime() {
  time_t now = object_info_.adhmi_clock.clock.sec;
  tm* p = localtime(&now);

  sprintf(local_time, "%04d:%02d:%02d %02d:%02d:%02d.%09d", p->tm_year+1900, p->tm_mon+1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec, object_info_.adhmi_clock.clock.nsec);
  //printf("local_time:%s\n", local_time);
}

// 在csv_data目录下创建csv文件
void TaskSaveDataToCsv::CreateCsvFile(Int32_t data_flag) {
  // 判断csv存放路径是否存在以及创建目录
  path_is_exist(current_program_path);

  switch (data_flag) {
    case framework::MSG_ID_LANE_MARK_CAMERA_LIST:
      if (!out_files_.maxieye_lane_mark_out_file.is_open()){
        std::string maxieye_lane_mark_csv_file_name = current_program_path + "/maxieye_lane_mark.csv";
        //maxieye_lane_mark_csv_file_name = current_program_path + "/maxieye_lane_mark.csv";

        out_files_.maxieye_lane_mark_out_file.open(maxieye_lane_mark_csv_file_name);
        out_files_.maxieye_lane_mark_out_file << "timestamp, lane_mark_num, lane_index, id, lane_mark_type, quality, view_range_valid, mark_width, view_range_start, view_range_end, c0, c1, c2, c3" << std::endl;
      }
    break;

    case framework::MSG_ID_LANE_CURB_CAMERA_LIST:
      if (!out_files_.maxieye_lane_curb_out_file.is_open()){
        std::string maxieye_lane_curb_csv_file_name = current_program_path + "/maxieye_lane_curb.csv";
        //maxieye_lane_curb_csv_file_name = current_program_path + "/maxieye_lane_curb.csv";

        out_files_.maxieye_lane_curb_out_file.open(maxieye_lane_curb_csv_file_name);
        out_files_.maxieye_lane_curb_out_file << "timestamp, lane_mark_num, lane_index, id, lane_mark_type, quality, view_range_valid, mark_width, view_range_start, view_range_end, c0, c1, c2, c3" << std::endl;        
      }
    break;

    case framework::MSG_ID_RECV_ESR_DATA_FRONT:
      if (!out_files_.esr_front_objects_out_file.is_open()){
        std::string esr_front_objects_csv_file_name = current_program_path + "/esr_front_objects.csv";
        //esr_front_objects_csv_file_name = current_program_path + "/esr_front_objects.csv";

        out_files_.esr_front_objects_out_file.open(esr_front_objects_csv_file_name);
        out_files_.esr_front_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,type,track_status,merged_status,oncomming,bridge,range,angle,range_rate,range_acceleration,lateral_rate,length,width,x,y,v_x,v_y,accel_x,accel_y,yaw_rate" << std::endl;
      }
    break;

    case framework::MSG_ID_CHASSIS:
      if (!out_files_.chassis_info_out_file.is_open()){
        std::string chassis_info_csv_file_name = current_program_path + "/chassis_info.csv";
        //chassis_info_csv_file_name = current_program_path + "/chassis_info.csv";

        out_files_.chassis_info_out_file.open(chassis_info_csv_file_name);
        out_files_.chassis_info_out_file << "timestamp, driving_mode,e_stop,eps_status,throttle_sys_status,ebs_status,steering_wheel_angle_valid,steering_wheel_angle,steering_wheel_speed_valid, \
                steering_wheel_speed,steering_wheel_torque_valid,steering_wheel_torque,v_valid,v,a_valid,a,yaw_rate_valid,yaw_rate,ax_valid,ax,ay_valid,ay, \
                wheel_speed_fl_valid,wheel_speed_fl,wheel_speed_fr_valid,wheel_speed_fr,wheel_speed_rl_valid,wheel_speed_rl,wheel_speed_rr_valid,wheel_speed_rr, \
                wheel_speed_rl2_valid,wheel_speed_rl2,wheel_speed_rr2_valid,wheel_speed_rr2,epb_status,gear,gear_number,signal_turning_indicator,signal_turn_lamp, \
                signal_brake_lamp,brake_pedal_value,acc_pedal_value,engine_speed_valid,engine_speed,engine_torque_valid,engine_torque" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT:
      if (!out_files_.maxieye_objects_out_file.is_open()){
        std::string maxieye_obstacle_csv_file_name = current_program_path + "/maxieye_obstacle.csv";
        //maxieye_obstacle_csv_file_name = current_program_path + "/maxieye_obstacle.csv";

        out_files_.maxieye_objects_out_file.open(maxieye_obstacle_csv_file_name);
        out_files_.maxieye_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST:
      if (!out_files_.visual_control_lane_mark_out_file.is_open()){
        std::string visual_control_lane_mark_csv_file_name = current_program_path + "/visual_control_lane_mark.csv";
        //visual_control_lane_mark_csv_file_name = current_program_path + "/visual_control_lane_mark.csv";

        out_files_.visual_control_lane_mark_out_file.open(visual_control_lane_mark_csv_file_name);
        out_files_.visual_control_lane_mark_out_file << "timestamp, lane_mark_num, lane_index, id, lane_mark_type, quality, view_range_valid, mark_width, view_range_start, view_range_end, c0, c1, c2, c3" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT:
      if (!out_files_.visual_control_front_objects_out_file.is_open()){
        std::string visual_control_front_objects_csv_file_name = current_program_path + "/visual_control_front_objects.csv";
        //visual_control_front_objects_csv_file_name = current_program_path + "/visual_control_front_objects.csv";

        out_files_.visual_control_front_objects_out_file.open(visual_control_front_objects_csv_file_name);
        out_files_.visual_control_front_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT:
      if (!out_files_.visual_control_front_left_objects_out_file.is_open()){
        std::string visual_control_front_left_objects_csv_file_name = current_program_path + "/visual_control_front_left_objects.csv";
        //visual_control_front_left_objects_csv_file_name = current_program_path + "/visual_control_front_left_objects.csv";

        out_files_.visual_control_front_left_objects_out_file.open(visual_control_front_left_objects_csv_file_name);
        out_files_.visual_control_front_left_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT:
      if (!out_files_.visual_control_front_right_objects_out_file.is_open()){
        std::string visual_control_front_right_objects_csv_file_name = current_program_path + "/visual_control_front_right_objects.csv";
        //visual_control_front_right_objects_csv_file_name = current_program_path + "/visual_control_front_right_objects.csv";

        out_files_.visual_control_front_right_objects_out_file.open(visual_control_front_right_objects_csv_file_name);
        out_files_.visual_control_front_right_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT:
      if (!out_files_.visual_control_rear_left_objects_out_file.is_open()){
        std::string visual_control_rear_left_objects_csv_file_name = current_program_path + "/visual_control_rear_left_objects.csv";
        //visual_control_rear_left_objects_csv_file_name = current_program_path + "/visual_control_rear_left_objects.csv";

        out_files_.visual_control_rear_left_objects_out_file.open(visual_control_rear_left_objects_csv_file_name);
        out_files_.visual_control_rear_left_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT:
      if (!out_files_.visual_control_rear_right_objects_out_file.is_open()){
        std::string visual_control_rear_right_objects_csv_file_name = current_program_path + "/visual_control_rear_right_objects.csv";
        //visual_control_rear_right_objects_csv_file_name = current_program_path + "/visual_control_rear_right_objects.csv";

        out_files_.visual_control_rear_right_objects_out_file.open(visual_control_rear_right_objects_csv_file_name);
        out_files_.visual_control_rear_right_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR:
      if (!out_files_.visual_control_rear_objects_out_file.is_open()){
        std::string visual_control_rear_objects_csv_file_name = current_program_path + "/visual_control_rear_objects.csv";
        //visual_control_rear_objects_csv_file_name = current_program_path + "/visual_control_rear_objects.csv";

        out_files_.visual_control_rear_objects_out_file.open(visual_control_rear_objects_csv_file_name);
        out_files_.visual_control_rear_objects_out_file << "timestamp, obstacle_num, obstacle_index, id, type, status, cut_in, blinker, brake_lights, age, lane, length, width, height, x, y, heading, v_x, v_y, accel_x, accel_y, yaw_rate, scale_change" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT:
      if (!out_files_.anngic_radar_front_left_objects_out_file.is_open()){
        std::string anngic_radar_front_left_objects_csv_file_name = current_program_path + "/anngic_radar_front_left_objects.csv";
        //anngic_radar_front_left_objects_csv_file_name = current_program_path + "/anngic_radar_front_left_objects.csv";

        out_files_.anngic_radar_front_left_objects_out_file.open(anngic_radar_front_left_objects_csv_file_name);
        out_files_.anngic_radar_front_left_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,type,track_status,merged_status,oncomming,bridge,range,angle,range_rate,range_acceleration,lateral_rate,length,width,x,y,v_x,v_y,accel_x,accel_y,yaw_rate" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT:
      if (!out_files_.anngic_radar_front_right_objects_out_file.is_open()){
        std::string anngic_radar_front_right_objects_csv_file_name = current_program_path + "/anngic_radar_front_right_objects.csv";
        //anngic_radar_front_right_objects_csv_file_name = current_program_path + "/anngic_radar_front_right_objects.csv";

        out_files_.anngic_radar_front_right_objects_out_file.open(anngic_radar_front_right_objects_csv_file_name);
        out_files_.anngic_radar_front_right_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,type,track_status,merged_status,oncomming,bridge,range,angle,range_rate,range_acceleration,lateral_rate,length,width,x,y,v_x,v_y,accel_x,accel_y,yaw_rate" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT:
      if (!out_files_.anngic_radar_rear_left_objects_out_file.is_open()){
        std::string anngic_radar_rear_left_objects_csv_file_name = current_program_path + "/anngic_radar_rear_left_objects.csv";
        //anngic_radar_rear_left_objects_csv_file_name = current_program_path + "/anngic_radar_rear_left_objects.csv";

        out_files_.anngic_radar_rear_left_objects_out_file.open(anngic_radar_rear_left_objects_csv_file_name);
        out_files_.anngic_radar_rear_left_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,type,track_status,merged_status,oncomming,bridge,range,angle,range_rate,range_acceleration,lateral_rate,length,width,x,y,v_x,v_y,accel_x,accel_y,yaw_rate" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT:
      if (!out_files_.anngic_radar_rear_right_objects_out_file.is_open()){
        std::string anngic_radar_rear_right_objects_csv_file_name = current_program_path + "/anngic_radar_rear_right_objects.csv";
        //anngic_radar_rear_right_objects_csv_file_name = current_program_path + "/anngic_radar_rear_right_objects.csv";

        out_files_.anngic_radar_rear_right_objects_out_file.open(anngic_radar_rear_right_objects_csv_file_name);
        out_files_.anngic_radar_rear_right_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,type,track_status,merged_status,oncomming,bridge,range,angle,range_rate,range_acceleration,lateral_rate,length,width,x,y,v_x,v_y,accel_x,accel_y,yaw_rate" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_SENSORS_FUSION_DATA:
      if (!out_files_.fusion_objects_out_file.is_open()){
        std::string fusion_objects_csv_file_name = current_program_path + "/fusion_obstacle.csv";
        //fusion_objects_csv_file_name = current_program_path + "/fusion_obstacle.csv";

        out_files_.fusion_objects_out_file.open(fusion_objects_csv_file_name);
        out_files_.fusion_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,x,y,obb.x,obb.y,obb.heading,obb.half_width,obb.half_length,height,height_to_ground,type,dynamic,confidence,perception_type,v_x,v_y,v,a_x,a_y,a" << std::endl;
      }
    break;

    case framework::MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA:
      if (!out_files_.adhmi_fusion_objects_out_file.is_open()){
        std::string adhmi_fusion_objects_csv_file_name = current_program_path + "/adhmi_fusion_obstacle.csv";
        //adhmi_fusion_objects_csv_file_name = current_program_path + "/adhmi_fusion_obstacle.csv";

        out_files_.adhmi_fusion_objects_out_file.open(adhmi_fusion_objects_csv_file_name);
        out_files_.adhmi_fusion_objects_out_file << "timestamp, obstacle_num, obstacle_index, id,x,y,obb.x,obb.y,obb.heading,obb.half_width,obb.half_length,height,height_to_ground,type,dynamic,confidence,perception_type,v_x,v_y,v,a_x,a_y,a" << std::endl;
      }
    break;
  }
}

// 保存数据到csv文件中
void TaskSaveDataToCsv::SaveDataCsvFile(Int32_t data_flag) {
  switch (data_flag) {
    case framework::MSG_ID_LANE_MARK_CAMERA_LIST:
    {
      Int32_t lane_mark_num = object_info_.lane_mark_camera_list.lane_mark_num;
      for (Int32_t i = 0; i < lane_mark_num; ++i) {
        const ad_msg::LaneMarkCamera& lane_mark =
            object_info_.lane_mark_camera_list.lane_marks[i];

        out_files_.maxieye_lane_mark_out_file << local_time << ","
                        << lane_mark_num << ","
                        << i << ","
                        << lane_mark.id << ","
                        << lane_mark.lane_mark_type << ","
                        << lane_mark.quality << ","
                        << lane_mark.view_range_valid << ","
                        << lane_mark.mark_width << ","
                        << lane_mark.view_range_start << ","
                        << lane_mark.view_range_end << ","
                        << lane_mark.c0 << ","
                        << lane_mark.c1 << ","
                        << lane_mark.c2 << ","
                        << lane_mark.c3 << ","
                        << std::endl;
      }
    } 
    break;

    case framework::MSG_ID_LANE_CURB_CAMERA_LIST:
    {
      Int32_t lane_curb_num = object_info_.lane_curb_camera_list.lane_mark_num;
      for (Int32_t i = 0; i < object_info_.lane_curb_camera_list.lane_mark_num; ++i) {
        const ad_msg::LaneMarkCamera& lane_mark =
            object_info_.lane_curb_camera_list.lane_marks[i];

        out_files_.maxieye_lane_curb_out_file << local_time << ","
                        << lane_curb_num << ","
                        << i << ","
                        << lane_mark.id << ","
                        << lane_mark.lane_mark_type << ","
                        << lane_mark.quality << ","
                        << lane_mark.view_range_valid << ","
                        << lane_mark.mark_width << ","
                        << lane_mark.view_range_start << ","
                        << lane_mark.view_range_end << ","
                        << lane_mark.c0 << ","
                        << lane_mark.c1 << ","
                        << lane_mark.c2 << ","
                        << lane_mark.c3 << ","
                        << std::endl;

      }
    }
    break;

    case framework::MSG_ID_RECV_ESR_DATA_FRONT:
    {
      Int32_t obstacle_num = object_info_.obstacle_esr_front_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleRadar& obstacle = object_info_.obstacle_esr_front_list.obstacles[i];

        out_files_.esr_front_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.track_status << ","
                        << obstacle.merged_status << ","
                        << obstacle.oncomming << ","
                        << obstacle.bridge << ","
                        << obstacle.range << ","
                        << obstacle.angle << ","
                        << obstacle.range_rate << ","
                        << obstacle.range_acceleration << ","
                        << obstacle.lateral_rate << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_CHASSIS:
    {
      out_files_.chassis_info_out_file << local_time << ","
                  << (Int32_t)object_info_.chassis_info.driving_mode << ","
                  << (Int32_t)object_info_.chassis_info.e_stop << ","
                  << (Int32_t)object_info_.chassis_info.eps_status << ","
                  << (Int32_t)object_info_.chassis_info.throttle_sys_status << ","
                  << (Int32_t)object_info_.chassis_info.ebs_status << ","
                  << (Int32_t)object_info_.chassis_info.steering_wheel_angle_valid << ","
                  << object_info_.chassis_info.steering_wheel_angle << ","
                  << (Int32_t)object_info_.chassis_info.steering_wheel_speed_valid << ","
                  << object_info_.chassis_info.steering_wheel_speed << ","
                  << (Int32_t)object_info_.chassis_info.steering_wheel_torque_valid << ","
                  << object_info_.chassis_info.steering_wheel_torque << ","
                  << (Int32_t)object_info_.chassis_info.v_valid << ","
                  << object_info_.chassis_info.v << ","
                  << (Int32_t)object_info_.chassis_info.a_valid << ","
                  << object_info_.chassis_info.a << ","
                  << (Int32_t)object_info_.chassis_info.yaw_rate_valid << ","
                  << object_info_.chassis_info.yaw_rate << ","
                  << (Int32_t)object_info_.chassis_info.ax_valid << ","
                  << object_info_.chassis_info.ax << ","
                  << (Int32_t)object_info_.chassis_info.ay_valid << ","
                  << object_info_.chassis_info.ay << ","
                  << (Int32_t)object_info_.chassis_info.wheel_speed_fl_valid << ","
                  << object_info_.chassis_info.wheel_speed_fl << ","
                  << (Int32_t)object_info_.chassis_info.wheel_speed_fr_valid << ","
                  << object_info_.chassis_info.wheel_speed_fr << ","
                  << (Int32_t)object_info_.chassis_info.wheel_speed_rl_valid << ","
                  << object_info_.chassis_info.wheel_speed_rl << ","
                  << (Int32_t)object_info_.chassis_info.wheel_speed_rr_valid << ","
                  << object_info_.chassis_info.wheel_speed_rr << ","
                  << (Int32_t)object_info_.chassis_info.wheel_speed_rl2_valid << ","
                  << object_info_.chassis_info.wheel_speed_rl2 << ","
                  << (Int32_t)object_info_.chassis_info.wheel_speed_rr2_valid << ","
                  << object_info_.chassis_info.wheel_speed_rr2 << ","
                  << (Int32_t)object_info_.chassis_info.epb_status << ","
                  << (Int32_t)object_info_.chassis_info.gear << ","
                  << (Int32_t)object_info_.chassis_info.gear_number << ","
                  << (Int32_t)object_info_.chassis_info.signal_turning_indicator << ","
                  << (Int32_t)object_info_.chassis_info.signal_turn_lamp << ","
                  << (Int32_t)object_info_.chassis_info.signal_brake_lamp << ","
                  << (Int32_t)object_info_.chassis_info.brake_pedal_value << ","
                  << (Int32_t)object_info_.chassis_info.acc_pedal_value << ","
                  << (Int32_t)object_info_.chassis_info.engine_speed_valid << ","
                  << object_info_.chassis_info.engine_speed << ","
                  << (Int32_t)object_info_.chassis_info.engine_torque_valid << ","
                  << object_info_.chassis_info.engine_torque
                  << std::endl;
    }
    break;

    case framework::MSG_ID_RECV_MAXIRYR_CAMERA_OBJECTS_FRONT:
    {
      Int32_t obstacle_num = object_info_.obstacle_maxieye_camera_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_maxieye_camera_list.obstacles[i];

        out_files_.maxieye_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_VISUAL_CONTROL_LANE_MARK_CAMERA_LIST:
    {
      Int32_t lane_mark_num = object_info_.visual_control_lane_mark_camera_list.lane_mark_num;
      for (Int32_t i = 0; i < lane_mark_num; ++i) {
        const ad_msg::LaneMarkCamera& lane_mark =
            object_info_.visual_control_lane_mark_camera_list.lane_marks[i];
       
        out_files_.visual_control_lane_mark_out_file << local_time << ","
                        << lane_mark_num << ","
                        << i << ","
                        << lane_mark.id << ","
                        << lane_mark.lane_mark_type << ","
                        << lane_mark.quality << ","
                        << lane_mark.view_range_valid << ","
                        << lane_mark.mark_width << ","
                        << lane_mark.view_range_start << ","
                        << lane_mark.view_range_end << ","
                        << lane_mark.c0 << ","
                        << lane_mark.c1 << ","
                        << lane_mark.c2 << ","
                        << lane_mark.c3 << ","
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT:
    {
      Int32_t obstacle_num = object_info_.obstacle_visual_control_front_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_visual_control_front_list.obstacles[i];

        out_files_.visual_control_front_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_LEFT:
    {
      Int32_t obstacle_num = object_info_.obstacle_visual_control_front_left_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_visual_control_front_left_list.obstacles[i];

        out_files_.visual_control_front_left_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_FRONT_RIGHT:
    {
      Int32_t obstacle_num = object_info_.obstacle_visual_control_front_right_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_visual_control_front_right_list.obstacles[i];

        out_files_.visual_control_front_right_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_LEFT:
    {
      Int32_t obstacle_num = object_info_.obstacle_visual_control_rear_left_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_visual_control_rear_left_list.obstacles[i];

        out_files_.visual_control_rear_left_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR_RIGHT:
    {
      Int32_t obstacle_num = object_info_.obstacle_visual_control_rear_right_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_visual_control_rear_right_list.obstacles[i];

        out_files_.visual_control_rear_right_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_VISUAL_CONTROL_OBJECTS_REAR:
    {
      Int32_t obstacle_num = object_info_.obstacle_visual_control_rear_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleCamera& obstacle = object_info_.obstacle_visual_control_rear_list.obstacles[i];

        out_files_.visual_control_rear_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.status << ","
                        << obstacle.cut_in << ","
                        << obstacle.blinker << ","
                        << obstacle.brake_lights << ","
                        << obstacle.age << ","
                        << obstacle.lane << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.height << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.heading << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << obstacle.scale_change
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_LEFT:
    {
      Int32_t obstacle_num = object_info_.obstacle_anngic_radar_front_left_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleRadar& obstacle = object_info_.obstacle_anngic_radar_front_left_list.obstacles[i];

        out_files_.anngic_radar_front_left_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.track_status << ","
                        << obstacle.merged_status << ","
                        << obstacle.oncomming << ","
                        << obstacle.bridge << ","
                        << obstacle.range << ","
                        << obstacle.angle << ","
                        << obstacle.range_rate << ","
                        << obstacle.range_acceleration << ","
                        << obstacle.lateral_rate << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_FRONT_RIGHT:
    {
      Int32_t obstacle_num = object_info_.obstacle_anngic_radar_front_right_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleRadar& obstacle = object_info_.obstacle_anngic_radar_front_right_list.obstacles[i];

        out_files_.anngic_radar_front_right_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.track_status << ","
                        << obstacle.merged_status << ","
                        << obstacle.oncomming << ","
                        << obstacle.bridge << ","
                        << obstacle.range << ","
                        << obstacle.angle << ","
                        << obstacle.range_rate << ","
                        << obstacle.range_acceleration << ","
                        << obstacle.lateral_rate << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_LEFT:
    {
      Int32_t obstacle_num = object_info_.obstacle_anngic_radar_rear_left_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleRadar& obstacle = object_info_.obstacle_anngic_radar_rear_left_list.obstacles[i];

        out_files_.anngic_radar_rear_left_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.track_status << ","
                        << obstacle.merged_status << ","
                        << obstacle.oncomming << ","
                        << obstacle.bridge << ","
                        << obstacle.range << ","
                        << obstacle.angle << ","
                        << obstacle.range_rate << ","
                        << obstacle.range_acceleration << ","
                        << obstacle.lateral_rate << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_ANNGIC_RADAR_OBJECTS_REAR_RIGHT:
    {
      Int32_t obstacle_num = object_info_.obstacle_anngic_radar_rear_right_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::ObstacleRadar& obstacle = object_info_.obstacle_anngic_radar_rear_right_list.obstacles[i];

        out_files_.anngic_radar_rear_right_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << (int32_t)obstacle.type << ","
                        << obstacle.track_status << ","
                        << obstacle.merged_status << ","
                        << obstacle.oncomming << ","
                        << obstacle.bridge << ","
                        << obstacle.range << ","
                        << obstacle.angle << ","
                        << obstacle.range_rate << ","
                        << obstacle.range_acceleration << ","
                        << obstacle.lateral_rate << ","
                        << obstacle.length << ","
                        << obstacle.width << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.accel_x << ","
                        << obstacle.accel_y << ","
                        << obstacle.yaw_rate << ","
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_SENSORS_FUSION_DATA:
    {
      Int32_t obstacle_num = object_info_.obstacle_fusion_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::Obstacle& obstacle = object_info_.obstacle_fusion_list.obstacles[i];

        out_files_.fusion_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.obb.x << ","
                        << obstacle.obb.y << ","
                        << obstacle.obb.heading << ","
                        << obstacle.obb.half_width << ","
                        << obstacle.obb.half_length << ","
                        << obstacle.height << ","
                        << obstacle.height_to_ground << ","
                        << (int32_t)obstacle.type << ","
                        << (int32_t)obstacle.dynamic << ","
                        << (int32_t)obstacle.confidence << ","
                        << (int32_t)obstacle.perception_type << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.v << ","
                        << obstacle.a_x << ","
                        << obstacle.a_y << ","
                        << obstacle.a
                        << std::endl;
      }
    }
    break;

    case framework::MSG_ID_RECV_ADHMI_SENSORS_FUSION_DATA:
    {
      Int32_t obstacle_num = object_info_.adhmi_obstacle_fusion_list.obstacle_num;
      for (Int32_t i = 0; i < obstacle_num; ++i) {
        const ad_msg::Obstacle& obstacle = object_info_.adhmi_obstacle_fusion_list.obstacles[i];

        out_files_.adhmi_fusion_objects_out_file << local_time << ","
                        << obstacle_num << ","
                        << i << ","
                        << obstacle.id << ","
                        << obstacle.x << ","
                        << obstacle.y << ","
                        << obstacle.obb.x << ","
                        << obstacle.obb.y << ","
                        << obstacle.obb.heading << ","
                        << obstacle.obb.half_width << ","
                        << obstacle.obb.half_length << ","
                        << obstacle.height << ","
                        << obstacle.height_to_ground << ","
                        << (int32_t)obstacle.type << ","
                        << (int32_t)obstacle.dynamic << ","
                        << (int32_t)obstacle.confidence << ","
                        << (int32_t)obstacle.perception_type << ","
                        << obstacle.v_x << ","
                        << obstacle.v_y << ","
                        << obstacle.v << ","
                        << obstacle.a_x << ","
                        << obstacle.a_y << ","
                        << obstacle.a
                        << std::endl;
      }
    }
    break;

    default:

    break;
  }
}

} // namespace sensor
} // namespace phoenix

