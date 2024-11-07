//
#include "pc/task_control.h"

#include <algorithm>
#include <vector>
#include "boost/date_time.hpp"

#include "utils/com_clock_c.h"
#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "math/math_utils_c.h"
#include "pc/util.h"


namespace phoenix {
namespace framework {


TaskControl::TaskControl(Task* manager, TaskChassisControl* chassis_control)
  : Task(TASK_ID_CONTROL, "Control", manager),
    chassis_control_(chassis_control),
    log_file_lat_ctl_("lat_ctl"),
    log_file_veh_ctl_("veh_ctl") {
  thread_running_flag_lateral_control_ = false;
  thread_running_flag_vehicle_control_ = false;

  open_log_file_lat_ctl_ = false;
  open_log_file_veh_ctl_ = false;

  Phoenix_Work_LateralCtl_Initialize(&work_lat_ctl_instance_);
  Phoenix_Work_VehicleCtl_Initialize(&work_veh_ctl_instance_);
}

TaskControl::~TaskControl() {
  Stop();
}

bool TaskControl::Start() {
  if (!thread_running_flag_lateral_control_) {
    Phoenix_Work_LateralCtl_Initialize(&work_lat_ctl_instance_);
  }
  if (!thread_running_flag_vehicle_control_) {
    Phoenix_Work_VehicleCtl_Initialize(&work_veh_ctl_instance_);
  }

  // Start lateral control thread
  if (!thread_running_flag_lateral_control_) {
    LOG_INFO(3) << "Start Lateral Control Thread ...";
    thread_running_flag_lateral_control_ = true;
    thread_lateral_control_ =
        boost::thread(boost::bind(&TaskControl::ThreadLateralControl, this));
    LOG_INFO(3) << "Start Lateral Control Thread ... [OK]";
  }

  // Start vehicle control thread
  if (!thread_running_flag_vehicle_control_) {
    LOG_INFO(3) << "Start Vehicle Control Thread ...";
    thread_running_flag_vehicle_control_ = true;
    thread_vehicle_control_ =
        boost::thread(boost::bind(&TaskControl::ThreadVehicleControl, this));
    LOG_INFO(3) << "Start Vehicle Control Thread ... [OK]";
  }

  return (true);
}

bool TaskControl::Stop() {
  // Stop lateral control thread
  if (thread_running_flag_lateral_control_) {
    LOG_INFO(3) << "Stop lateral control thread ...";
    thread_running_flag_lateral_control_ = false;
    bool ret = thread_lateral_control_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread \"Lateral Control\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Lateral Control Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Lateral Control Thread... [NG]";
    }
  }

  // Stop vehicle control thread
  if (thread_running_flag_vehicle_control_) {
    LOG_INFO(3) << "Stop Vehicle Control Thread ...";
    thread_running_flag_vehicle_control_ = false;
    bool ret = thread_vehicle_control_.timed_join(
          boost::posix_time::seconds(2));
    if (false == ret) {
      LOG_ERR << "Failed to wait thread \"Vehicle Control\" exit.";
    }
    if (ret) {
      LOG_INFO(3) << "Stop Vehicle Control Thread... [OK]";
    } else {
      LOG_INFO(3) << "Stop Vehicle Control Thread... [NG]";
    }
  }

  return (true);
}

bool TaskControl::HandleMessage(const Message& msg, Task* sender) {
  // Warning: Must be reentrant function

  bool ret = true;

  return (ret);
}

void TaskControl::ThreadLateralControl() {
  LOG_INFO(3) << "Lateral Control Thread ... [Started]";

  Dormancy dormancy(50);
  while (thread_running_flag_lateral_control_) {
    Phoenix_Work_LateralCtl_DoWork(&work_lat_ctl_instance_);

    Phoenix_Work_VehicleCtl_TurnSteeringWheelDirectly(
          &work_veh_ctl_instance_,
          work_lat_ctl_instance_.tar_steering_wheel_angle,
          work_lat_ctl_instance_.tar_steering_wheel_speed);


#if 1
    if (!open_log_file_lat_ctl_) {
      open_log_file_lat_ctl_ = true;
      log_file_lat_ctl_.Open();
      log_file_lat_ctl_.Enable(true);

      std::snprintf(
            str_buff_lat_ctl_, sizeof(str_buff_lat_ctl_)-1,
            "timestamp,"
            "yaw_rate_imu,yaw_rate_chassis,yaw_rate_steering,"
            "yaw_rate,yaw_rate_chg_rate,tar_yaw_rate,"
            "lat_err_0,lat_err_1,lat_err_chg_rate_0,lat_err_chg_rate_1,"
            "yaw_err_0,yaw_err_1,yaw_err_chg_rate_0,yaw_err_chg_rate_1"
            "veh_spd,veh_gross_weight,"
            "goal_dist,goal_dist_near,goal_dist_far,"
            "feed_value_near,feed_value_far,feed_value,feed_value_smooth,"
            "lat_err,lat_err_spd,yaw_err,yaw_err_spd,lat_err_lv_idx,lat_err_spd_lv_idx,"
            "lat_err_p[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "lat_err_i[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "lat_err_d[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "lat_err_feed_value,"
            "yaw_rate_err,yaw_rate_spd,yaw_rate_err_lv_idx,yaw_rate_spd_lv_idx,"
            "yaw_rate_err_p[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "yaw_rate_err_i[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "yaw_rate_err_d[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "yaw_rate_err_feed_value,"
            "trj_curvature,"
            "veh_dynamic_p[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "veh_dynamic_i[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "veh_dynamic_d[v_ratio,w_ratio,k_ratio,k_value,feed],"
            "veh_dynamic_feed_value,"
            "steering_gain,"
            "tar_steering_angle,tar_steering_angle_smooth,"
            "target_steering_wheel_angle,target_steering_wheel_angle_speed,"
            "cur_steering_angle,cur_a,gear,gear_num,"
            "\n");
      log_file_lat_ctl_.Write(str_buff_lat_ctl_);
    }

    const Chassis_t& chassis_info = work_lat_ctl_instance_.chassis;
    const LateralControlInfo_t& lat_ctl_info = work_lat_ctl_instance_.lat_ctl_info;
    const LateralControlPidInfo_t& pid_info = lat_ctl_info.lat_ctl_pid_info;

    std::snprintf(
          str_buff_lat_ctl_, sizeof(str_buff_lat_ctl_)-1,
          "%ld,"
          "%0.3f,%0.3f,%0.3f,"
          "%0.3f,%0.3f,%0.3f,"
          "%0.3f,%0.3f,%0.3f,%0.3f,"
          "%0.3f,%0.3f,%0.3f,%0.3f,"
          "%0.1f,%0.1f,"
          "%0.1f,%0.1f,%0.1f,"
          "%0.3f,%0.3f,%0.3f,%0.3f,"
          "%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.2f,"
          "%0.2f,%0.2f,%d,%d,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.2f,"
          "%0.5f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.4f,%0.4f,%0.4f,%0.2f,%0.2f,"
          "%0.2f,"
          "%0.3f,"
          "%0.2f,%0.2f,"
          "%0.2f,%0.2f,"
          "%0.2f,%0.2f,%d,%d,"
          "\n",
          Phoenix_Common_GetSysClockNowMs(),
          common::com_rad2deg(lat_ctl_info.yaw_rate_imu), common::com_rad2deg(lat_ctl_info.yaw_rate_chassis), common::com_rad2deg(lat_ctl_info.yaw_rate_steering),
          common::com_rad2deg(lat_ctl_info.yaw_rate), common::com_rad2deg(lat_ctl_info.yaw_rate_chg_rate), common::com_rad2deg(lat_ctl_info.target_yaw_rate),
          lat_ctl_info.lat_err[0], lat_ctl_info.lat_err[1], lat_ctl_info.lat_err_chg_rate[0], lat_ctl_info.lat_err_chg_rate[1],
          lat_ctl_info.yaw_err[0], lat_ctl_info.yaw_err[1], lat_ctl_info.yaw_err_chg_rate[0], lat_ctl_info.yaw_err_chg_rate[1],
          lat_ctl_info.veh_spd*3.6F, lat_ctl_info.veh_gross_weight,
          pid_info.trj_feed.goal_dist, pid_info.trj_feed.goal_dist_near, pid_info.trj_feed.goal_dist_far,
          common::com_rad2deg(pid_info.trj_feed.feed_value_near), common::com_rad2deg(pid_info.trj_feed.feed_value_far), common::com_rad2deg(pid_info.trj_feed.feed_value), common::com_rad2deg(pid_info.trj_feed.feed_value_smooth),
          pid_info.lat_err_feed.lat_err, pid_info.lat_err_feed.lat_err_spd, pid_info.yaw_err_feed.yaw_err, pid_info.yaw_err_feed.yaw_err_spd, pid_info.lat_err_feed.lat_err_lv_idx, pid_info.lat_err_feed.lat_err_spd_lv_idx,
          pid_info.lat_err_feed.p_feed.v_ratio, pid_info.lat_err_feed.p_feed.w_ratio, pid_info.lat_err_feed.p_feed.k_ratio, common::com_rad2deg(pid_info.lat_err_feed.p_feed.k_value), common::com_rad2deg(pid_info.lat_err_feed.p_feed.feed),
          pid_info.lat_err_feed.i_feed.v_ratio, pid_info.lat_err_feed.i_feed.w_ratio, pid_info.lat_err_feed.i_feed.k_ratio, common::com_rad2deg(pid_info.lat_err_feed.i_feed.k_value), common::com_rad2deg(pid_info.lat_err_feed.i_feed.feed),
          pid_info.lat_err_feed.d_feed.v_ratio, pid_info.lat_err_feed.d_feed.w_ratio, pid_info.lat_err_feed.d_feed.k_ratio, common::com_rad2deg(pid_info.lat_err_feed.d_feed.k_value), common::com_rad2deg(pid_info.lat_err_feed.d_feed.feed),
          common::com_rad2deg(pid_info.lat_err_feed.feed_value),
          common::com_rad2deg(pid_info.yaw_rate_err_feed.yaw_rate_err), common::com_rad2deg(pid_info.yaw_rate_err_feed.yaw_rate_spd), pid_info.yaw_rate_err_feed.yaw_rate_err_lv_idx, pid_info.yaw_rate_err_feed.yaw_rate_spd_lv_idx,
          pid_info.yaw_rate_err_feed.p_feed.v_ratio, pid_info.yaw_rate_err_feed.p_feed.w_ratio, pid_info.yaw_rate_err_feed.p_feed.k_ratio, common::com_rad2deg(pid_info.yaw_rate_err_feed.p_feed.k_value), common::com_rad2deg(pid_info.yaw_rate_err_feed.p_feed.feed),
          pid_info.yaw_rate_err_feed.i_feed.v_ratio, pid_info.yaw_rate_err_feed.i_feed.w_ratio, pid_info.yaw_rate_err_feed.i_feed.k_ratio, common::com_rad2deg(pid_info.yaw_rate_err_feed.i_feed.k_value), common::com_rad2deg(pid_info.yaw_rate_err_feed.i_feed.feed),
          pid_info.yaw_rate_err_feed.d_feed.v_ratio, pid_info.yaw_rate_err_feed.d_feed.w_ratio, pid_info.yaw_rate_err_feed.d_feed.k_ratio, common::com_rad2deg(pid_info.yaw_rate_err_feed.d_feed.k_value), common::com_rad2deg(pid_info.yaw_rate_err_feed.d_feed.feed),
          common::com_rad2deg(pid_info.yaw_rate_err_feed.feed_value),
          pid_info.veh_dynamic_feed.trj_curvature,
          pid_info.veh_dynamic_feed.p_feed.v_ratio, pid_info.veh_dynamic_feed.p_feed.w_ratio, pid_info.veh_dynamic_feed.p_feed.k_ratio, common::com_rad2deg(pid_info.veh_dynamic_feed.p_feed.k_value), common::com_rad2deg(pid_info.veh_dynamic_feed.p_feed.feed),
          pid_info.veh_dynamic_feed.i_feed.v_ratio, pid_info.veh_dynamic_feed.i_feed.w_ratio, pid_info.veh_dynamic_feed.i_feed.k_ratio, common::com_rad2deg(pid_info.veh_dynamic_feed.i_feed.k_value), common::com_rad2deg(pid_info.veh_dynamic_feed.i_feed.feed),
          pid_info.veh_dynamic_feed.d_feed.v_ratio, pid_info.veh_dynamic_feed.d_feed.w_ratio, pid_info.veh_dynamic_feed.d_feed.k_ratio, common::com_rad2deg(pid_info.veh_dynamic_feed.d_feed.k_value), common::com_rad2deg(pid_info.veh_dynamic_feed.d_feed.feed),
          common::com_rad2deg(pid_info.veh_dynamic_feed.feed_value),
          pid_info.steering_gain,
          common::com_rad2deg(pid_info.tar_steering_angle), common::com_rad2deg(pid_info.tar_steering_angle_smooth),
          common::com_rad2deg(lat_ctl_info.target_steering_wheel_angle), common::com_rad2deg(lat_ctl_info.target_steering_wheel_angle_speed),
          common::com_rad2deg(chassis_info.steering_wheel_angle), chassis_info.a, chassis_info.gear, chassis_info.gear_number
          );
    log_file_lat_ctl_.Write(str_buff_lat_ctl_);
#endif

    dormancy.Sleep();
  }

  LOG_INFO(3) << "Lateral Control Thread ... [Stopped]";
}

void TaskControl::ThreadVehicleControl() {
  LOG_INFO(3) << "Vehicle Control Thread ... [Started]";

  Dormancy dormancy(10);
  while (thread_running_flag_vehicle_control_) {
    Phoenix_Work_VehicleCtl_DoWork(&work_veh_ctl_instance_);

    // 开启/停止自动驾驶
    if (2 == work_veh_ctl_instance_.chassis_ctl_cmd.start_robotic_ctl) {
      chassis_control_->StartRobotCtl();
    } else if (1 == work_veh_ctl_instance_.chassis_ctl_cmd.start_robotic_ctl) {
      chassis_control_->StopRobotCtl();
    } else {
      // nothing to do
    }

    dormancy.Sleep();
  }

  LOG_INFO(3) << "Vehicle Control Thread ... [Stopped]";
}

void TaskControl::StartRobotCtl() {
  Phoenix_Work_VehicleCtl_StartRobotCtl(&work_veh_ctl_instance_);
}

void TaskControl::StopRobotCtl() {
  Phoenix_Work_VehicleCtl_StopRobotCtl(&work_veh_ctl_instance_);
}

void TaskControl::EnableDirectCtl(bool enable) {
  Phoenix_Work_VehicleCtl_EnableDirectCtl(&work_veh_ctl_instance_, enable);
}

void TaskControl::EnableRemoteCtl(bool enable) {
  Phoenix_Work_VehicleCtl_EnableRemoteCtl(&work_veh_ctl_instance_, enable);
}

void TaskControl::EnableEps(bool enable) {
  Phoenix_Work_VehicleCtl_EnableEps(&work_veh_ctl_instance_, enable);
}

void TaskControl::EnableThrottleSys(bool enable) {
  Phoenix_Work_VehicleCtl_EnableThrottleSys(&work_veh_ctl_instance_, enable);
}

void TaskControl::EnableEbs(bool enable) {
  Phoenix_Work_VehicleCtl_EnableEbs(&work_veh_ctl_instance_, enable);
}

void TaskControl::TurnSteeringWheel(Float32_t angle, Float32_t speed) {
  Phoenix_Work_VehicleCtl_TurnSteeringWheel(
        &work_veh_ctl_instance_, angle, speed);
}

void TaskControl::SpeedUp(Float32_t velocity, Float32_t acc) {
  Phoenix_Work_VehicleCtl_SpeedUp(
        &work_veh_ctl_instance_, velocity, acc);
}

void TaskControl::Accelerate(Float32_t value) {
  Phoenix_Work_VehicleCtl_Accelerate(&work_veh_ctl_instance_, value);
}

void TaskControl::Brake(Float32_t value) {
  Phoenix_Work_VehicleCtl_Brake(&work_veh_ctl_instance_, value);
}

void TaskControl::ChangeGear(Int8_t value) {
  Phoenix_Work_VehicleCtl_ChangeGear(&work_veh_ctl_instance_, value);
}

void TaskControl::SetTurnLamp(Int8_t value) {
  Phoenix_Work_VehicleCtl_SetTurnLamp(&work_veh_ctl_instance_, value);
}

void TaskControl::SetWiper(Int8_t value) {
  Phoenix_Work_VehicleCtl_SetWiper(&work_veh_ctl_instance_, value);
}

void TaskControl::ReqManualInterrupt(bool enable) {
  //
}


}  // namespace framework
}  // namespace phoenix
