/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** 主窗口
 ******************************************************************************
 *
 *  主窗口
 *
 *  @file       main_window.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_HMI_MAIN_WINDOW_H_
#define PHOENIX_HMI_MAIN_WINDOW_H_

#include <string>
#include <vector>
#include <list>
#include <QMainWindow>
#include <QCloseEvent>
#include <QLabel>
#include "widget_line_chart.h"
#include "widget_map.h"
#include "widget_event_reporting.h"
#include "motion_planning.h"


namespace Ui {
class MainWindow;
}


namespace phoenix {
namespace hmi {


class LineChartWidgetLatErr;
class LineChartWidgetYawErr;
class LineChartWidgetYawRate;
class LineChartWidgetAccInfo;
class LineChartWidgetVehVelocity;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(int argc, char** argv,
                      const std::string& work_space, QWidget *parent = 0);
  ~MainWindow();

public slots:
  void OpenWidgetSettingsPrefrence();
  void OpenWidgetVehicleParameters();
  void OpenWidgetDebugInfo();
  void OpenWidgetAbout();

protected:
  void closeEvent(QCloseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void moveEvent(QMoveEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void timerEvent(QTimerEvent *event) override;

  void InitHmiStatus();
  void UpdateHmiStatus();
  void UpdateModuleInfo();
  void UpdateHmiSettings();
  void UpdateLineChartWidgets();
  void HandleEventReporting();

private slots:
  void on_pushButtonStartAdas_clicked();

  void on_pushButtonEnableLKA_clicked();
  void on_pushButtonEnableACC_clicked();
  void on_pushButtonEnableAEB_clicked();
  void on_pushButtonEnableALC_clicked();
  void on_pushButtonEnableISL_clicked();
  void on_pushButtonEnableNGP_clicked();
  void on_pushButtonEnableLevel_clicked();
  void on_pushButtonEnablePCC_clicked();
  void on_pushButtonSetTarTimeGap_clicked();
  
  void on_pushButtonSetTarSpeed_clicked();
  void on_pushButtonSetTarAcceleration_clicked();
  void on_pushButtonSetTarLevel_clicked();
  
  void on_pushButtonChangingLaneLeft_clicked();
  void on_pushButtonChangingLaneAbort_clicked();
  void on_pushButtonChangingLaneRight_clicked();

  void on_checkBoxShowLineChart_clicked();

private:
  class SettingControl {
  public:
    SettingControl() {
      setting_req_ = false;
      send_count_ = 0;
      int_value_ = 0;
      float_value_ = 0.0F;
    }

    void SetValue(int value) {
      int_value_ = value;

      setting_req_ = true;
      send_count_ = 0;
    }
    void SetValue(float value) {
      float_value_ = value;

      setting_req_ = true;
      send_count_ = 0;
    }

    bool ReqSend() {
      if (setting_req_) {
        send_count_++;
        if (send_count_ > 3) {
          send_count_ = 0;
          setting_req_ = false;
        }
      }

      return (setting_req_);
    }

    int GetIntValue() const {
      return (int_value_);
    }

    float GetFloatValue() const {
      return (float_value_);
    }

  private:
    bool setting_req_;
    int send_count_;
    int int_value_;
    float float_value_;
  };

private:
  std::string work_space_;
  Ui::MainWindow *ui;
  int refresh_timer_ = 0;

  struct {
    ad_msg::PlanningSettings curr_planning_settings;
    ad_msg::Chassis chassis_status;
    pos_filter::PosFilterInfo pos_filter_info;
    driv_map::DrivingMapInfo driving_map_info;
    planning::TrajectoryPlanningInfo trajectory_planning_info;
    planning::VelocityPlanningResult velocity_planning_result;
  } module_info_;

  WidgetMap* widget_map_;
  WidgetEventReporting* widget_event_reporting_;

  LineChartWidgetLatErr* widget_lat_err_;
  LineChartWidgetYawErr* widget_yaw_err_;
  LineChartWidgetYawRate* widget_yaw_rate_;
  LineChartWidgetAccInfo* widget_acc_info_;
  LineChartWidgetVehVelocity* widget_velocity_info_;

  QLabel* label_status_;

  /// Settings
  Int32_t send_settings_count_;
  // Start ADAS
  SettingControl control_start_adas_;
  // Enable LKA
  SettingControl control_enable_lka_;
  // Enable ACC
  SettingControl control_enable_acc_;
  // Enable AEB
  SettingControl control_enable_aeb_;
  // Enable ALC
  SettingControl control_enable_alc_;
  // Enable ISL
  SettingControl control_enable_isl_;
  // Enable NGP
  SettingControl control_enable_ngp_;
  // Enable Fallback
  SettingControl control_enable_fallback_;
  // Enable PCC
  SettingControl control_enable_pcc_;
  // Set Target Speed
  SettingControl control_set_tar_speed_;
  // Set Target Acceleration
  SettingControl control_set_tar_acceleration_;
  // Set Target TimeGap
  SettingControl control_set_tar_time_gap_;
  // Set Target fallback level 0 - A, 1 - B, 2 - C, 3 - D
  SettingControl control_set_tar_fallback_level_;
  // 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  SettingControl control_changing_lane_;

  // Event reporting
  ad_msg::EventReportingList event_reporting_list_;

  enum { MAX_STR_BUFF_SIZE = 1024*4 };
  char str_buff_[MAX_STR_BUFF_SIZE];
};


static int s_line_chart_x_offset = 150;

class LineChartWidgetLatErr : public WidgetLineChart  {
  Q_OBJECT

public:
  explicit LineChartWidgetLatErr(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    lat_err_ = 0.0F;
    lat_err_smooth_ = 0.0F;
    lat_err_spd_ = 0.0F;
    lat_err_spd_smooth_ = 0.0F;

    PARAM param[4];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 0;
    param[1].color.B = 0;

    param[2].color.R = 0;
    param[2].color.G = 255;
    param[2].color.B = 255;

    param[3].color.R = 0;
    param[3].color.G = 0;
    param[3].color.B = 255;

    Config(-100.0, 100.0, 4, param);
    SetMinValue(-5.0, 5.0);
  }

  ~LineChartWidgetLatErr() {
  }

  void Refresh(double err, double smooth_err,
               double err_spd, double err_spd_smooth) {
    lat_err_ = err*100;
    lat_err_smooth_ = smooth_err*100;
    lat_err_spd_ = err_spd*100;
    lat_err_spd_smooth_ = err_spd_smooth*100;

    double data[4] = {lat_err_, lat_err_smooth_, lat_err_spd_, lat_err_spd_smooth_};
    PushBack(data);
    UpdateWindow();
  }

private:
  void DrawSomething() {
    DrawDatas();
  }

  void DrawSomeText() {
    QFont font("AnyStyle");

    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "横向偏差");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Err: %0.0f", lat_err_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Smt: %0.0f", lat_err_smooth_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Spd: %0.0f", lat_err_spd_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Smt: %0.0f", lat_err_spd_smooth_);
    renderText(x, y, 0, string_buffer_, font);
  }

private:
  double lat_err_;
  double lat_err_smooth_;
  double lat_err_spd_;
  double lat_err_spd_smooth_;
};

class LineChartWidgetYawErr : public WidgetLineChart  {
  Q_OBJECT

public:
  explicit LineChartWidgetYawErr(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    yaw_err_ = 0.0F;
    yaw_err_smooth_ = 0.0F;
    yaw_err_spd_ = 0.0F;
    yaw_err_spd_smooth_ = 0.0F;

    PARAM param[4];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 0;
    param[1].color.B = 0;

    param[2].color.R = 0;
    param[2].color.G = 255;
    param[2].color.B = 255;

    param[3].color.R = 0;
    param[3].color.G = 0;
    param[3].color.B = 255;

    Config(-1.0, 1.0, 4, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetYawErr() {
  }

  void Refresh(double err, double smooth_err,
               double err_spd, double err_spd_smooth) {
    yaw_err_ = common::com_rad2deg(err);
    yaw_err_smooth_ = common::com_rad2deg(smooth_err);
    yaw_err_spd_ = common::com_rad2deg(err_spd);
    yaw_err_spd_smooth_ = common::com_rad2deg(err_spd_smooth);

    double data[4] = {yaw_err_, yaw_err_smooth_, yaw_err_spd_, yaw_err_spd_smooth_};
    PushBack(data);
    UpdateWindow();
  }

private:
  void DrawSomething() {
    DrawDatas();
  }

  void DrawSomeText() {
    QFont font("AnyStyle");

    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "角度偏差");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;

    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Err: %0.2f", yaw_err_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Smt: %0.2f", yaw_err_smooth_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Spd: %0.2f", yaw_err_spd_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Smt: %0.2f", yaw_err_spd_smooth_);
    renderText(x, y, 0, string_buffer_, font);
  }

private:
  double yaw_err_;
  double yaw_err_smooth_;
  double yaw_err_spd_;
  double yaw_err_spd_smooth_;
};

class LineChartWidgetYawRate : public WidgetLineChart {
  Q_OBJECT

public:
  explicit LineChartWidgetYawRate(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    current_yaw_rate_steering_ = 0;
    current_yaw_rate_imu_ = 0;
    current_yaw_rate_chassis_ = 0;
    current_yaw_rate_ = 0;

    PARAM param[4];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 255;
    param[1].color.B = 0;

    param[2].color.R = 0;
    param[2].color.G = 255;
    param[2].color.B = 255;

    param[3].color.R = 0;
    param[3].color.G = 0;
    param[3].color.B = 255;

    Config(-20.0, 20.0, 4, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetYawRate() {
  }

  void Refresh(double yaw_rate_str, double yaw_rate_imu,
               double yaw_rate_chsssis, double yaw_rate) {
    current_yaw_rate_steering_ = phoenix::common::com_rad2deg(yaw_rate_str);
    current_yaw_rate_imu_ = phoenix::common::com_rad2deg(yaw_rate_imu);
    current_yaw_rate_chassis_ = phoenix::common::com_rad2deg(yaw_rate_chsssis);
    current_yaw_rate_ = phoenix::common::com_rad2deg(yaw_rate);

    double data[4] = {current_yaw_rate_steering_, current_yaw_rate_imu_,
                      current_yaw_rate_chassis_, current_yaw_rate_};
    PushBack(data);
    UpdateWindow();
  }

private:
  // ????
  void DrawSomething() {
    DrawDatas();
  }

  void DrawSomeText() {
    QFont font("AnyStyle");

    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "角速度");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Str: %0.2f", current_yaw_rate_steering_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Imu: %0.2f", current_yaw_rate_imu_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Chs: %0.2f", current_yaw_rate_chassis_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Cur: %0.2f", current_yaw_rate_);
    renderText(x, y, 0, string_buffer_, font);
  }

private:
  double current_yaw_rate_steering_;
  double current_yaw_rate_imu_;
  double current_yaw_rate_chassis_;
  double current_yaw_rate_;
};

class LineChartWidgetAccInfo : public WidgetLineChart {
  Q_OBJECT

public:
  explicit LineChartWidgetAccInfo(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    tar_type_ = 0;
    obj_valid_ = false;
    dist_to_obj_ = 0.0F;
    time_gap_ = 0.0F;
    ttc_ = 0.0F;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;
    Config(0, 100.0, 2, param);
    SetMinValue(0, 4.0);
  }

  ~LineChartWidgetAccInfo() {
  }

  void Refresh(
      int tar_type, bool obj_valid,
      double dist_to_obj, double t_gap, double ego_vel ,double relative_v) {
    tar_type_ = tar_type;
    obj_valid_ = obj_valid;
    dist_to_obj_ = dist_to_obj;
    time_gap_ = t_gap;

    if ( -1 * relative_v > 0) {
      ttc_ = dist_to_obj / relative_v;
    }

    if ( -1 *relative_v > 0) {
        if (ego_vel < 55 / 3.6) {
            Bakering_distance_ = (relative_v * relative_v) / (3.0 * 2.0);
        } else if (55 / 3.6 <= ego_vel < 65.6 / 3.6) {
            Bakering_distance_ = (relative_v * relative_v) / (3.5 * 2.0);
        } else if (65 / 3.6 <= ego_vel < 75 / 3.6) {
            Bakering_distance_ = (relative_v * relative_v) / (3.5 * 2.0);
        } else {
            Bakering_distance_ = (relative_v * relative_v) / (4.0 * 2.0);
        }
    } else {
        Bakering_distance_ = 0.0;
    }

    if (Bakering_distance_ > dist_to_obj - 10.0) {
        Emergency_stop_ = true;
    } else {
        Emergency_stop_ = false;
    }


    double data[2] = {2.0, time_gap_};
    PushBack(data);

    UpdateWindow();
  }

private:
  void DrawSomething() {
    DrawDatas();
  }

  void DrawSomeText() {
    QFont font("AnyStyle");

    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "目标时距:3s");
    renderText(5, height() - 15, 0, string_buffer_, font);

    qglColor(QColor(0, 0, 255));
    switch(tar_type_) {
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "巡航(用户设定)");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_CURVATURE):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "巡航(曲率限制)");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "巡航(障碍物限制)");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING):
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "跟随");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "AEB预减速");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "紧急制动");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    case (planning::VELOCITY_PLANNING_TARGET_TYPE_SCENE_STORY):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "场景任务");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    /*
    下一个版本 引入急停状态的时候打开
    case (planning::)
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "紧急停车");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    */  
    default:
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "未知类型");
      renderText(5, height() - 50, 0, string_buffer_, font);
      break;
    }

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    if (obj_valid_) {
      qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    } else {
      qglColor(QColor(210, 210, 210));
    }
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "time_gap: %0.2fs", time_gap_);
    renderText(x, y, 0, string_buffer_, font);

    if (obj_valid_) {
      qglColor(QColor(0, 0, 0));
    } else {
      qglColor(QColor(210, 210, 210));
    }

    y -= 15;
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "dist: %0.2fm", dist_to_obj_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "ttc: %0.2fs", ttc_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Baker_dis: %0.2fm", Bakering_distance_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "E_stop_flag: %d", Emergency_stop_);
    renderText(x, y, 0, string_buffer_, font);

  }

private:
  int tar_type_;
  bool obj_valid_;
  double dist_to_obj_;
  double time_gap_;
  double ttc_;
  bool Emergency_stop_;
  double Bakering_distance_;
};

class LineChartWidgetVehVelocity : public WidgetLineChart {
  Q_OBJECT

public:
  explicit LineChartWidgetVehVelocity(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    target_velocity_ = 0;
    current_velocity_ = 0;
    obj_velocity_ = 0;
    target_a_ = 0;
    current_a_ = 0;

    PARAM param[3];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;

    param[2].color.R = 255;
    param[2].color.G = 255;
    param[2].color.B = 0;

    Config(0.0, 200.0, 3, param);
    SetMinValue(0.0, 30.0);
  }

  ~LineChartWidgetVehVelocity() {
  }

  void Refresh(double tar_v, double cur_v, double obj_v, double tar_a, double cur_a) {
    target_velocity_ = tar_v * 3.6;
    current_velocity_ = cur_v * 3.6;
    obj_velocity_ = obj_v * 3.6;
    target_a_ =  tar_a;
    current_a_ = cur_a;

    double data[3] = {target_velocity_, current_velocity_, obj_velocity_};
    PushBack(data);

    UpdateWindow();
  }

private:
  void DrawSomething() {
    DrawDatas();
  }

  void DrawSomeText() {
    QFont font("AnyStyle");

    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "速度");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "tar_v: %0.1f", target_velocity_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "cur_v: %0.1f", current_velocity_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "obj_v: %0.2f", obj_velocity_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "tar_a: %0.2f", target_a_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "cur_a: %0.2f", current_a_);
    renderText(x, y, 0, string_buffer_, font);
  }

private:
  double target_velocity_;
  double current_velocity_;
  double obj_velocity_;
  double target_a_;
  double current_a_;
};


} // hmi
} // phoenix


#endif  // PHOENIX_HMI_MAIN_WINDOW_H_
