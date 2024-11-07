/******************************************************************************
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
#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <string>
#include <vector>
#include "QMainWindow"
#include "QCloseEvent"
#include "QLabel"
#include "widget_map.h"
#include "widget_line_chart.h"
#include "widget_histogram.h"
#include "common/message.h"
#include "communication_c/shared_data_c.h"

namespace Ui {
class MainWindow;
}

namespace phoenix {
namespace framework {
class TaskManager;
}
}

namespace phoenix {
namespace hmi {


class LineChartWidgetLatErr;
class LineChartWidgetYawErr;
class LineChartWidgetYawRate;
class LineChartWidgetTrjFeedforward;
class LineChartWidgetLatErrFeedBack;
class LineChartWidgetYawErrFeedBack;
class LineChartWidgetYawRateFeedBack;
class LineChartWidgetVehSteeringWheelAngle;
class LineChartWidgetVehVelocity;
class LineChartWidgetVehAcceleration;
class LineChartWidgetLonCtlValue;
// Add By ZQ
class LineChartWidgetPrevLatDist;
class LineChartWidgetPreHeading;
class LineChartWidgetPreCurvature;
class LineChartWidgetSteerAngle;

class HistogramWidgetYawRateFrequencyDomain;


class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(int argc, char** argv,
      const std::string& work_space, QWidget *parent = 0);
  ~MainWindow();

 private:
  void closeEvent(QCloseEvent *event) override;
  void resizeEvent(QResizeEvent * event) override;

 private slots:
  void on_pushButtonStartRobot_clicked();
  void on_checkBoxEnableEps_clicked();
  void on_checkBoxEnableThrottleSys_clicked();
  void on_checkBoxEnableEbs_clicked();

  void on_radioButtonEnableAutoMode_clicked();
  void on_radioButtonEnableDebugMode_clicked();
  void on_checkBoxEnableRemoteControl_clicked();

  void on_pushButtonSelectGearP_clicked();
  void on_pushButtonSelectGearN_clicked();
  void on_pushButtonSelectGearR_clicked();
  void on_pushButtonSelectGearD_clicked();
  void on_pushButtonTurnSteeringWheel_clicked();
  void on_pushButtonAccelerate_clicked();
  void on_pushButtonControlVelocity_clicked();
  void on_pushButtonBrake_clicked();
  void on_pushButtonReleaseLongitudinalCtl_clicked();
  void on_pushButtonInterrupt_clicked();

private:
  void timerEvent(QTimerEvent *event) override;

  void UpdateModuleInfo();
  void UpdateButtonStatus();
  void UpdateLineChartWidgets();
  void UpdateVehicleStatus();

 private:
  std::string work_space_;
  Ui::MainWindow *ui;
  int refresh_timer_ = 0;

  struct {
    bool is_module_ready = false;
    bool is_robotic_mode = false;

    Chassis_t chassis_status;
    ChassisCtlCmd_t chassis_cmd;
    LateralControlInfo_t lateral_control_info;
    SteeringControlInfo_t steering_ctl_info;
  } module_info_;

  WidgetMap* widget_map_;

  framework::TaskManager* task_manager_;

  QLabel* label_status_;

  LineChartWidgetLatErr* widget_lat_err_;
  LineChartWidgetYawErr* widget_yaw_err_;
  LineChartWidgetYawRate* widget_yaw_rate_;
  LineChartWidgetTrjFeedforward* widget_trj_feedforward_;
  LineChartWidgetLatErrFeedBack* widget_lat_err_feed_back_;
  LineChartWidgetYawErrFeedBack* widget_yaw_err_feed_back_;
  LineChartWidgetYawRateFeedBack* widget_yaw_rate_feed_back_;
  LineChartWidgetVehSteeringWheelAngle* widget_veh_steering_wheel_angle_;
  LineChartWidgetVehVelocity* widget_veh_velocity_;
  LineChartWidgetVehAcceleration* widget_veh_acceleration_;
  LineChartWidgetLonCtlValue* widget_lon_ctl_value_;
  //Add BY ZQ 
  LineChartWidgetPrevLatDist* widget_lat_dist_;
  LineChartWidgetPreHeading*  widget_heading_;
  LineChartWidgetPreCurvature* widget_Curvature_; 
  LineChartWidgetSteerAngle* widget_SteerAngle_;
  HistogramWidgetYawRateFrequencyDomain* widget_yaw_rate_frequency_domain_;
  HistogramWidgetYawRateFrequencyDomain* widget_tar_yaw_rate_frequency_domain_;
};


static int s_line_chart_x_offset = 150;

class LineChartWidgetLatErr : public WidgetLineChart  {
  Q_OBJECT

 public:
  explicit LineChartWidgetLatErr(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    lat_err_ = 0.0F;
    lat_err_v_ = 0.0F;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;

    Config(-100.0, 100.0, 2, param);
    SetMinValue(-5.0, 5.0);
  }

  ~LineChartWidgetLatErr() {
  }

  void Refresh(double err, double err_v) {
    lat_err_ = err*100;
    lat_err_v_ = err_v*100;

    double data[2] = {lat_err_, lat_err_v_};
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
        "Spd: %0.0f", lat_err_v_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double lat_err_;
  double lat_err_v_;
};

class LineChartWidgetYawErr : public WidgetLineChart  {
  Q_OBJECT

 public:
  explicit LineChartWidgetYawErr(QWidget* parent = 0)
    : WidgetLineChart(parent) {
    yaw_err_ = 0.0F;
    yaw_err_v_ = 0.0F;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;

    Config(-1.0, 1.0, 2, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetYawErr() {
  }

  void Refresh(double err, double err_v) {
    yaw_err_ = common::com_rad2deg(err);
    yaw_err_v_ = common::com_rad2deg(err_v);

    double data[4] = {yaw_err_, yaw_err_v_};
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
        "Spd: %0.2f", yaw_err_v_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double yaw_err_;
  double yaw_err_v_;
};

//角速度
class LineChartWidgetYawRate : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetYawRate(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    target_yaw_rate_ = 0.0;
    current_yaw_rate_imu_ = 0.0;
    current_yaw_rate_chassis_ = 0.0;
    current_yaw_rate_steering_ = 0.0;
    current_yaw_rate_ = 0.0;
    target_yaw_rate_diff_v_ = 0.0;
    current_yaw_rate_diff_v_ = 0.0;

    yaw_rate_hunting_energe_ = 0.0;

    PARAM param[7];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;

    param[2].color.R = 255;
    param[2].color.G = 255;
    param[2].color.B = 0;

    param[3].color.R = 0;
    param[3].color.G = 255;
    param[3].color.B = 0;

    param[4].color.R = 0;
    param[4].color.G = 255;
    param[4].color.B = 255;

    param[5].color.R = 255;
    param[5].color.G = 255;
    param[5].color.B = 255;

    param[6].color.R = 50;
    param[6].color.G = 50;
    param[6].color.B = 50;

    Config(-20.0, 20.0, 7, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetYawRate() {
  }

  void Refresh(double target_yaw_rate, double curr_yaw_rate,
               double yaw_rate_imu, double yaw_rate_chassis, double yaw_rate_steering,
               double tar_yaw_rate_diff_v, double curr_yaw_rate_diff_v,
               double hunting_energy) {
    target_yaw_rate_ = common::com_rad2deg(target_yaw_rate);
    current_yaw_rate_ = common::com_rad2deg(curr_yaw_rate);

    current_yaw_rate_imu_ = common::com_rad2deg(yaw_rate_imu);
    current_yaw_rate_chassis_ = common::com_rad2deg(yaw_rate_chassis);
    current_yaw_rate_steering_ = common::com_rad2deg(yaw_rate_steering);

    target_yaw_rate_diff_v_ = common::com_rad2deg(tar_yaw_rate_diff_v);
    current_yaw_rate_diff_v_ = common::com_rad2deg(curr_yaw_rate_diff_v);

    yaw_rate_hunting_energe_ = common::com_rad2deg(hunting_energy);

#if 0
    double data[7] = {
      target_yaw_rate_, current_yaw_rate_,
      current_yaw_rate_imu_, current_yaw_rate_chassis_, current_yaw_rate_steering_,
      target_yaw_rate_diff_v_, current_yaw_rate_diff_v_};
#else
    double data[7] = {
      target_yaw_rate_, current_yaw_rate_,
      current_yaw_rate_imu_, current_yaw_rate_chassis_, current_yaw_rate_steering_,
      target_yaw_rate_diff_v_, yaw_rate_hunting_energe_*3.0};
#endif
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

    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "hunting_energy: %0.3f", yaw_rate_hunting_energe_);
    renderText(5, 8, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Tar: %0.2f", target_yaw_rate_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Cur: %0.2f", current_yaw_rate_);
    renderText(x, y, 0, string_buffer_, font);

#if 0
    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Imu: %0.2f", current_yaw_rate_imu_);
    renderText(x, y, 0, string_buffer_, font);
#endif

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Cha: %0.2f", current_yaw_rate_chassis_);
    renderText(x, y, 0, string_buffer_, font);

#if 0
    y -= 15;
    qglColor(QColor(param_[4].color.R, param_[4].color.G, param_[4].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Str: %0.2f", current_yaw_rate_steering_);
    renderText(x, y, 0, string_buffer_, font);
#endif

    y -= 15;
    qglColor(QColor(param_[5].color.R, param_[5].color.G, param_[5].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Tif: %0.2f", target_yaw_rate_diff_v_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[6].color.R, param_[6].color.G, param_[6].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Dif: %0.2f", current_yaw_rate_diff_v_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double target_yaw_rate_;
  double current_yaw_rate_imu_;
  double current_yaw_rate_chassis_;
  double current_yaw_rate_steering_;
  double current_yaw_rate_;
  double target_yaw_rate_diff_v_;
  double current_yaw_rate_diff_v_;
  double yaw_rate_hunting_energe_;
};
// BY ZQ lat_dist replace yaw_rate
class LineChartWidgetPrevLatDist : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetPrevLatDist(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    prev_lat_dist_lane_ = 0.0;
    prev_lat_dist_path_ = 0.0;
    prev_lat_dist_cal_ = 0.0;

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

    Config(-20.0, 20.0, 3, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetPrevLatDist() {
  }

  void Refresh(double prev_lat_dist_lane, double prev_lat_dist_path, double prev_lat_dist_cal) {
    prev_lat_dist_lane_ = prev_lat_dist_lane;
    prev_lat_dist_path_ = prev_lat_dist_path;
    prev_lat_dist_cal_ = prev_lat_dist_cal;
    double data[3] = {prev_lat_dist_lane_, prev_lat_dist_path_,prev_lat_dist_cal_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "横向距离");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "lane: %0.2f", prev_lat_dist_lane_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "path: %0.2f", prev_lat_dist_path_);
    renderText(x, y, 0, string_buffer_, font);
    
    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "cal: %0.2f", prev_lat_dist_cal_);
    renderText(x, y, 0, string_buffer_, font);

  }

 private:
  double prev_lat_dist_lane_;
  double prev_lat_dist_path_;
  double prev_lat_dist_cal_;
};

// 轨迹前馈
class LineChartWidgetTrjFeedforward : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetTrjFeedforward(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    trj_feedforward_ = 0;
    trj_feedforward_smooth_ = 0;
    leading_len_ = 0;
    leading_len_near_ = 0;
    leading_len_far_ = 0;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 0;
    param[1].color.B = 0;

    Config(-720.0, 720.0, 2, param);
    SetMinValue(-5.0, 5.0);
  }

  ~LineChartWidgetTrjFeedforward() {
  }

  void Refresh(double fed, double fed_smt,
               double led_len, double led_len_n, double led_len_f) {
    trj_feedforward_ = common::com_rad2deg(fed);
    trj_feedforward_smooth_ = common::com_rad2deg(fed_smt);
    leading_len_ = led_len;
    leading_len_near_ = led_len_n;
    leading_len_far_ = led_len_f;
    double data[2] = {trj_feedforward_, trj_feedforward_smooth_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "轨迹前馈");
    renderText(5, height() - 15, 0, string_buffer_, font);

    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "leading: %0.1f[%0.1f, %0.1f]",
                  leading_len_, leading_len_near_, leading_len_far_);
    renderText(5, 8, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Fed: %0.2f", trj_feedforward_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Smt: %0.2f", trj_feedforward_smooth_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double trj_feedforward_ = 0;
  double trj_feedforward_smooth_ = 0;
  double leading_len_ = 0;
  double leading_len_near_ = 0;
  double leading_len_far_ = 0;
};
// BY ZQ PreHeading replace TrjFeedforward
class LineChartWidgetPreHeading : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetPreHeading(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    prev_heading_lane_ = 0.0;
    prev_heading_path_ = 0.0;
    prev_heading_cal_ = 0.0;

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

    Config(-20.0, 20.0, 3, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetPreHeading() {
  }

  void Refresh(double prev_heading_lane, double prev_heading_path, double prev_heading_cal) {
    prev_heading_lane_ = common::com_rad2deg(prev_heading_lane);
    prev_heading_path_ = common::com_rad2deg(prev_heading_path);
    prev_heading_cal_ = common::com_rad2deg(prev_heading_cal);

    double data[3] = {prev_heading_lane_, prev_heading_path_,prev_heading_cal_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "航向角");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "lane: %0.2f", prev_heading_lane_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "path: %0.2f", prev_heading_path_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "cal: %0.2f", prev_heading_cal_);
    renderText(x, y, 0, string_buffer_, font);

  }

 private:
  double prev_heading_lane_;
  double prev_heading_path_;
  double prev_heading_cal_;
};

// 位置偏差补偿
class LineChartWidgetLatErrFeedBack : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetLatErrFeedBack(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    value_p_ = 0;
    value_i_ = 0;
    value_d_ = 0;
    feed_value_ = 0;
    lat_err_ = 0.0F;
    lat_err_spd_ = 0.0F;
    lat_err_lv_idx_ = 0;
    lat_err_spd_lv_idx_ = 0;

    PARAM param[4];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 255;
    param[1].color.B = 255;

    param[2].color.R = 0;
    param[2].color.G = 255;
    param[2].color.B = 255;

    param[3].color.R = 0;
    param[3].color.G = 0;
    param[3].color.B = 255;

    Config(-80.0, 80.0, 4, param);
    SetMinValue(-5.0, 5.0);
  }

  ~LineChartWidgetLatErrFeedBack() {
  }

  void Refresh(
      double value_p, double value_i,
      double value_d, double feed_value,
      double lat_err, double lat_err_spd,
      int lat_err_lv_idx, int lat_err_spd_lv_idx) {
    value_p_ = common::com_rad2deg(value_p);
    value_i_ = common::com_rad2deg(value_i);
    value_d_ = common::com_rad2deg(value_d);
    feed_value_ = common::com_rad2deg(feed_value);

    lat_err_ = lat_err;
    lat_err_spd_ = lat_err_spd;
    lat_err_lv_idx_ = lat_err_lv_idx;
    lat_err_spd_lv_idx_ = lat_err_spd_lv_idx;

    double data[4] = {value_p_,
                      value_i_,
                      value_d_,
                      feed_value_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "位置偏差补偿");
    renderText(5, height() - 15, 0, string_buffer_, font);

    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "err: %0.2f(%d) spd: %0.2f(%d)",
                  lat_err_, lat_err_lv_idx_,
                  lat_err_spd_, lat_err_spd_lv_idx_);
    renderText(5, 8, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "P: %0.1f", value_p_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "I: %0.1f", value_i_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "D: %0.1f", value_d_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "F: %0.1f", feed_value_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double value_p_ = 0;
  double value_i_ = 0;
  double value_d_ = 0;
  double feed_value_ = 0;
  double lat_err_ = 0.0F;
  double lat_err_spd_ = 0.0F;
  int lat_err_lv_idx_ = 0;
  int lat_err_spd_lv_idx_ = 0;
};
// BY ZQ PreCurvature replace LatErrFeedBack
class LineChartWidgetPreCurvature : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetPreCurvature(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    prev_curvature_lane_ = 0.0;
    prev_curvature_path_ = 0.0;
    prev_curvature_cal_ =0.0;

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

    Config(-1000.0, 1000.0, 3, param);
    SetMinValue(-0.5, 0.5);
  }

  ~LineChartWidgetPreCurvature() {
  }

  void Refresh(double prev_curvature_lane, double prev_curvature_path, double prev_curvature_cal) {
    prev_curvature_lane_ = prev_curvature_lane;
    prev_curvature_path_ = prev_curvature_path;
    prev_curvature_cal_ = prev_curvature_cal;

    double data[3] = {prev_curvature_lane_, prev_curvature_path_, prev_curvature_cal_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "曲率");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "lane: %0.2f", prev_curvature_lane_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "path: %0.2f", prev_curvature_path_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "cal: %0.2f", prev_curvature_cal_);
    renderText(x, y, 0, string_buffer_, font);

  }

 private:
  double prev_curvature_lane_;
  double prev_curvature_path_;
  double prev_curvature_cal_;
}; 

// 角速度补偿
class LineChartWidgetYawErrFeedBack : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetYawErrFeedBack(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    value_p_ = 0;
    value_i_ = 0;
    value_d_ = 0;
    feed_value_ = 0;
    yaw_err_ = 0.0F;
    yaw_err_spd_ = 0.0F;
    yaw_err_lv_idx_ = 0;
    yaw_err_spd_lv_idx_ = 0;

    PARAM param[4];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 255;
    param[1].color.B = 255;

    param[2].color.R = 0;
    param[2].color.G = 255;
    param[2].color.B = 255;

    param[3].color.R = 0;
    param[3].color.G = 0;
    param[3].color.B = 255;

    Config(-80.0, 80.0, 4, param);
    SetMinValue(-5.0, 5.0);
  }

  ~LineChartWidgetYawErrFeedBack() {
  }

  void Refresh(
      double value_p, double value_i,
      double value_d, double feed_value,
      double yaw_err, double yaw_err_spd,
      int yaw_err_lv_idx, int yaw_err_spd_lv_idx) {
    value_p_ = common::com_rad2deg(value_p);
    value_i_ = common::com_rad2deg(value_i);
    value_d_ = common::com_rad2deg(value_d);
    feed_value_ = common::com_rad2deg(feed_value);

    yaw_err_ = common::com_rad2deg(yaw_err);
    yaw_err_spd_ = common::com_rad2deg(yaw_err_spd);
    yaw_err_lv_idx_ = yaw_err_lv_idx;
    yaw_err_spd_lv_idx_ = yaw_err_spd_lv_idx;

    double data[4] = {value_p_,
                      value_i_,
                      value_d_,
                      feed_value_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "角度偏差补偿");
    renderText(5, height() - 15, 0, string_buffer_, font);

    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "err: %0.2f(%d) spd: %0.2f(%d)",
                  yaw_err_, yaw_err_lv_idx_,
                  yaw_err_spd_, yaw_err_spd_lv_idx_);
    renderText(5, 8, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "P: %0.1f", value_p_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "I: %0.1f", value_i_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "D: %0.1f", value_d_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "F: %0.1f", feed_value_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double value_p_ = 0;
  double value_i_ = 0;
  double value_d_ = 0;
  double feed_value_ = 0;
  double yaw_err_ = 0.0F;
  double yaw_err_spd_ = 0.0F;
  int yaw_err_lv_idx_ = 0;
  int yaw_err_spd_lv_idx_ = 0;
};


class LineChartWidgetYawRateFeedBack : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetYawRateFeedBack(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    value_p_ = 0;
    value_i_ = 0;
    value_d_ = 0;
    feed_value_ = 0;
    yaw_rate_err_ = 0.0;
    yaw_rate_spd_ = 0.0;
    yaw_rate_err_lv_idx_ = 0;
    yaw_rate_spd_lv_idx_ = 0;

    PARAM param[4];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 255;
    param[1].color.B = 255;

    param[2].color.R = 0;
    param[2].color.G = 255;
    param[2].color.B = 255;

    param[3].color.R = 0;
    param[3].color.G = 0;
    param[3].color.B = 255;

    Config(-80.0, 80.0, 4, param);
    SetMinValue(-2.0, 2.0);
  }

  ~LineChartWidgetYawRateFeedBack() {
  }

  void Refresh(
      double value_p, double value_i,
      double value_d, double feed_value,
      double yaw_rate_err, double yaw_rate_spd,
      int yaw_rate_err_lv_idx, int yaw_rate_spd_lv_idx) {
    value_p_ = common::com_rad2deg(value_p);
    value_i_ = common::com_rad2deg(value_i);
    value_d_ = common::com_rad2deg(value_d);
    feed_value_ = common::com_rad2deg(feed_value);

    yaw_rate_err_ = common::com_rad2deg(yaw_rate_err);
    yaw_rate_spd_ = common::com_rad2deg(yaw_rate_spd);
    yaw_rate_err_lv_idx_ = yaw_rate_err_lv_idx;
    yaw_rate_spd_lv_idx_ = yaw_rate_spd_lv_idx;

    double data[4] = {value_p_,
                      value_i_,
                      value_d_,
                      feed_value_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "角速度补偿");
    renderText(5, height() - 15, 0, string_buffer_, font);

    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "err: %0.2f(%d) spd: %0.2f(%d)",
                  yaw_rate_err_, yaw_rate_err_lv_idx_,
                  yaw_rate_spd_, yaw_rate_spd_lv_idx_);
    renderText(5, 8, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "P: %0.1f", value_p_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "I: %0.1f", value_i_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "D: %0.1f", value_d_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "F: %0.1f", feed_value_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double value_p_ = 0;
  double value_i_ = 0;
  double value_d_ = 0;
  double feed_value_ = 0;
  double yaw_rate_err_ = 0.0;
  double yaw_rate_spd_ = 0.0;
  int yaw_rate_err_lv_idx_ = 0;
  int yaw_rate_spd_lv_idx_ = 0;
};

class LineChartWidgetVehSteeringWheelAngle : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetVehSteeringWheelAngle(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    target_angle_ = 0;
    target_angle_smooth_ = 0;
    current_angle_ = 0;
    target_angle_speed_ = 0;
    current_angle_speed_ = 0;

    PARAM param[5];
    param[0].color.R = 255;
    param[0].color.G = 255;
    param[0].color.B = 0;

    param[1].color.R = 255;
    param[1].color.G = 0;
    param[1].color.B = 0;

    param[2].color.R = 0;
    param[2].color.G = 0;
    param[2].color.B = 255;

    param[3].color.R = 255;
    param[3].color.G = 255;
    param[3].color.B = 255;

    param[4].color.R = 0;
    param[4].color.G = 0;
    param[4].color.B = 0;
    Config(-720.0, 720.0, 5, param);
    SetMinValue(-5.0, 5.0);
  }

  ~LineChartWidgetVehSteeringWheelAngle() {
  }

  void Refresh(double tar, double tar_smooth, double curr, double tsd, double spd) {
    target_angle_ = phoenix::common::com_rad2deg(tar);
    target_angle_smooth_ = phoenix::common::com_rad2deg(tar_smooth);
    current_angle_ = phoenix::common::com_rad2deg(curr);
    target_angle_speed_ = phoenix::common::com_rad2deg(tsd);
    current_angle_speed_ = phoenix::common::com_rad2deg(spd);
    double data[5] = {target_angle_, target_angle_smooth_, current_angle_, target_angle_speed_, current_angle_speed_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "方向盘转角");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Tar: %0.1f", target_angle_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Smt: %0.1f", target_angle_smooth_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[2].color.R, param_[2].color.G, param_[2].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Cur: %0.1f", current_angle_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[3].color.R, param_[3].color.G, param_[3].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Tsd: %0.1f", target_angle_speed_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[4].color.R, param_[4].color.G, param_[4].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Spd: %0.1f", current_angle_speed_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double target_angle_;
  double target_angle_smooth_;
  double target_angle_speed_;
  double current_angle_;
  double current_angle_speed_;
};

class LineChartWidgetVehVelocity : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetVehVelocity(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    target_velocity_ = 0;
    current_velocity_ = 0;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;
    Config(0.0, 100.0, 2, param);
    SetMinValue(0.0, 20.0);
  }

  ~LineChartWidgetVehVelocity() {
  }

  void Refresh(double tar, double curr) {
    target_velocity_ = tar*3.6f;
    current_velocity_ = curr*3.6f;
    double data[2] = {target_velocity_, current_velocity_};
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
        "Tar: %0.1f", target_velocity_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Cur: %0.1f", current_velocity_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double target_velocity_;
  double current_velocity_;
};

class LineChartWidgetVehAcceleration : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetVehAcceleration(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    target_acceleration_ = 0;
    current_acceleration_ = 0;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;
    Config(-15, 15, 2, param);
    SetMinValue(-2.0, 2.0);
  }

  ~LineChartWidgetVehAcceleration() {
  }

  void Refresh(double tar, double curr) {
    target_acceleration_ = tar;
    current_acceleration_ = curr;
    double data[2] = {target_acceleration_, current_acceleration_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "加速度");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Tar: %0.2f", target_acceleration_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Cur: %0.2f", current_acceleration_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double target_acceleration_;
  double current_acceleration_;
};

class LineChartWidgetLonCtlValue : public WidgetLineChart {
  Q_OBJECT

 public:
  explicit LineChartWidgetLonCtlValue(QWidget* parent = 0)
      : WidgetLineChart(parent) {
    brake_value_ = 0;
    acc_value_ = 0;

    PARAM param[2];
    param[0].color.R = 255;
    param[0].color.G = 0;
    param[0].color.B = 0;

    param[1].color.R = 0;
    param[1].color.G = 0;
    param[1].color.B = 255;
    Config(0.0, 100.0, 2, param);
    SetMinValue(0.0, 30.0);
  }

  ~LineChartWidgetLonCtlValue() {
  }

  void Refresh(double brake, double acc) {
    brake_value_ = brake;
    acc_value_ = acc;
    double data[2] = {brake_value_, acc_value_};
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
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "油门/制动");
    renderText(5, height() - 15, 0, string_buffer_, font);

    double x = width() - s_line_chart_x_offset;
    double y = height() - 15;
    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Brk: %0.1f", brake_value_);
    renderText(x, y, 0, string_buffer_, font);

    y -= 15;
    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
        "Acc: %0.1f", acc_value_);
    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  double brake_value_;
  double acc_value_;
};


class HistogramWidgetYawRateFrequencyDomain : public WidgetHistogram {
  Q_OBJECT

 public:
  explicit HistogramWidgetYawRateFrequencyDomain(QWidget* parent = 0)
      : WidgetHistogram(parent) {
    data_number_ = 0;
    common::com_memset(param_idxs_, 0, sizeof(param_idxs_));
    common::com_memset(feq_domain_mag_, 0, sizeof(feq_domain_mag_));

    Param param[2];
    param[0].color.R = 0;
    param[0].color.G = 0;
    param[0].color.B = 255;

    param[1].color.R = 255;
    param[1].color.G = 0;
    param[1].color.B = 0;
    Config(-1.0, 1.0, LAT_CTL_FREQUENCY_SPECTRUM_SIZE, 2, param);
  }

  ~HistogramWidgetYawRateFrequencyDomain() {
  }

  void Refresh(int data_num, const signed char* hunting_idxs, const float* mag) {
    data_number_ = data_num;
    if (data_number_ > LAT_CTL_FREQUENCY_SPECTRUM_SIZE) {
      data_number_ = LAT_CTL_FREQUENCY_SPECTRUM_SIZE;
    }

    for (int i = 0; i < data_number_; ++i) {
      if (hunting_idxs[i]) {
        param_idxs_[i] = 1;
      } else {
        param_idxs_[i] = 0;
      }
      feq_domain_mag_[i] = common::com_rad2deg(mag[i]);
    }

    UpdateWindow();
  }

 private:
  void DrawSomething() {
    DrawDatas(data_number_, feq_domain_mag_, param_idxs_);
  }

  void DrawSomeText() {
    QFont font("AnyStyle");

    qglColor(QColor(0, 0, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "角速度频谱图");
    renderText(5, height() - 15, 0, string_buffer_, font);

//    double x = width() - s_line_chart_x_offset;
//    double y = height() - 15;
//    qglColor(QColor(param_[0].color.R, param_[0].color.G, param_[0].color.B));
//    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
//        "Brk: %0.1f", brake_value_);
//    renderText(x, y, 0, string_buffer_, font);

//    y -= 15;
//    qglColor(QColor(param_[1].color.R, param_[1].color.G, param_[1].color.B));
//    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
//        "Acc: %0.1f", acc_value_);
//    renderText(x, y, 0, string_buffer_, font);
  }

 private:
  // data number
  int data_number_;
  // param list
  int param_idxs_[LAT_CTL_FREQUENCY_SPECTRUM_SIZE];
  // Magnitude
  float feq_domain_mag_[LAT_CTL_FREQUENCY_SPECTRUM_SIZE];
};


} // namespace hmi
} // namespace phoenix



#endif  // MAIN_WINDOW_H_
