/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** 主窗口
 ******************************************************************************
 *
 *  主窗口
 *
 *  @file       main_window.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "main_window.h"

#include <string>
#include "QMessageBox"
#include "ui_main_window.h"

#include "vehicle_model_wrapper.h"

#define SHOW_LINE_CHART_WIDGET (1)


namespace phoenix {
namespace hmi {


MainWindow::MainWindow(int argc, char **argv,
       const std::string &work_space, QWidget *parent) :
    QMainWindow(parent),
    work_space_(work_space),
    ui(new Ui::MainWindow) {
  ui->setupUi(this);

  // Set main window icon
  setWindowIcon(QIcon(":/images/icon.png"));

  // Adjust main window position
  //QRect rect;
  //rect.setX(100);
  //rect.setY(200);
  //rect.setSize(this->size());
  //this->setGeometry(rect);

  // Create map window
  widget_map_ = new WidgetMap();

  // Event reporting widget
  widget_event_reporting_ = new WidgetEventReporting(this);

  // 折线图
  // 位置误差
  widget_lat_err_ = new LineChartWidgetLatErr();
  // 角度误差
  widget_yaw_err_ = new LineChartWidgetYawErr();
  // 角速度
  widget_yaw_rate_ = new LineChartWidgetYawRate();
  // Acc info
  widget_acc_info_ = new LineChartWidgetAccInfo();
  // Vehicle velocity info
  widget_velocity_info_ = new LineChartWidgetVehVelocity();

  // gridLayoutMaps
  if (SHOW_LINE_CHART_WIDGET) {
    ui->gridLayoutMaps->addWidget(widget_map_, 0, 0, 5, 1);
    ui->gridLayoutMaps->addWidget(widget_lat_err_, 0, 1);
    ui->gridLayoutMaps->addWidget(widget_yaw_err_, 1, 1);
    ui->gridLayoutMaps->addWidget(widget_yaw_rate_, 2, 1);
    ui->gridLayoutMaps->addWidget(widget_acc_info_, 3, 1);
    ui->gridLayoutMaps->addWidget(widget_velocity_info_, 4, 1);
  } else {
    ui->gridLayoutMaps->addWidget(widget_map_, 0, 0);
  }

  on_checkBoxShowLineChart_clicked();

  Char_t str_buff[256] = { 0 };
  label_status_ = new QLabel();
  ui->statusbar->addWidget(label_status_);

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DF D17 B1");
  label_status_->setText(str_buff);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DF D17 B2");
  label_status_->setText(str_buff);
#else
  com_snprintf(str_buff, sizeof(str_buff)-1, "Unknow Vehicle Platform");
  label_status_->setText(str_buff);
#endif

//  // Settings widgets
//  widget_settings_preference_ = new WidgetSettingsPreference(work_space_);
//  connect(ui->actionSettingsPreference,
//      SIGNAL(triggered()), this, SLOT(OpenWidgetSettingsPrefrence()));

//  // Vehicle parameters
//  widget_vehicle_parameters_ = new WidgetVehicleParameters(work_space_);
//  connect(ui->actionVehicle,
//      SIGNAL(triggered()), this, SLOT(OpenWidgetVehicleParameters()));

//  // Debug widgets
//  widget_debug_info_ = new WidgetDebugInfo();
//  connect(ui->actionDebug,
//      SIGNAL(triggered()), this, SLOT(OpenWidgetDebugInfo()));

//  // about widget
//  widget_about_ = new WidgetAbout(work_space_);
//  connect(ui->actionAbout, SIGNAL(triggered()), this, SLOT(OpenWidgetAbout()));

//  // log widget
//  widget_log_ = new WidgetLog(this);
//  //widget_log_->AddLogInfo("uoijjs");
//  widget_log_->show();

//  connect(widget_settings_preference_,
//          SIGNAL(reqUpdateWindowSettings(const WindowSettings&)), this,
//          SLOT(UpdateWindowSettings(const WindowSettings&)));

  InitHmiStatus();

  refresh_timer_ = this->startTimer(100);
  if (0 == refresh_timer_) {
    LOG_ERR << "MainWindow, failed to start refresh Timer";
  }
}

MainWindow::~MainWindow() {
  //delete widget_settings_preference_;
  //delete widget_vehicle_parameters_;
  //delete widget_debug_info_;
  //delete widget_about_;
  if (SHOW_LINE_CHART_WIDGET) {
    // nothing to do
  } else {
    delete widget_lat_err_;
    delete widget_yaw_err_;
    delete widget_yaw_rate_;
    delete widget_acc_info_;
    delete widget_velocity_info_;
  }
  delete ui;
  LOG_INFO(3) << "MainWindow Deconstructed !";
}

void MainWindow::OpenWidgetSettingsPrefrence() {
//  widget_settings_preference_->show();
//  widget_settings_preference_->raise();
//  widget_settings_preference_->activateWindow();
}

void MainWindow::OpenWidgetVehicleParameters() {
//  widget_vehicle_parameters_->show();
//  widget_vehicle_parameters_->raise();
//  widget_vehicle_parameters_->activateWindow();
}

void MainWindow::OpenWidgetDebugInfo()  {
//  widget_debug_info_->show();
//  widget_debug_info_->raise();
//  widget_debug_info_->activateWindow();
}

void MainWindow::OpenWidgetAbout() {
//  widget_about_->show();
//  widget_about_->raise();
//  widget_about_->activateWindow();
}

//void MainWindow::UpdateWindowSettings(const WindowSettings& settings) {
//  std::cout << "Update window settings" << std::endl;

//  widget_map_->UpdateWindowSettings(settings);
//}

void MainWindow::timerEvent(QTimerEvent *event) {
  if (event->timerId() == refresh_timer_) {
    UpdateModuleInfo();

    // 更新HMI的状态信息
    UpdateHmiStatus();

    UpdateHmiSettings();

    widget_map_->Update();

    UpdateLineChartWidgets();

    HandleEventReporting();
  }
}

void MainWindow::closeEvent(QCloseEvent *event) {
  LOG_INFO(3) << "Received close event!";
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  //std::cout << "press" << std::endl;

  QMainWindow::mousePressEvent(event);
}

void MainWindow::moveEvent(QMoveEvent * event) {
  //std::cout << "poe: x=" << this->pos().x()
  //          << ", y=" << this->pos().y()
  //          << std::endl;

  widget_event_reporting_->UpdatePos(
        this->pos() + widget_event_reporting_->GetPosOffsetToParent());

  QMainWindow::moveEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent * event) {

  widget_event_reporting_->UpdatePos(
        this->pos() + widget_event_reporting_->GetPosOffsetToParent());

  QMainWindow::resizeEvent(event);
}

void MainWindow::UpdateModuleInfo()  {
  framework::SharedData* shared_data = framework::SharedData::instance();
  // Read some datas
  shared_data->GetCurrentPlanningSettings(&module_info_.curr_planning_settings);
  shared_data->GetChassis(&module_info_.chassis_status);
  shared_data->GetPosFilterInfo(&module_info_.pos_filter_info);
  shared_data->GetDrivingMapInfo(&module_info_.driving_map_info);
  shared_data->GetTrajectoryPlanningInfo(&module_info_.trajectory_planning_info);
  shared_data->GetVelocityPlanningResult(&module_info_.velocity_planning_result);
}

void MainWindow::UpdateHmiSettings() {
  framework::SharedData* shared_data = framework::SharedData::instance();

  bool enable_send = false;
  ad_msg::PlanningSettings settings;

  // ADAS功能: 0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_start_adas_.ReqSend()) {
    enable_send = true;
    settings.start_adas = control_start_adas_.GetIntValue();
  } else {
    settings.start_adas = 0;
  }

  // LKA功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_lka_.ReqSend()) {
    enable_send = true;
    settings.enable_lka = control_enable_lka_.GetIntValue();
  } else {
    settings.enable_lka = 0;
  }
  // ACC功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_acc_.ReqSend()) {
    enable_send = true;
    settings.enable_acc = control_enable_acc_.GetIntValue();
  } else {
    settings.enable_acc = 0;
  }
  // AEB功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_aeb_.ReqSend()) {
    enable_send = true;
    settings.enable_aeb = control_enable_aeb_.GetIntValue();
  } else {
    settings.enable_aeb = 0;
  }

  // 变道功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_alc_.ReqSend()) {
    enable_send = true;
    settings.enable_alc = control_enable_alc_.GetIntValue();
  } else {
    settings.enable_alc = 0;
  }
  // 智能限速功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_isl_.ReqSend()) {
    enable_send = true;
    settings.enable_isl = control_enable_isl_.GetIntValue();
  } else {
    settings.enable_isl = 0;
  }
  // NGP功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_ngp_.ReqSend()) {
    enable_send = true;
    settings.enable_ngp = control_enable_ngp_.GetIntValue();
  } else {
    settings.enable_ngp = 0;
  }
  // 降级功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_fallback_.ReqSend()) {
    enable_send = true;
    settings.enable_fallback = control_enable_fallback_.GetIntValue();
  } else {
    settings.enable_fallback = 0;
  }

  // PCC节油功能  0 ~ 无效, 1 ~ 关闭, 2 ~ 开启
  if (control_enable_pcc_.ReqSend()) {
    enable_send = true;
    settings.enable_pcc = control_enable_pcc_.GetIntValue();
  } else {
    settings.enable_pcc = 0;
  }  

  // 目标车速[m/s]
  if (control_set_tar_speed_.ReqSend()) {
    enable_send = true;
    settings.target_velocity_valid = true;
    settings.target_velocity = control_set_tar_speed_.GetFloatValue();
  } else {
    settings.target_velocity_valid = false;
    settings.target_velocity = control_set_tar_speed_.GetFloatValue();
  }
  // 加速度[m/s^2]
  if (control_set_tar_acceleration_.ReqSend()) {
    enable_send = true;
    settings.target_acc_valid = true;
    settings.target_acc = control_set_tar_acceleration_.GetFloatValue();
  } else {
    settings.target_acc_valid = false;
    settings.target_acc = control_set_tar_acceleration_.GetFloatValue();
  }
  // 时距[s]
  if (control_set_tar_time_gap_.ReqSend()) {
    enable_send = true;
    settings.target_time_gap_valid = true;
    settings.target_time_gap = control_set_tar_time_gap_.GetFloatValue();
  } else {
    settings.target_time_gap_valid = false;
    settings.target_time_gap = control_set_tar_time_gap_.GetFloatValue();
  }

  // 变道请求, 0 - 无请求, 1 - 左变道, 2 - 右变道, 3 - 取消变道
  if (control_changing_lane_.ReqSend()) {
    enable_send = true;
    settings.changing_lane_req = control_changing_lane_.GetIntValue();
  } else {
    settings.changing_lane_req = 0;
  }
  // 设定降级等级, 0 ~ 无请求, 1 ~ A, 2 ~ B, 3 ~ C, 4 ~ D
  if (control_set_tar_fallback_level_.ReqSend()) {
    enable_send = true;
    settings.target_fallback_level_valid = true;
    settings.target_fallback_level = control_set_tar_fallback_level_.GetIntValue();
  } else {
    settings.target_fallback_level_valid = false;
    settings.target_fallback_level = control_set_tar_fallback_level_.GetIntValue();
  }

  if (enable_send) {
    send_settings_count_ = 0;
    shared_data->SetLocalPlanningSettings(settings);
  } else {
    send_settings_count_++;
    if (send_settings_count_ > 4) {
      send_settings_count_ = 4;
    }
    if (send_settings_count_ < 4) {
      shared_data->SetLocalPlanningSettings(settings);
    }
  }
}

void MainWindow::UpdateLineChartWidgets() {
  const planning::TrajectoryPlanningInfo::LatErr& lat_err_info =
      module_info_.trajectory_planning_info.lat_err_sample[0];
  //const planning::TrajectoryPlanningInfo::LatErr& pred_lat_err_info =
  //    module_info_.trajectory_planning_info.lat_err_sample[1];

  widget_lat_err_->Refresh(
        lat_err_info.lat_err,
        lat_err_info.lat_err_smooth,
        lat_err_info.lat_err_v,
        lat_err_info.lat_err_v_smooth);
  widget_yaw_err_->Refresh(
        lat_err_info.yaw_err,
        lat_err_info.yaw_err_smooth,
        lat_err_info.yaw_err_v,
        lat_err_info.yaw_err_v_smooth);
  widget_yaw_rate_->Refresh(
        module_info_.pos_filter_info.yaw_rate_info.yaw_rate_steering,
        module_info_.pos_filter_info.yaw_rate_info.yaw_rate_imu,
        module_info_.pos_filter_info.yaw_rate_info.yaw_rate_chassis,
        module_info_.pos_filter_info.yaw_rate_info.yaw_rate);

  widget_acc_info_->Refresh(
        module_info_.velocity_planning_result.tar_type,
        module_info_.velocity_planning_result.tar_obj.valid,
        module_info_.velocity_planning_result.tar_obj.dist_to_obj,
        module_info_.velocity_planning_result.tar_obj.time_gap,
        module_info_.chassis_status.v,
        module_info_.velocity_planning_result.tar_obj.relative_v);

  widget_velocity_info_->Refresh(
        module_info_.velocity_planning_result.tar_v,
        module_info_.chassis_status.v,
        module_info_.velocity_planning_result.tar_obj.obj_v,
        module_info_.velocity_planning_result.tar_a,
        module_info_.chassis_status.a);
}

void MainWindow::HandleEventReporting() {
  framework::SharedData* shared_data = framework::SharedData::instance();

  shared_data->GetEventReportingList(&event_reporting_list_);

  if (event_reporting_list_.event_reporting_num > 0) {
    widget_event_reporting_->ShowEventInfo(event_reporting_list_.event_reporting[0]);
    widget_event_reporting_->show();
  } else {
    widget_event_reporting_->hide();
  }

  // std::cout << "Event: " << curr_module_event.event.event_id << std::endl;
}


void MainWindow::InitHmiStatus() {
  send_settings_count_ = 0;

  // Start ADAS
  control_start_adas_.SetValue(0);
  ui->pushButtonStartAdas->setText("Start");
  ui->pushButtonStartAdas->setEnabled(false);

  // Changeing lane request
  control_changing_lane_.SetValue(0);
}

void MainWindow::UpdateHmiStatus() {
  QString background_normal = "background-color: rgbd(82, 235, 233, 200);";
  QString background_warn = "background-color: rgbd(255, 255, 0, 200);";
  QString background_err = "background-color: rgbd(255, 100, 50, 200);";
  QString background_timeout = "background-color: rgbd(255, 128, 128, 200);";
  QString background_invalid = "background-color: rgbd(180, 180, 180, 200);";

  QColor color_normal(82, 235, 233, 200);
  QColor color_warn(255, 255, 0, 200);
  QColor color_err(255, 100, 50, 200);
  QColor color_timeout(255, 128, 128, 200);
  QColor color_invalid(180, 180, 180, 200);

  // Driving Mode
  switch (module_info_.chassis_status.driving_mode) {
  case (ad_msg::VEH_DRIVING_MODE_MANUAL):
    ui->pushButtonStartAdas->setText("Start");
    ui->pushButtonStartAdas->setEnabled(true);
    break;
  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC):
    ui->pushButtonStartAdas->setText("Stop");
    ui->pushButtonStartAdas->setEnabled(true);
    break;
  default:
    ui->pushButtonStartAdas->setText("Start");
    ui->pushButtonStartAdas->setEnabled(false);
    break;
  }

  switch (module_info_.curr_planning_settings.enable_lka) {
  case (1):
    // 已关闭 LKA
    ui->pushButtonEnableLKA->setStyleSheet(background_invalid);
    ui->pushButtonEnableLKA->setText("LKA");
    break;
  case (2):
    // 已开启 LKA
    ui->pushButtonEnableLKA->setStyleSheet(background_normal);
    ui->pushButtonEnableLKA->setText(">LKA<");
    break;
  default:
    // 无效状态
    ui->pushButtonEnableLKA->setStyleSheet(background_invalid);
    ui->pushButtonEnableLKA->setText("LKA");
    break;
  }

  switch (module_info_.curr_planning_settings.enable_acc) {
  case (1):
    // 已关闭 ACC
    ui->pushButtonEnableACC->setStyleSheet(background_invalid);
    ui->pushButtonEnableACC->setText("ACC");
    break;
  case (2):
    // 已开启 ACC
    ui->pushButtonEnableACC->setStyleSheet(background_normal);
    ui->pushButtonEnableACC->setText(">ACC<");
    break;
  default:
    // 无效状态
    ui->pushButtonEnableACC->setStyleSheet(background_invalid);
    ui->pushButtonEnableACC->setText("ACC");
    break;
  }

  switch (module_info_.curr_planning_settings.enable_aeb) {
  case (1):
    // 已关闭 AEB
    ui->pushButtonEnableAEB->setStyleSheet(background_invalid);
    ui->pushButtonEnableAEB->setText("AEB");
    break;
  case (2):
    // 已开启 AEB
    ui->pushButtonEnableAEB->setStyleSheet(background_normal);
    ui->pushButtonEnableAEB->setText(">AEB<");
    break;
  default:
    // 无效状态
    ui->pushButtonEnableAEB->setStyleSheet(background_invalid);
    ui->pushButtonEnableAEB->setText("AEB");
    break;
  }

  switch (module_info_.curr_planning_settings.enable_alc) {
  case (1):
    // 已关闭 ALC
    ui->pushButtonEnableALC->setStyleSheet(background_invalid);
    ui->pushButtonEnableALC->setText("ALC");

    ui->pushButtonChangingLaneLeft->setEnabled(false);
    ui->pushButtonChangingLaneRight->setEnabled(false);
    ui->pushButtonChangingLaneAbort->setEnabled(false);
    break;
  case (2):
    // 已开启 ALC
    ui->pushButtonEnableALC->setStyleSheet(background_normal);
    ui->pushButtonEnableALC->setText(">ALC<");

    ui->pushButtonChangingLaneLeft->setEnabled(true);
    ui->pushButtonChangingLaneRight->setEnabled(true);
    ui->pushButtonChangingLaneAbort->setEnabled(true);
    break;
  default:
    // 无效状态
    ui->pushButtonEnableALC->setStyleSheet(background_invalid);
    ui->pushButtonEnableALC->setText("ALC");

    ui->pushButtonChangingLaneLeft->setEnabled(false);
    ui->pushButtonChangingLaneRight->setEnabled(false);
    ui->pushButtonChangingLaneAbort->setEnabled(false);
    break;
  }

  switch (module_info_.curr_planning_settings.enable_isl) {
  case (1):
    // 已关闭 ISL
    ui->pushButtonEnableISL->setStyleSheet(background_invalid);
    ui->pushButtonEnableISL->setText("ISL");
    break;
  case (2):
    // 已开启 ISL
    ui->pushButtonEnableISL->setStyleSheet(background_normal);
    ui->pushButtonEnableISL->setText(">ISL<");
    break;
  default:
    // 无效状态
    ui->pushButtonEnableISL->setStyleSheet(background_invalid);
    ui->pushButtonEnableISL->setText("ISL");
    break;
  }

  switch (module_info_.curr_planning_settings.enable_ngp) {
  case (1):
    // 已关闭 NGP
    ui->pushButtonEnableNGP->setStyleSheet(background_invalid);
    ui->pushButtonEnableNGP->setText("NGP");
    break;
  case (2):
    // 已开启 NGP
    ui->pushButtonEnableNGP->setStyleSheet(background_normal);
    ui->pushButtonEnableNGP->setText(">NGP<");
    break;
  default:
    // 无效状态
    ui->pushButtonEnableNGP->setStyleSheet(background_invalid);
    ui->pushButtonEnableNGP->setText("NGP");
    break;
  }

  switch (module_info_.curr_planning_settings.enable_fallback) {
  case (1):
    // 已关闭 Level
    ui->pushButtonEnableLevel->setStyleSheet(background_invalid);
    ui->pushButtonEnableLevel->setText("Level");

    ui->pushButtonSetTarLevel->setEnabled(false);
    ui->spinBoxVehLevelSetting->setEnabled(false);
    break;
  case (2):
    // 已开启 Level
    ui->pushButtonEnableLevel->setStyleSheet(background_normal);
    ui->pushButtonEnableLevel->setText(">Level<");

    ui->pushButtonSetTarLevel->setEnabled(true);
    ui->spinBoxVehLevelSetting->setEnabled(true);
    break;
  default:
    // 无效状态
    ui->pushButtonEnableLevel->setStyleSheet(background_invalid);
    ui->pushButtonEnableLevel->setText("Level");

    ui->pushButtonSetTarLevel->setEnabled(false);
    ui->spinBoxVehLevelSetting->setEnabled(false);
    break;
  }

  switch (module_info_.curr_planning_settings.enable_pcc) {
  case (1):
    // 已关闭 Level
    ui->pushButtonEnablePCC->setStyleSheet(background_invalid);
    ui->pushButtonEnablePCC->setText("PCC");
    break;
  case (2):
    // 已开启 Level
    ui->pushButtonEnablePCC->setStyleSheet(background_normal);
    ui->pushButtonEnablePCC->setText(">PCC<");
    break;
  default:
    // 无效状态
    ui->pushButtonEnablePCC->setStyleSheet(background_invalid);
    ui->pushButtonEnablePCC->setText("PCC");
    break;
  }

  // 目标速度
  if (module_info_.curr_planning_settings.target_velocity_valid) {
    ui->labelCurrSettingTarSpeed->setStyleSheet(background_normal);
  } else {
    ui->labelCurrSettingTarSpeed->setStyleSheet(background_invalid);
  }
  std::snprintf(str_buff_, MAX_STR_BUFF_SIZE-1, "%0.0f",
                module_info_.curr_planning_settings.target_velocity*3.6F);
  ui->labelCurrSettingTarSpeed->setText(str_buff_);

  // 目标加速度
  if (module_info_.curr_planning_settings.target_acc_valid) {
    ui->labelCurrSettingTarAcceleration->setStyleSheet(background_normal);
  } else {
    ui->labelCurrSettingTarAcceleration->setStyleSheet(background_invalid);
  }
  std::snprintf(str_buff_, MAX_STR_BUFF_SIZE-1, "%0.1f",
                module_info_.curr_planning_settings.target_acc);
  ui->labelCurrSettingTarAcceleration->setText(str_buff_);

  // 目标跟车时距
  if (module_info_.curr_planning_settings.target_time_gap_valid) {
    ui->labelCurrSettingTarTimeGap->setStyleSheet(background_normal);
  } else {
    ui->labelCurrSettingTarTimeGap->setStyleSheet(background_invalid);
  }
  std::snprintf(str_buff_, MAX_STR_BUFF_SIZE-1, "%0.0f",
                module_info_.curr_planning_settings.target_time_gap);
  ui->labelCurrSettingTarTimeGap->setText(str_buff_);

  // 请求降级等级
  if (module_info_.curr_planning_settings.target_fallback_level_valid) {
    ui->labelCurrSettingTarLevel->setStyleSheet(background_normal);
  } else {
    ui->labelCurrSettingTarLevel->setStyleSheet(background_invalid);
  }
  std::snprintf(str_buff_, MAX_STR_BUFF_SIZE-1, "%d",
                module_info_.curr_planning_settings.target_fallback_level);
  ui->labelCurrSettingTarLevel->setText(str_buff_);
}


/// Settings
void MainWindow::on_pushButtonStartAdas_clicked() {
  switch (module_info_.chassis_status.driving_mode) {
  case (ad_msg::VEH_DRIVING_MODE_MANUAL):
    // 开启 ADAS
    control_start_adas_.SetValue(2);
    break;
  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC):
    // 关闭 ADAS
    control_start_adas_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_start_adas_.SetValue(0);
    break;
  }
}


void MainWindow::on_pushButtonEnableLKA_clicked() {
  switch (module_info_.curr_planning_settings.enable_lka) {
  case (1):
    // 开启 LKA
    control_enable_lka_.SetValue(2);
    break;
  case (2):
    // 关闭 LKA
    control_enable_lka_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_lka_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnableACC_clicked() {
  switch (module_info_.curr_planning_settings.enable_acc) {
  case (1):
    // 开启 ACC
    control_enable_acc_.SetValue(2);
    break;
  case (2):
    // 关闭 ACC
    control_enable_acc_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_acc_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnableAEB_clicked() {
  switch (module_info_.curr_planning_settings.enable_aeb) {
  case (1):
    // 开启 AEB
    control_enable_aeb_.SetValue(2);
    break;
  case (2):
    // 关闭 AEB
    control_enable_aeb_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_aeb_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnableALC_clicked() {
  switch (module_info_.curr_planning_settings.enable_alc) {
  case (1):
    // 开启 ALC
    control_enable_alc_.SetValue(2);
    break;
  case (2):
    // 关闭 ALC
    control_enable_alc_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_alc_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnableISL_clicked() {
  switch (module_info_.curr_planning_settings.enable_isl) {
  case (1):
    // 开启 ISL
    control_enable_isl_.SetValue(2);
    break;
  case (2):
    // 关闭 ISL
    control_enable_isl_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_isl_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnableNGP_clicked() {
  switch (module_info_.curr_planning_settings.enable_ngp) {
  case (1):
    // 开启 NGP
    control_enable_ngp_.SetValue(2);
    break;
  case (2):
    // 关闭 NGP
    control_enable_ngp_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_ngp_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnableLevel_clicked() {
  switch (module_info_.curr_planning_settings.enable_fallback) {
  case (1):
    // 开启 降级
    control_enable_fallback_.SetValue(2);
    break;
  case (2):
    // 关闭 降级
    control_enable_fallback_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_fallback_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonEnablePCC_clicked() {
  switch (module_info_.curr_planning_settings.enable_pcc) {
  case (1):
    // 开启 节油
    control_enable_pcc_.SetValue(2);
    break;
  case (2):
    // 关闭 节油
    control_enable_pcc_.SetValue(1);
    break;
  default:
    // 不发出新指令
    control_enable_pcc_.SetValue(0);
    break;
  }
}

void MainWindow::on_pushButtonSetTarSpeed_clicked() {
  // 目标车速[m/s]
  control_set_tar_speed_.SetValue(
        (Float32_t)ui->spinBoxVehSpeedSetting->value() / 3.6F);
}

void MainWindow::on_pushButtonSetTarAcceleration_clicked() {
  // 加速度[m/s^2]
  control_set_tar_acceleration_.SetValue(
        (Float32_t)ui->doubleSpinBoxAcclSetting->value());
}

void MainWindow::on_pushButtonSetTarLevel_clicked() {
  // 目标等级[-]
  control_set_tar_fallback_level_.SetValue(
        (Int8_t)ui->spinBoxVehLevelSetting->value());//target_fallback_level
}

void MainWindow::on_pushButtonSetTarTimeGap_clicked() {
  // 目标跟车时距[s]
  control_set_tar_time_gap_.SetValue(
        (Float32_t)ui->SpinBoxTimeGapSetting->value());
}

void MainWindow::on_pushButtonChangingLaneLeft_clicked() {
  control_changing_lane_.SetValue(1);
}

void MainWindow::on_pushButtonChangingLaneAbort_clicked() {
  control_changing_lane_.SetValue(3);
}

void MainWindow::on_pushButtonChangingLaneRight_clicked() {
  control_changing_lane_.SetValue(2);
}

void MainWindow::on_checkBoxShowLineChart_clicked() {
  if (ui->checkBoxShowLineChart->isChecked()) {
    widget_lat_err_->show();
    widget_yaw_err_->show();
    widget_yaw_rate_->show();
    widget_acc_info_->show();
    widget_velocity_info_->show();
  } else {
    widget_lat_err_->hide();
    widget_yaw_err_->hide();
    widget_yaw_rate_->hide();
    widget_acc_info_->hide();
    widget_velocity_info_->hide();
  }
}


} // hmi
} // phoenix


