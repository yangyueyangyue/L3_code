/******************************************************************************
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
#include "ui_main_window.h"
#include "QMessageBox"
#include "QDockWidget"
#include "pc/task_manager.h"
#include "vehicle_control_c.h"
#include "vehicle_model_c.h"


namespace phoenix {
namespace hmi {


MainWindow::MainWindow(
    int argc, char **argv,
    const std::string &work_space, QWidget *parent)
    : QMainWindow(parent),
      work_space_(work_space),
      ui(new Ui::MainWindow) {
  ui->setupUi(this);

  // Set main window icon
  setWindowIcon(QIcon(":/images/icon.png"));

  // setWindowOpacity(1);
  // setWindowFlags(Qt::FramelessWindowHint);
  // setAttribute(Qt::WA_TranslucentBackground, true);

  Char_t str_buff[256] = { 0 };
  label_status_ = new QLabel();
  ui->statusbar->addWidget(label_status_);
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DF D17_B1");
  label_status_->setText(str_buff);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DF D17_B2");
  label_status_->setText(str_buff);
#else
  com_snprintf(str_buff, sizeof(str_buff)-1, "Unknow Vehicle Platform");
  label_status_->setText(str_buff);
#endif

  // 方向盘角度输入
  int max_steering_wheel_angle = 1000;
  QValidator *validator_steering_wheel_angle =
      new QIntValidator(-max_steering_wheel_angle,
          max_steering_wheel_angle, this);
  ui->lineEditSteeringWheelAngle->setValidator(validator_steering_wheel_angle);
  ui->lineEditSteeringWheelAngle->setText("0");
  // 速度输入
  QValidator* validator_velocity = new QIntValidator(0, 100, this);
  ui->lineEditTargetVelocity->setValidator(validator_velocity);
  ui->lineEditTargetVelocity->setText("0");

  QFont font_label_status;
  font_label_status.setPointSize(7);
  ui->labelStatus01->setText("");
  ui->labelStatus01->setFont(font_label_status);
  ui->labelStatus01->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus02->setText("");
  ui->labelStatus02->setFont(font_label_status);
  ui->labelStatus02->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus03->setText("");
  ui->labelStatus03->setFont(font_label_status);
  ui->labelStatus03->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus04->setText("");
  ui->labelStatus04->setFont(font_label_status);
  ui->labelStatus04->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus05->setText("");
  ui->labelStatus05->setFont(font_label_status);
  ui->labelStatus05->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus06->setText("");
  ui->labelStatus06->setFont(font_label_status);
  ui->labelStatus06->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus07->setText("");
  ui->labelStatus07->setFont(font_label_status);
  ui->labelStatus07->setStyleSheet("background-color: rgb(128, 128, 128);");
  ui->labelStatus08->setText("");
  ui->labelStatus08->setFont(font_label_status);
  ui->labelStatus08->setStyleSheet("background-color: rgb(128, 128, 128);");

  // Adjust main window position
  // QRect rect;
  // rect.setX(8);
  // rect.setY(30);
  // rect.setSize(this->size());
  // this->setGeometry(rect);

  // Create map window
  widget_map_ = new WidgetMap();

  // 折线图窗
  /// 位置误差
  //widget_lat_err_ = new LineChartWidgetLatErr();
  /// 角度误差
  /// widget_yaw_err_ = new LineChartWidgetYawErr();
  
  // 角速度
  widget_yaw_rate_ = new LineChartWidgetYawRate();
  // BY ZQ lat_dist replace yaw_rate
  widget_lat_dist_ = new LineChartWidgetPrevLatDist();

  // 轨迹前馈
  widget_trj_feedforward_ = new LineChartWidgetTrjFeedforward();
  // BY ZQ heading replace trj_feedforward
  widget_heading_ = new LineChartWidgetPreHeading();

  // 横向偏差反馈
  widget_lat_err_feed_back_ = new LineChartWidgetLatErrFeedBack();
  // BY ZQ PreCurvature replace LatErrFeedBack
  widget_Curvature_ = new LineChartWidgetPreCurvature();

  // 角速度偏差反馈
  widget_yaw_rate_feed_back_ = new LineChartWidgetYawRateFeedBack();
  // BY ZQ SteerAngle replace YawRateFeedBack
 // widget_SteerAngle_ = new LineChartWidgetSteerAngle();
  // 角度偏差反馈
  widget_yaw_err_feed_back_ = new LineChartWidgetYawErrFeedBack();
  // 方向盘转角
  widget_veh_steering_wheel_angle_ = new LineChartWidgetVehSteeringWheelAngle();
  // 车辆速度
  widget_veh_velocity_ = new LineChartWidgetVehVelocity();
  // 车辆加速度
  widget_veh_acceleration_ = new LineChartWidgetVehAcceleration();
  widget_yaw_rate_frequency_domain_ = new HistogramWidgetYawRateFrequencyDomain();
  widget_tar_yaw_rate_frequency_domain_ = new HistogramWidgetYawRateFrequencyDomain();
  /// 纵向控制
  /// widget_lon_ctl_value_ = new LineChartWidgetLonCtlValue();
  // 添加到布局中
  ui->gridLayoutMaps->addWidget(widget_map_, 0, 0, 7, 1);

  // ui->gridLayoutMaps->addWidget(widget_lat_err_, 0, 1);
  // ui->gridLayoutMaps->addWidget(widget_yaw_err_, 1, 1);
  
  // BY ZQ lat_dist instance yaw_rate
  //ui->gridLayoutMaps->addWidget(widget_yaw_rate_, 0, 1);
  ui->gridLayoutMaps->addWidget(widget_lat_dist_, 0, 1);

  // BY ZQ heading instance trj_feedforward
  //ui->gridLayoutMaps->addWidget(widget_trj_feedforward_, 1, 1);
  ui->gridLayoutMaps->addWidget(widget_heading_,1, 1);

  // BY ZQ PreCurvature replace LatErrFeedBack
  //ui->gridLayoutMaps->addWidget(widget_lat_err_feed_back_, 2, 1);
  ui->gridLayoutMaps->addWidget(widget_Curvature_, 2, 1);

  // BY ZQ SteerAngle replace YawRateFeedBack
  ui->gridLayoutMaps->addWidget(widget_yaw_rate_feed_back_, 3, 1);


  ui->gridLayoutMaps->addWidget(widget_veh_steering_wheel_angle_, 4, 1);
  /// ui->gridLayoutMaps->addWidget(widget_veh_velocity_, 5, 1);
  ui->gridLayoutMaps->addWidget(widget_yaw_rate_, 5, 1);
  ui->gridLayoutMaps->addWidget(widget_yaw_rate_frequency_domain_, 6, 1);
  ui->gridLayoutMaps->addWidget(widget_tar_yaw_rate_frequency_domain_, 7, 1);
  /// ui->gridLayoutMaps->addWidget(widget_veh_acceleration_, 6, 1);
  /// ui->gridLayoutMaps->addWidget(widget_lon_ctl_value_, 6, 1);

  // Create task manager
  task_manager_ = new framework::TaskManager(argc, argv, work_space);

  // Start task manager
  bool ret = task_manager_->Start();
  if (false == ret) {
    LOG_ERR << "Failed to start task manager.";

    QMessageBox::critical(
        this, tr("Vehicle Control"),
        tr("Failed to start Task Manager, close the window, "
           "check error logs, and retry!"),
        QMessageBox::Ok, QMessageBox::NoButton, QMessageBox::NoButton);
  }

  // Update windows
  UpdateModuleInfo();
  UpdateButtonStatus();

  refresh_timer_ = this->startTimer(100);
  if (0 == refresh_timer_) {
    LOG_ERR << "MainWindow, Failed to start refresh Timer";
  }
}

MainWindow::~MainWindow() {
  delete task_manager_;
  delete ui;
  qDebug("MainWindow: Deconstructed !");
}

void MainWindow::closeEvent(QCloseEvent *event) {
  qDebug("Received close event!");

  task_manager_->Stop();
}

void MainWindow::resizeEvent(QResizeEvent * event) {
}

void MainWindow::timerEvent(QTimerEvent *event) {
  if (event->timerId() == refresh_timer_) {
    UpdateModuleInfo();
    UpdateButtonStatus();
    widget_map_->Update();
    UpdateLineChartWidgets();
    UpdateVehicleStatus();
  }
}

void MainWindow::UpdateModuleInfo() {
  module_info_.is_module_ready = task_manager_->IsReady();

  Int32_t chassis_ctl_status = VEH_CHASSIS_CTL_STATUS_MANUAL;
  Phoenix_SharedData_GetChassisCtlStatus(&chassis_ctl_status);
  module_info_.is_robotic_mode =
      Phoenix_AdMsg_IsChassisInRoboticMode(chassis_ctl_status);

  // Read some datas
  Phoenix_SharedData_GetChassis(&module_info_.chassis_status);
  Phoenix_SharedData_GetChassisCtlCmd(&module_info_.chassis_cmd);
  Phoenix_SharedData_GetLateralControlInfo(&module_info_.lateral_control_info);
  Phoenix_SharedData_GetSteeringControlInfo(&module_info_.steering_ctl_info);
}

void MainWindow::UpdateButtonStatus() {
  if (!module_info_.is_module_ready) {
    ui->pushButtonStartRobot->setEnabled(false);

    ui->radioButtonEnableAutoMode->setEnabled(false);
    ui->radioButtonEnableDebugMode->setEnabled(false);

    ui->checkBoxEnableRemoteControl->setEnabled(false);
    ui->checkBoxEnableEps->setEnabled(false);
    ui->checkBoxEnableThrottleSys->setEnabled(false);
    ui->checkBoxEnableEbs->setEnabled(false);

    // ui->pushButtonSelectGearP->setEnabled(false);
    // ui->pushButtonSelectGearN->setEnabled(false);
    // ui->pushButtonSelectGearR->setEnabled(false);
    // ui->pushButtonSelectGearD->setEnabled(false);

    ui->pushButtonTurnSteeringWheel->setEnabled(false);
    ui->pushButtonAccelerate->setEnabled(false);
    ui->pushButtonControlVelocity->setEnabled(false);
    ui->pushButtonBrake->setEnabled(false);
    ui->pushButtonReleaseLongitudinalCtl->setEnabled(false);
  } else {
    ui->pushButtonStartRobot->setEnabled(true);

    ui->radioButtonEnableAutoMode->setEnabled(true);
    ui->radioButtonEnableDebugMode->setEnabled(true);

    ui->checkBoxEnableRemoteControl->setEnabled(true);
    ui->checkBoxEnableEps->setEnabled(true);
    ui->checkBoxEnableThrottleSys->setEnabled(true);
    ui->checkBoxEnableEbs->setEnabled(true);

    if (module_info_.chassis_cmd.enable_direct_ctl) {
      // ui->pushButtonSelectGearP->setEnabled(true);
      // ui->pushButtonSelectGearN->setEnabled(true);
      // ui->pushButtonSelectGearR->setEnabled(true);
      // ui->pushButtonSelectGearD->setEnabled(true);

      ui->pushButtonTurnSteeringWheel->setEnabled(true);
      ui->pushButtonAccelerate->setEnabled(true);
      ui->pushButtonControlVelocity->setEnabled(true);
    } else {
      // ui->pushButtonSelectGearP->setEnabled(false);
      // ui->pushButtonSelectGearN->setEnabled(false);
      // ui->pushButtonSelectGearR->setEnabled(false);
      // ui->pushButtonSelectGearD->setEnabled(false);

      ui->pushButtonTurnSteeringWheel->setEnabled(false);
      ui->pushButtonAccelerate->setEnabled(false);
      ui->pushButtonControlVelocity->setEnabled(false);
    }
    ui->pushButtonBrake->setEnabled(true);
    ui->pushButtonReleaseLongitudinalCtl->setEnabled(true);
  }
  // Mode Show
  if (module_info_.is_robotic_mode) {
    ui->pushButtonStartRobot->setText(tr("Stop"));
  } else {
    ui->pushButtonStartRobot->setText(tr("Start"));
  }
  // Chassis Actuator Show
  if (module_info_.chassis_cmd.enable_direct_ctl) {
    ui->radioButtonEnableAutoMode->setChecked(false);
    ui->radioButtonEnableDebugMode->setChecked(true);
  } else {
    ui->radioButtonEnableAutoMode->setChecked(true);
    ui->radioButtonEnableDebugMode->setChecked(false);
  }

  // if (module_info_.chassis_cmd.enable_remote_control) {
  //   ui->checkBoxEnableRemoteControl->setChecked(true);
  // } else {
  //   ui->checkBoxEnableRemoteControl->setChecked(false);
  // }
  // Chassis Actuator EPS Show
  if (module_info_.chassis_cmd.enable_eps) {
    ui->checkBoxEnableEps->setChecked(true);
  } else {
    ui->checkBoxEnableEps->setChecked(false);
  }
  // Chassis Actuator Throttle Show
  if (module_info_.chassis_cmd.enable_throttle_sys) {
    ui->checkBoxEnableThrottleSys->setChecked(true);
  } else {
    ui->checkBoxEnableThrottleSys->setChecked(false);
  }
  // Chassis Actuator EBS Show
  if (module_info_.chassis_cmd.enable_ebs) {
    ui->checkBoxEnableEbs->setChecked(true);
  } else {
    ui->checkBoxEnableEbs->setChecked(false);
  }
// Gear Show
  switch (module_info_.chassis_cmd.gear) {
  case (VEH_GEAR_P):
    ui->pushButtonSelectGearP->setStyleSheet(
        "background-color: rgb(82, 235, 233);");
    ui->pushButtonSelectGearN->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearR->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearD->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    break;
  case (VEH_GEAR_N):
    ui->pushButtonSelectGearP->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearN->setStyleSheet(
        "background-color: rgb(82, 235, 233);");
    ui->pushButtonSelectGearR->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearD->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    break;
  case (VEH_GEAR_R):
    ui->pushButtonSelectGearP->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearN->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearR->setStyleSheet(
        "background-color: rgb(82, 235, 233);");
    ui->pushButtonSelectGearD->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    break;
  case (VEH_GEAR_D):
    ui->pushButtonSelectGearP->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearN->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearR->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearD->setStyleSheet(
        "background-color: rgb(82, 235, 233);");
    break;
  default:
    ui->pushButtonSelectGearP->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearN->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearR->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    ui->pushButtonSelectGearD->setStyleSheet(
        "background-color: rgb(235, 235, 235);");
    break;
  }
}

void MainWindow::UpdateLineChartWidgets() {
  const LateralControlInfo_t& lat_ctl_info = module_info_.lateral_control_info;

  //widget_lat_err_->Refresh(
  //      module_info_.lateral_control_info.lat_ctl_pid_info.lat_err,
  //      module_info_.lateral_control_info.lat_ctl_pid_info.lat_err_v);
  //widget_yaw_err_->Refresh(
  //      module_info_.lateral_control_info.lat_ctl_pid_info.yaw_err,
  //      module_info_.lateral_control_info.lat_ctl_pid_info.yaw_err_v);
  widget_yaw_rate_->Refresh(
        lat_ctl_info.target_yaw_rate, lat_ctl_info.yaw_rate,
        0.0/*lat_ctl_info.yaw_rate_imu*/,
        0.0/*lat_ctl_info.yaw_rate_chassis*/,
        0.0/*lat_ctl_info.yaw_rate_steering*/,
        0.0/*lat_ctl_info.yaw_rate_d_chg_rate*//*0.0*//*lat_ctl_info.lat_ctl_pid_info.tar_yaw_rate_diff_v*/,
        0.0/*lat_ctl_info.yaw_rate_chg_rate*/,
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.hunting_energy);
  widget_trj_feedforward_->Refresh(
        lat_ctl_info.lat_ctl_pid_info.trj_feed.feed_value,
        lat_ctl_info.lat_ctl_pid_info.trj_feed.feed_value_smooth,
        lat_ctl_info.lat_ctl_pid_info.trj_feed.goal_dist,
        lat_ctl_info.lat_ctl_pid_info.trj_feed.goal_dist_near,
        lat_ctl_info.lat_ctl_pid_info.trj_feed.goal_dist_far);
  widget_lat_err_feed_back_->Refresh(
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.p_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.i_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.d_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.feed_value,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.lat_err,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.lat_err_spd,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.lat_err_lv_idx,
        lat_ctl_info.lat_ctl_pid_info.lat_err_feed.lat_err_spd_lv_idx);
  widget_yaw_err_feed_back_->Refresh(
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.p_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.i_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.d_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.feed_value,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.yaw_err,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.yaw_err_spd,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.yaw_err_lv_idx,
        lat_ctl_info.lat_ctl_pid_info.yaw_err_feed.yaw_err_spd_lv_idx);
  widget_yaw_rate_feed_back_->Refresh(
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.p_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.i_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.d_feed.feed,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.feed_value,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.yaw_rate_err,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.yaw_rate_spd,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.yaw_rate_err_lv_idx,
        lat_ctl_info.lat_ctl_pid_info.yaw_rate_err_feed.yaw_rate_spd_lv_idx);
  widget_veh_steering_wheel_angle_->Refresh(
        lat_ctl_info.lat_ctl_pid_info.tar_steering_angle,
        /*lat_ctl_info.lat_ctl_pid_info.tar_steering_angle_smooth,*/
        module_info_.chassis_cmd.steering_wheel_angle,
        module_info_.chassis_status.steering_wheel_angle,
        0.0/*module_info_.steering_ctl_info.target_steering_speed*/,
        0.0/*module_info_.chassis_status.steering_wheel_speed*/);
  //widget_veh_velocity_->Refresh(
  //      module_info_.chassis_cmd.velocity,
  //      module_info_.chassis_status.v);
  widget_yaw_rate_frequency_domain_->Refresh(
        LAT_CTL_FREQUENCY_SPECTRUM_SIZE,
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.hunting_fre_idxs,
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.cur_yaw_rate_mag);
  Float32_t yaw_rate_mag[LAT_CTL_FREQUENCY_SPECTRUM_SIZE];
  for (Int32_t i = 0; i < LAT_CTL_FREQUENCY_SPECTRUM_SIZE; ++i) {
    yaw_rate_mag[i] = 
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.cur_yaw_rate_mag[i] -
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.tar_yaw_rate_mag[i];
  }
  widget_tar_yaw_rate_frequency_domain_->Refresh(
        LAT_CTL_FREQUENCY_SPECTRUM_SIZE,
        lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.hunting_fre_idxs,
        yaw_rate_mag/*lat_ctl_info.lat_ctl_pid_info.frequency_spectrum.yaw_rate.tar_yaw_rate_mag*/);
  // widget_veh_acceleration_->Refresh(
  //       module_info_.chassis_cmd.acceleration,
  //       module_info_.chassis_status.a);
  // widget_lon_ctl_value_->Refresh(
  //       module_info_.chassis_cmd.brake_value,
  //       module_info_.chassis_cmd.acc_value);
}

void MainWindow::UpdateVehicleStatus() {
  char str_buff[256] = { 0 };

  QString background_normal = "background-color: rgb(82, 235, 233);";
  QString background_warn = "background-color: rgb(255, 255, 0);";
  QString background_err = "background-color: rgb(255, 100, 50);";
  QString background_invalid = "background-color: rgb(128, 128, 128);";


  switch (module_info_.chassis_status.e_stop) {
  case (VEH_E_STOP_OFF):
    ui->labelSwitchEStop->setStyleSheet(background_normal);
    break;
  case (VEH_E_STOP_ON):
    ui->labelSwitchEStop->setStyleSheet(background_err);
    break;
  default:
    ui->labelSwitchEStop->setStyleSheet(background_invalid);
    break;
  }
}

void MainWindow::on_pushButtonStartRobot_clicked() {
  if (module_info_.is_robotic_mode) {
    task_manager_->StopRobotCtl();
  } else {
    task_manager_->StartRobotCtl();
  }
}

void MainWindow::on_checkBoxEnableEps_clicked() {
  if (ui->checkBoxEnableEps->isChecked()) {
    task_manager_->EnableEps(true);
  } else {
    task_manager_->EnableEps(false);
  }
}

void MainWindow::on_checkBoxEnableThrottleSys_clicked() {
  if (ui->checkBoxEnableThrottleSys->isChecked()) {
    task_manager_->EnableThrottleSys(true);
  } else {
    task_manager_->EnableThrottleSys(false);
  }
}

void MainWindow::on_checkBoxEnableEbs_clicked() {
  if (ui->checkBoxEnableEbs->isChecked()) {
    task_manager_->EnableEbs(true);
  } else {
    task_manager_->EnableEbs(false);
  }
}

void MainWindow::on_radioButtonEnableAutoMode_clicked() {
  ui->radioButtonEnableAutoMode->setChecked(true);
  ui->radioButtonEnableDebugMode->setChecked(false);
  task_manager_->EnableDirectCtl(false);
}

void MainWindow::on_radioButtonEnableDebugMode_clicked() {
  ui->radioButtonEnableAutoMode->setChecked(false);
  ui->radioButtonEnableDebugMode->setChecked(true);
  task_manager_->EnableDirectCtl(true);
}

void MainWindow::on_checkBoxEnableRemoteControl_clicked() {
  if (ui->checkBoxEnableRemoteControl->isChecked()) {
    task_manager_->EnableRemoteCtl(true);
  } else {
    task_manager_->EnableRemoteCtl(false);
  }
}

void MainWindow::on_pushButtonSelectGearP_clicked() {
  task_manager_->ChangeGear(VEH_GEAR_P);
}

void MainWindow::on_pushButtonSelectGearN_clicked() {
  task_manager_->ChangeGear(VEH_GEAR_N);
}

void MainWindow::on_pushButtonSelectGearR_clicked() {
  task_manager_->ChangeGear(VEH_GEAR_R);
}

void MainWindow::on_pushButtonSelectGearD_clicked() {
  task_manager_->ChangeGear(VEH_GEAR_D);
}

void MainWindow::on_pushButtonTurnSteeringWheel_clicked() {
  double angle = ui->lineEditSteeringWheelAngle->text().toInt();
  task_manager_->TurnSteeringWheel(common::com_deg2rad(angle),
                                   common::com_deg2rad(90.0));
}

void MainWindow::on_pushButtonAccelerate_clicked() {
  double value = ui->spinBoxAcceleratorValue->text().toInt();
  task_manager_->Accelerate(value);
}

void MainWindow::on_pushButtonControlVelocity_clicked() {
  int velocity = ui->lineEditTargetVelocity->text().toInt();
  task_manager_->SpeedUp(velocity*0.277778, 0.0);
}

void MainWindow::on_pushButtonBrake_clicked() {
  int value = ui->spinBoxBrakeValue->text().toInt();
  task_manager_->Brake(value);
}

void MainWindow::on_pushButtonReleaseLongitudinalCtl_clicked() {
  task_manager_->Accelerate(0.0);
  task_manager_->SpeedUp(0.0, 0.0);
  task_manager_->Brake(0.0);
}

void MainWindow::on_pushButtonInterrupt_clicked() {
  task_manager_->ReqManualInterrupt(true);
}


} // namespace hmi
} // namespace phoenix


