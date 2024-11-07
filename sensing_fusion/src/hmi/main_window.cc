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

#include "utils/log.h"
#include "communication/shared_data.h"


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
  ui->gridLayoutMaps->addWidget(widget_map_, 0, 0);

  Char_t str_buff[256] = { 0 };
  label_status_ = new QLabel();
  ui->statusbar->addWidget(label_status_);

#if 0
#if (DEV_IMU_TYPE == DEV_IMU_TYPE_MPSK)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DEV_IMU: MPSK");
  label_status_->setText(str_buff);
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_BDSTAR)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DEV_IMU: BDStar");
  label_status_->setText(str_buff);
#elif (DEV_IMU_TYPE == DEV_IMU_TYPE_INTERTIAL_LAB)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DEV_IMU: Intertial Lab");
  label_status_->setText(str_buff);
#else
  com_snprintf(str_buff, sizeof(str_buff)-1, "DEV_IMU: Undefined");
  label_status_->setText(str_buff);
#endif
#endif

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_QIN_EV)
  com_snprintf(str_buff, sizeof(str_buff)-1, "Qin Ev");
  label_status_->setText(str_buff);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_FT_AUMAN)
  com_snprintf(str_buff, sizeof(str_buff)-1, "FT Auman");
  label_status_->setText(str_buff);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_X320)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DF X320");
  label_status_->setText(str_buff);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_XD_EANT)
  com_snprintf(str_buff, sizeof(str_buff)-1, "XD_EAnt");
  label_status_->setText(str_buff);
#elif (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17)
  com_snprintf(str_buff, sizeof(str_buff)-1, "DF D17");
  label_status_->setText(str_buff);
#else
  com_snprintf(str_buff, sizeof(str_buff)-1, "Unknow Vehicle Platform");
  label_status_->setText(str_buff);
#endif

  refresh_timer_ = this->startTimer(100);
  if (0 == refresh_timer_) {
    LOG_ERR << "MainWindow, failed to start refresh Timer";
  }
}

MainWindow::~MainWindow() {
  delete ui;
  LOG_INFO(3) << "MainWindow Deconstructed !";
}

void MainWindow::OpenWidgetSettingsPrefrence() {
}

void MainWindow::OpenWidgetVehicleParameters() {
}

void MainWindow::OpenWidgetDebugInfo()  {
}

void MainWindow::OpenWidgetAbout() {
}


void MainWindow::timerEvent(QTimerEvent *event) {
  if (event->timerId() == refresh_timer_) {
    UpdateModuleInfo();
    UpdateStatusBar();

    widget_map_->Update();
  }
}

void MainWindow::closeEvent(QCloseEvent *event) {
  LOG_INFO(3) << "Received close event!";
}

void MainWindow::mousePressEvent(QMouseEvent *event) {
  QMainWindow::mousePressEvent(event);
}

void MainWindow::moveEvent(QMoveEvent * event) {


  QMainWindow::moveEvent(event);
}

void MainWindow::resizeEvent(QResizeEvent * event) {
  QMainWindow::resizeEvent(event);
}

void MainWindow::UpdateModuleInfo()  {
  phoenix::framework::SharedData*
      shared_data = phoenix::framework::SharedData::instance();
}

void MainWindow::UpdateStatusBar() {
  char str_buff[256] = { 0 };
  QString background_normal = "background-color: rgb(82, 235, 233);";
  QString background_warn = "background-color: rgb(255, 255, 0);";
  QString background_err = "background-color: rgb(255, 0, 0);";
  QString background_timeout = "background-color: rgb(255, 128, 128);";
}


} // hmi
} // phoenix

