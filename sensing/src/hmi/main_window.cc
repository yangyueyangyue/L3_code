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

  InitSettingViews();

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

  /**
   * @brief 此处确定了界面UI的刷新频率为100ms
   */
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

void MainWindow::InitSettingViews()
{

  UISettingManager * setting_manager = UISettingManager::instance();

  ui->checkBoxMainLane->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingMainLaneState()));
  ui->checkBoxLaneCurb->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingLaneCurbState()));
  ui->checkBoxMainCamera->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingMainCameraState()));
  ui->checkBoxMainLidar->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingMainLidarState()));
  ui->checkBoxLeftFrontRadar->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingLeftFrontRadarState()));
  ui->checkBoxRightFrontRadar->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingRightFrontRadarState()));
  ui->checkBoxLeftBackRadar->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingLeftBackRadarState()));
  ui->checkBoxRightBackRadar->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingRightBackRadarState()));
  ui->checkBoxVCLane->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCLaneState()));
  ui->checkBoxVCFront->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCFrontState()));
  ui->checkBoxVCLeftFront->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCLeftFrontState()));
  ui->checkBoxVCRightFront->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCRightFrontState()));
  ui->checkBoxVCLeftBack->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCLeftBackState()));
  ui->checkBoxVCRightBack->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCRightBackState()));
  ui->checkBoxFrontRadar->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingFrontRadarState()));
  ui->checkBoxPCFusion->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingPCFusionState()));
  ui->checkBoxADECUFusion->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingADECUFusionState()));

  ui->checkBoxMainLaneTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingMainLaneTxtState()));
  ui->checkBoxLaneCurbTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingLaneCurbTxtState()));
  ui->checkBoxMainCameraTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingMainCameraTxtState()));
  ui->checkBoxMainLidarTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingMainLidarTxtState()));
  ui->checkBoxLeftFrontRadarTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingLeftFrontRadarTxtState()));
  ui->checkBoxRightFrontRadarTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingRightFrontRadarTxtState()));
  ui->checkBoxLeftBackRadarTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingLeftBackRadarTxtState()));
  ui->checkBoxRightBackRadarTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingRightBackRadarTxtState()));
  ui->checkBoxVCLaneTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCLaneTxtState()));
  ui->checkBoxVCFrontTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCFrontTxtState()));
  ui->checkBoxVCLeftFrontTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCLeftFrontTxtState()));
  ui->checkBoxVCRightFrontTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCRightFrontTxtState()));
  ui->checkBoxVCLeftBackTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCLeftBackTxtState()));
  ui->checkBoxVCRightBackTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingVCRightBackTxtState()));
  ui->checkBoxFrontRadarTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingFrontRadarTxtState()));
  ui->checkBoxPCFusionTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingPCFusionTxtState()));
  ui->checkBoxADECUFusionTxt->setCheckState(static_cast<Qt::CheckState>(setting_manager->GetSettingADECUFusionTxtState()));
  
  ui->radioBtnFusion0->setChecked(setting_manager->GetSettingPublishFusionTopicFromAdecu()>0);
  ui->radioBtnFusion1->setChecked(setting_manager->GetSettingPublishFusionTopicFromAdecu()<=0);



}


void MainWindow::on_checkBoxMainLane_stateChanged(int arg1)
{
  std::cout << "on_checkBoxMainLane_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingMainLaneState(arg1);
}

void MainWindow::on_checkBoxMainLaneTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxMainLaneTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingMainLaneTxtState(arg1);
}

void MainWindow::on_checkBoxLaneCurb_stateChanged(int arg1)
{
  std::cout << "on_checkBoxLaneCurb_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingLaneCurbState(arg1);
}

void MainWindow::on_checkBoxLaneCurbTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxLaneCurbTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingLaneCurbTxtState(arg1);
}

void MainWindow::on_checkBoxMainCamera_stateChanged(int arg1)
{
  std::cout << "on_checkBoxMainCamera_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingMainCameraState(arg1);
}

void MainWindow::on_checkBoxMainCameraTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxMainCameraTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingMainCameraTxtState(arg1);
}


void MainWindow::on_checkBoxMainLidar_stateChanged(int arg1)
{
  std::cout << "on_checkBoxMainLidar_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingMainLidarState(arg1);
}

void MainWindow::on_checkBoxMainLidarTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxMainLidarTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingMainLidarTxtState(arg1);
}

void MainWindow::on_checkBoxLeftFrontRadar_stateChanged(int arg1)
{
  std::cout << "on_checkBoxLeftFrontRadar_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingLeftFrontRadarState(arg1);
}

void MainWindow::on_checkBoxLeftFrontRadarTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxFrontRadarTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingLeftFrontRadarTxtState(arg1);
}

void MainWindow::on_checkBoxRightFrontRadar_stateChanged(int arg1)
{
  std::cout << "on_checkBoxRightFrontRadar_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingRightFrontRadarState(arg1);
}

void MainWindow::on_checkBoxRightFrontRadarTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxRightFrontRadarTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingRightFrontRadarTxtState(arg1);
}

void MainWindow::on_checkBoxLeftBackRadar_stateChanged(int arg1)
{
  std::cout << "on_checkBoxLeftBackRadar_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingLeftBackRadarState(arg1);
}

void MainWindow::on_checkBoxLeftBackRadarTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxLeftBackRadarTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingLeftBackRadarTxtState(arg1);
}

void MainWindow::on_checkBoxRightBackRadar_stateChanged(int arg1)
{
  std::cout << "on_checkBoxRightBackRadar_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingRightBackRadarState(arg1);
}

void MainWindow::on_checkBoxRightBackRadarTxt_stateChanged(int arg1)
{
  std::cout << "on_checkRightBackRadarTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingRightBackRadarTxtState(arg1);
}

void MainWindow::on_checkBoxVCLane_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCLane_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCLaneState(arg1);
}

void MainWindow::on_checkBoxVCLaneTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCLaneTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCLaneTxtState(arg1);
}

void MainWindow::on_checkBoxVCFront_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCFront_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCFrontState(arg1);
}

void MainWindow::on_checkBoxVCFrontTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCFrontTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCFrontTxtState(arg1);
}

void MainWindow::on_checkBoxVCLeftFront_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCLeftFront_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCLeftFrontState(arg1);
}

void MainWindow::on_checkBoxVCLeftFrontTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCLeftFrontTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCLeftFrontTxtState(arg1);
}

void MainWindow::on_checkBoxVCRightFront_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCRightFront_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCRightFrontState(arg1);
}

void MainWindow::on_checkBoxVCRightFrontTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCRightFrontTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCRightFrontTxtState(arg1);
}

void MainWindow::on_checkBoxVCLeftBack_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCLeftBack_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCLeftBackState(arg1);
}

void MainWindow::on_checkBoxVCLeftBackTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCLeftBackTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCLeftBackTxtState(arg1);
}

void MainWindow::on_checkBoxVCRightBack_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCRightBack_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCRightBackState(arg1);
}

void MainWindow::on_checkBoxVCRightBackTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxVCRightBackTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingVCRightBackTxtState(arg1);
}

void MainWindow::on_checkBoxFrontRadar_stateChanged(int arg1)
{
  std::cout << "on_checkBoxFrontRadar_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingFrontRadarState(arg1);
}

void MainWindow::on_checkBoxFrontRadarTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxFrontRadarTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingFrontRadarTxtState(arg1);
}

void MainWindow::on_checkBoxPCFusion_stateChanged(int arg1)
{
  std::cout << "on_checkBoxPCFusion_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingPCFusionState(arg1);
}

void MainWindow::on_checkBoxPCFusionTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxPCFusionTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingPCFusionTxtState(arg1);
}

void MainWindow::on_checkBoxADECUFusion_stateChanged(int arg1)
{
  std::cout << "on_checkBoxADECUFusion_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingADECUFusionState(arg1);
}

void MainWindow::on_checkBoxADECUFusionTxt_stateChanged(int arg1)
{
  std::cout << "on_checkBoxADECUFusionTxt_stateChanged arg1:" << arg1 << std::endl;
  UISettingManager::instance()->SetSettingADECUFusionTxtState(arg1);
}

void MainWindow::on_radioBtnFusion0_toggled(bool checked)
{
  std::cout << "on_radioBtnFusion0_toggled checked:" << checked << std::endl;
  UISettingManager::instance()->SetSettingPublishFusionTopicFromAdecu(checked);
}

void MainWindow::on_radioBtnFusion1_toggled(bool checked)
{
  std::cout << "on_radioBtnFusion1_toggled checked:" << checked << std::endl;
  UISettingManager::instance()->SetSettingPublishFusionTopicFromAdecu(!checked);
}



} // hmi
} // phoenix


