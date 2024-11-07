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
#ifndef MAIN_WINDOW_H_
#define MAIN_WINDOW_H_

#include <string>
#include <vector>
#include <QMainWindow>
#include <QCloseEvent>
#include <QLabel>
#include "widget_map.h"
#include "uisetting_manager.h"



namespace Ui {
class MainWindow;
}


namespace phoenix {
namespace hmi {


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
  void on_checkBoxMainLane_stateChanged(int arg1);
  void on_checkBoxMainLaneTxt_stateChanged(int arg1);
  void on_checkBoxLaneCurb_stateChanged(int arg1);
  void on_checkBoxLaneCurbTxt_stateChanged(int arg1);
  void on_checkBoxMainCamera_stateChanged(int arg1);
  void on_checkBoxMainCameraTxt_stateChanged(int arg1);
  void on_checkBoxMainLidar_stateChanged(int arg1);
  void on_checkBoxMainLidarTxt_stateChanged(int arg1);
  void on_checkBoxLeftFrontRadar_stateChanged(int arg1);
  void on_checkBoxLeftFrontRadarTxt_stateChanged(int arg1);
  void on_checkBoxRightFrontRadar_stateChanged(int arg1);
  void on_checkBoxRightFrontRadarTxt_stateChanged(int arg1);
  void on_checkBoxLeftBackRadar_stateChanged(int arg1);
  void on_checkBoxLeftBackRadarTxt_stateChanged(int arg1);
  void on_checkBoxRightBackRadar_stateChanged(int arg1);
  void on_checkBoxRightBackRadarTxt_stateChanged(int arg1);
  void on_checkBoxVCLane_stateChanged(int arg1);
  void on_checkBoxVCLaneTxt_stateChanged(int arg1);
  void on_checkBoxVCFront_stateChanged(int arg1);
  void on_checkBoxVCFrontTxt_stateChanged(int arg1);
  void on_checkBoxVCLeftFront_stateChanged(int arg1);
  void on_checkBoxVCLeftFrontTxt_stateChanged(int arg1);
  void on_checkBoxVCRightFront_stateChanged(int arg1);
  void on_checkBoxVCRightFrontTxt_stateChanged(int arg1);
  void on_checkBoxVCLeftBack_stateChanged(int arg1);
  void on_checkBoxVCLeftBackTxt_stateChanged(int arg1);
  void on_checkBoxVCRightBack_stateChanged(int arg1);
  void on_checkBoxVCRightBackTxt_stateChanged(int arg1);
  void on_checkBoxFrontRadar_stateChanged(int arg1);
  void on_checkBoxFrontRadarTxt_stateChanged(int arg1);
  void on_checkBoxPCFusion_stateChanged(int arg1);
  void on_checkBoxPCFusionTxt_stateChanged(int arg1);
  void on_checkBoxADECUFusion_stateChanged(int arg1);
  void on_checkBoxADECUFusionTxt_stateChanged(int arg1);
  void on_radioBtnFusion0_toggled(bool checked);
  void on_radioBtnFusion1_toggled(bool checked);

protected:
  void closeEvent(QCloseEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void moveEvent(QMoveEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void timerEvent(QTimerEvent *event) override;

  void UpdateModuleInfo();
  void UpdateStatusBar();

 private:
  std::string work_space_;
  Ui::MainWindow *ui;
  int refresh_timer_ = 0;

  WidgetMap* widget_map_;

  QLabel* label_status_;

  enum { MAX_LOG_BUFF_SIZE = 1024*4 };
  char log_buff_[MAX_LOG_BUFF_SIZE];


  void InitSettingViews(); 

};


} // hmi
} // phoenix


#endif  // MAIN_WINDOW_H_
