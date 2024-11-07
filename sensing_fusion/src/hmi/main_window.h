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
};


} // hmi
} // phoenix


#endif  // MAIN_WINDOW_H_
