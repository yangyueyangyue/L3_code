/******************************************************************************
 ** linechartwidget.h
 ******************************************************************************
 *  折线图界面
 *  在界面上绘制折线图
 *
 *  @file       linechartwidget.h
 *
 *  @author     pengc
 *  @date       2015.12.16
 *  @version    001 2015.12.16  新规作成
 *
 ******************************************************************************
 *  All Right Reserved. Copyright(C) 2015 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_HMI_WIDGET_LINE_CHART_H_
#define PHOENIX_HMI_WIDGET_LINE_CHART_H_


#include <vector>
#include "QtOpenGL/QGLWidget"
//#include "QOpenGLWidget"
//#include "QOpenGLWindow"
//#include "QOpenGLFunctions"

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"

#include "utils/log.h"
#include "container/ring_buffer.h"
#include "geometry/geometry_utils.h"


namespace phoenix {
namespace hmi {


class WidgetLineChart : public QGLWidget {
  Q_OBJECT

public:
  enum { MAX_DATA_NUM = 100 };

  typedef struct Rectangle {
    double left;
    double bottom;
    double right;
    double top;
    Rectangle() {
      left = 0.0;
      bottom = 0.0;
      right = 0.0;
      top = 0.0;
    }
    bool isValid() {
      return (((right - left) > phoenix::common::kGeometryEpsilon) &&
              ((top - bottom) > phoenix::common::kGeometryEpsilon));
    }
  } RECTANGLE;

  typedef struct Param {
    struct {
      int R;
      int G;
      int B;
    }color;
    double line_width;
    double point_size;
    Param() {
      color.R = 255;
      color.G = 255;
      color.B = 255;
      line_width = 1.0;
      point_size = 2.0;
    }
  } PARAM;

public:
  explicit WidgetLineChart(QWidget* parent = 0);
  virtual ~WidgetLineChart();

  bool Config(double min, double max, int channel, PARAM* param);
  void SetMinValue(double lower, double upper);
  void UpdateWindow();

protected:
  // 更新显示区域
  virtual void UpdateViewArea() {}
  // 绘制
  virtual void DrawSomething() {}
  virtual void DrawSomeText() {}
  // 绘制坐标轴
  void DrawCoordinateAxis();
  // 绘制坐标轴文本
  void DrawYAxisCoordinateValue();
  // 绘制数据
  void DrawDatas();
  // 压入数据
  void PushBack(double* new_data);

private:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();
  void wheelEvent(QWheelEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  // 更新视口
  void UpdateViewWindow();

  int GetWindowWidthInterval() { return (5); }
  int GetWindowHeightInterval() { return (2); }
  int GetWindowWidth() { return width() - 2 * GetWindowWidthInterval(); }
  int GetWindowHeight() { return height() - 2 * GetWindowHeightInterval(); }

 protected:
  // 用于地图缩放
  GLfloat scene_scaling_;                // 场景放大比例
  float scene_screen_scale_;             // 场景与屏幕的比例
  // 用于地图移动
  QPoint mouse_last_position_;           // 记录鼠标的位置
  GLdouble scene_move_dx_;               // 记录移动的位置
  GLdouble scene_move_dy_;               // 记录移动的位置
  // 用于控制显示地图的窗口
  RECTANGLE scene_area_;
  RECTANGLE scene_window_;               // 记录视口的窗口在场景中的位置
  // X轴分度
  double scale_division_axis_x_;
  // 数据个数
  int max_data_number_;
  // 通道数量
  int channel_number_;
  // 最大数据值
  double data_max_value_;
  // 最小数据值
  double data_min_value_;

  double conf_min_value_lower_ = -1.0;
  double conf_min_value_upper_ = 1.0;
  // 数据存放区域
  std::vector<common::RingBuffer<double, MAX_DATA_NUM>* > data_buff_;
  double* coordinate_axis_x_;
  PARAM* param_;
  // 字符串缓存，用来在窗口显示字符信息
  char string_buffer_[1024*2];
};


} // hmi
} // phoenix


#endif  // PHOENIX_HMI_WIDGET_LINE_CHART_H_
