/******************************************************************************
 ** widget_histogram.h
 ******************************************************************************
 *  柱状图界面
 *  在界面上绘制柱状图
 *
 *  @file       widget_histogram.h
 *
 *  @author     pengc
 *  @date       2023.08.31
 *  @version    001 2023.08.31  新规作成
 *
 ******************************************************************************
 *  All Right Reserved. Copyright(C) 2023 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_WIDGET_HISTOGRAM_H_
#define PHOENIX_WIDGET_HISTOGRAM_H_


#include <vector>

#include "QtOpenGL/QGLWidget"

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"

#include "utils/log.h"
#include "geometry/geometry_utils.h"


namespace phoenix {
namespace hmi {


class WidgetHistogram : public QGLWidget {
  Q_OBJECT

public:
  struct Param {
    struct {
      int R;
      int G;
      int B;
    } color;
    double line_width;
    double point_size;
    Param() {
      color.R = 255;
      color.G = 255;
      color.B = 255;
      line_width = 1.0;
      point_size = 2.0;
    }
  };

public:
  explicit WidgetHistogram(QWidget* parent = 0);
  virtual ~WidgetHistogram();

  bool Config(double min, double max, int max_data_num,
              int param_num,  Param* param);
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
  void DrawDatas(int data_num, const double* data, const int* param_idxs);
  void DrawDatas(int data_num, const float* data, const int* param_idxs);

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

private:
  struct Rectangle {
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
      return (((right - left) > common::kGeometryEpsilon) &&
              ((top - bottom) > common::kGeometryEpsilon));
    }
  };

protected:
  // 用于地图缩放
  GLfloat scene_scaling_;                // 场景放大比例
  float scene_screen_scale_;             // 场景与屏幕的比例
  // 用于地图移动
  QPoint mouse_last_position_;           // 记录鼠标的位置
  GLdouble scene_move_dx_;               // 记录移动的位置
  GLdouble scene_move_dy_;               // 记录移动的位置
  // 用于控制显示地图的窗口
  Rectangle scene_area_;
  Rectangle scene_window_;               // 记录视口的窗口在场景中的位置
  // X轴分度
  double scale_division_axis_x_;
  // 数据个数
  int max_data_number_;
  // 最大数据值
  double data_max_value_;
  // 最小数据值
  double data_min_value_;
  // 数据存放区域
  std::vector<Param> param_;
  std::vector<double> coordinate_axis_x_;
  double base_coor_y_;
  // 字符串缓存，用来在窗口显示字符信息
  char string_buffer_[1024*2];
};


} // hmi
} // phoenix


#endif  // PHOENIX_WIDGET_HISTOGRAM_H_
