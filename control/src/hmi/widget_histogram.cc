//
#include "widget_histogram.h"

#include <iostream>
#include <string>
#include <cmath>

#include "GL/gl.h"
#include "GL/glu.h"


namespace phoenix {
namespace hmi {


WidgetHistogram::WidgetHistogram(QWidget *parent)
  : QGLWidget(parent) {
  // 地图相关变量初始化
  scene_screen_scale_ = 1.0;
  scene_scaling_ = 1.0;
  scene_move_dx_ = 0.0;
  scene_move_dy_ = 0.0;
  scene_area_.left = 0;
  scene_area_.bottom = 0;
  scene_area_.right = 0;
  scene_area_.top = 0;
  scene_window_.left = GetWindowWidthInterval();
  scene_window_.bottom = GetWindowHeightInterval();
  scene_window_.right = GetWindowWidth();
  scene_window_.top = GetWindowHeight();

  // X轴分度
  scale_division_axis_x_ = 0.0;
  // 数据个数
  max_data_number_ = 0;
  // 最大数据值
  data_max_value_ = 0.0;
  // 最小数据值
  data_min_value_ = 0.0;

  coordinate_axis_x_.clear();
  base_coor_y_ = 0.0;
  common::com_memset(string_buffer_, 0, sizeof(string_buffer_));
}

WidgetHistogram::~WidgetHistogram() {
}

bool WidgetHistogram::Config(
    double min, double max, int max_data_num, int param_num, Param* param) {
  if ((max-min) < 1e-6) {
    return (false);
  }
  if (max_data_num < 1) {
    return (false);
  }
  if (param_num < 1) {
    return (false);
  }
  if (nullptr == param) {
    return (false);
  }

  param_.clear();
  for (int i = 0; i < param_num; ++i) {
    param_.push_back(param[i]);
  }

  max_data_number_ = max_data_num;
  data_max_value_ = max;
  data_min_value_ = min;

  // 设置显示区域
  GLdouble view_left = GetWindowWidthInterval();
  GLdouble view_bottom = GetWindowHeightInterval();
  GLdouble view_right = GetWindowWidthInterval() + GetWindowWidth();
  GLdouble view_top = GetWindowHeightInterval() + GetWindowHeight();

  scale_division_axis_x_ =
      (static_cast<double>(GetWindowWidth()) /
       static_cast<double>(GetWindowHeight())) *
      (data_max_value_ - data_min_value_) /
      max_data_number_;

  scene_area_.left = 0;
  scene_area_.right = max_data_number_ * scale_division_axis_x_;
  scene_area_.bottom = data_min_value_;
  scene_area_.top = data_max_value_;

  for (int i = 0; i < max_data_number_; ++i) {
    coordinate_axis_x_.push_back(i * scale_division_axis_x_);
  }

  if ((scene_area_.top > 0.0) && (scene_area_.bottom < 0.0)) {
    base_coor_y_ = 0.0F;
  } else if ((scene_area_.top > 0.0) && (scene_area_.bottom > 0.0)) {
    base_coor_y_ = scene_area_.bottom;
  } else if ((scene_area_.top < 0.0) && (scene_area_.bottom < 0.0)) {
    base_coor_y_ = scene_area_.top;
  } else {
    base_coor_y_ = 0.0;
  }

  // 根据地图大小，控制显示区域
  if (scene_area_.isValid()) {
    view_left = scene_area_.left;
    view_right = scene_area_.right;
    view_bottom = scene_area_.bottom;
    view_top = scene_area_.top;
  }

  // 限制缩放比例
  // if (scene_scaling_ < 0.01) {
  //    scene_scaling_ = 0.01;
  // } else if (scene_scaling_ > 10) {
  //    scene_scaling_ = 10;
  // }

  // 计算显示区域
  scene_window_.left = view_left;
  scene_window_.bottom = view_bottom;
  scene_window_.right = view_right;
  scene_window_.top = view_top;

  // 计算地图显示比例尺
  scene_screen_scale_ = (scene_window_.right - scene_window_.left) / width();

  resizeGL(width(), height());

  return (true);
}

void WidgetHistogram::UpdateWindow() {
  updateGL();
}

void WidgetHistogram::initializeGL() {
  makeCurrent();

  qglClearColor(QColor(210, 210, 210, 0));
}

void WidgetHistogram::resizeGL(int w, int h) {
  (void)w;
  (void)h;

  UpdateViewWindow();

  updateGL();
}

void WidgetHistogram::paintGL() {
  makeCurrent();

  // 清除当前场景
  glClear(GL_COLOR_BUFFER_BIT);

  // 更新显示区域
  UpdateViewArea();
  // 更新视口
  UpdateViewWindow();

  // 定义视口
  glViewport(GetWindowWidthInterval(), GetWindowHeightInterval(),
      GetWindowWidth(), GetWindowHeight());
  // 投影变换
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // 定义正投影(裁剪区域)
  // gluOrtho2D(/*left*/, /*right*/, /*bottom*/, /*top*/);
  glOrtho(scene_window_.left, scene_window_.right,
      scene_window_.bottom, scene_window_.top, 0, 100);
  // 在执行模型或视图变换之前，必须以GL_MODELVIEW为参数调用glMatrixMode()函数
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // 绘制坐标轴
  DrawCoordinateAxis();
  // 绘制
  DrawSomething();

  // 定义视口
  glViewport(0, 0, width(), height());
  // 投影变换
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // 定义正投影(裁剪区域)
  // gluOrtho2D(/*left*/, /*right*/, /*bottom*/, /*top*/);
  glOrtho(0, width(), 0, height(), 0, 100);
  // 在执行模型或视图变换之前，必须以GL_MODELVIEW为参数调用glMatrixMode()函数
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // 绘制Y坐标轴的值
  DrawYAxisCoordinateValue();

  // 绘制一些文本信息
  DrawSomeText();
}

void WidgetHistogram::wheelEvent(QWheelEvent *event) {
  //    double numDegrees = -event->delta() / 8.0;
  //    double numSteps = numDegrees / 15.0;
  //    scene_scaling_ *= std::pow(1.125, numSteps);
  //    updateGL();
}

void WidgetHistogram::mousePressEvent(QMouseEvent *event) {
  //    if (event->button() == Qt::LeftButton) {
  //        mouse_last_position_ = event->pos();
  //    }
}

void WidgetHistogram::mouseMoveEvent(QMouseEvent *event) {
  //    GLfloat dx = GLfloat(event->x() - mouse_last_position_.x());
  //    GLfloat dy = GLfloat(event->y() - mouse_last_position_.y());
  //    if (event->buttons() & Qt::LeftButton) {
  //        scene_move_dx_ -= dx * scene_screen_scale_;
  //        scene_move_dy_ += dy * scene_screen_scale_;
  //        updateGL();
  //    }
  //    mouse_last_position_ = event->pos();
}

void WidgetHistogram::UpdateViewWindow() {
  //
}

void WidgetHistogram::DrawCoordinateAxis() {
  qglColor(QColor(100, 100, 100));
  glLineWidth(1.0);
  glBegin(GL_LINES);
  {
    glVertex2d(scene_area_.left, base_coor_y_);
    glVertex2d(scene_area_.right, base_coor_y_);
  }
  glEnd();

  qglColor(QColor(180, 180, 180));
  glBegin(GL_LINES);
  for (int i = 0; i < max_data_number_; ++i) {
    glVertex2d(coordinate_axis_x_[i], scene_area_.bottom);
    glVertex2d(coordinate_axis_x_[i], scene_area_.top);
  }
  glEnd();
}

void WidgetHistogram::DrawYAxisCoordinateValue() {
  qglColor(QColor(128, 128, 128));

  QFont font("Arial");
  //double mid_y = (scene_area_.bottom + scene_area_.top) / 2.0;
  char str_buff[64] = { 0 };
  double x = width() / 2;
  //std::snprintf(str_buff, sizeof(str_buff)-1, "%0.2f", mid_y);
  renderText(x, height() / 2 + 2, 0, str_buff, font);
  std::snprintf(str_buff, sizeof(str_buff)-1, "%0.2f", scene_area_.bottom);
  renderText(x, 4, 0, str_buff, font);
  std::snprintf(str_buff, sizeof(str_buff)-1, "%0.2f", scene_area_.top);
  renderText(x, height()-14, 0, str_buff, font);
}

void WidgetHistogram::DrawDatas(
    int data_num, const double* data, const int* param_idxs) {
  if (max_data_number_ < 1) {
    return;
  }

  int param_size = param_.size();
  for (int i = 0; i < data_num; ++i) {
    int param_idx = param_idxs[i];
    if (param_idx < param_size) {
      qglColor(QColor(param_[param_idx].color.R,
                      param_[param_idx].color.G,
                      param_[param_idx].color.B));
    }

    glBegin(GL_QUADS);
    {
      if (data[i] > (base_coor_y_+1e-8)) {
        glVertex3d(coordinate_axis_x_[i], base_coor_y_, 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, base_coor_y_, 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, data[i], 0.0);
        glVertex3d(coordinate_axis_x_[i], data[i], 0.0);
      } else if (data[i] < (base_coor_y_-1e-8)) {
        glVertex3d(coordinate_axis_x_[i], base_coor_y_, 0.0);
        glVertex3d(coordinate_axis_x_[i], data[i], 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, data[i], 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, base_coor_y_, 0.0);
      } else {
        // nothing to do
      }
    }
    glEnd();
  }
}

void WidgetHistogram::DrawDatas(
    int data_num, const float* data, const int* param_idxs) {
  if (max_data_number_ < 1) {
    return;
  }

  int param_size = param_.size();
  for (int i = 0; i < data_num; ++i) {
    int param_idx = param_idxs[i];
    if (param_idx < param_size) {
      qglColor(QColor(param_[param_idx].color.R,
                      param_[param_idx].color.G,
                      param_[param_idx].color.B));
    }

    glBegin(GL_QUADS);
    {
      if (data[i] > (base_coor_y_+1e-8)) {
        glVertex3d(coordinate_axis_x_[i], base_coor_y_, 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, base_coor_y_, 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, data[i], 0.0);
        glVertex3d(coordinate_axis_x_[i], data[i], 0.0);
      } else if (data[i] < (base_coor_y_-1e-8)) {
        glVertex3d(coordinate_axis_x_[i], base_coor_y_, 0.0);
        glVertex3d(coordinate_axis_x_[i], data[i], 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, data[i], 0.0);
        glVertex3d(coordinate_axis_x_[i]+scale_division_axis_x_, base_coor_y_, 0.0);
      } else {
        // nothing to do
      }
    }
    glEnd();
  }
}


} // hmi
} // phoenix

