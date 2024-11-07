#include "widget_line_chart.h"

#include <iostream>
#include <string>
#include <cmath>

#include "GL/gl.h"
#include "GL/glu.h"


namespace phoenix {
namespace hmi {


/******************************************************************************/
/**  构造函数
 ******************************************************************************
 *  @param      parent           (IN)         指向父窗口
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       折线图界面的构造函数
 *
 *  <Attention>
 *       None
 *
 */
WidgetLineChart::WidgetLineChart(QWidget *parent)
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
  // 通道数量
  channel_number_ = 0;
  // 最大数据值
  data_max_value_ = 0.0;
  // 最小数据值
  data_min_value_ = 0.0;

  coordinate_axis_x_ = nullptr;
  param_ = nullptr;
  common::com_memset(string_buffer_, 0, sizeof(string_buffer_));
}

/******************************************************************************/
/**  析构函数
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       折线图界面的析构函数
 *
 *  <Attention>
 *       None
 *
 */
WidgetLineChart::~WidgetLineChart() {
  if (data_buff_.size() > 0) {
    for (int i = 0; i < data_buff_.size(); ++i) {
      delete data_buff_[i];
    }
    data_buff_.clear();
  }
  if (nullptr != coordinate_axis_x_) {
    delete []coordinate_axis_x_;
    coordinate_axis_x_ = nullptr;
  }
  if (nullptr != param_) {
    delete []param_;
    param_ = nullptr;
  }
}

/******************************************************************************/
/** 参数配置
 ******************************************************************************
 *  @param      min_            (IN)          数值范围(最小值)
 *              max_            (IN)          数值范围(最大值)
 *              count_          (IN)          最多显示数值的个数
 *              channel_        (IN)          通道数量
 *              pParam_         (IN)          显示参数
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       配置显示参数
 *
 *  <Attention>
 *       None
 *
 */
bool WidgetLineChart::Config(
    double min, double max, int channel, PARAM *param) {
  if (min > max) {
    return (false);
  }
  if (channel < 1) {
    return (false);
  }
  if (nullptr == param) {
    return (false);
  }

  if (data_buff_.size() > 0) {
    for (int i = 0; i < data_buff_.size(); ++i) {
      delete data_buff_[i];
    }
    data_buff_.clear();
  }
  if (nullptr != coordinate_axis_x_) {
    delete []coordinate_axis_x_;
    coordinate_axis_x_ = nullptr;
  }
  if (nullptr != param_) {
    delete []param_;
    param_ = nullptr;
  }
  for (int i = 0; i < channel; ++i) {
    data_buff_.push_back(new common::RingBuffer<double, MAX_DATA_NUM>());
  }
  coordinate_axis_x_ = new double[MAX_DATA_NUM];
  param_ = new PARAM[channel];

  channel_number_ = channel;
  max_data_number_ = MAX_DATA_NUM;
  data_max_value_ = max;
  data_min_value_ = min;
  for (int i = 0; i < channel; ++i) {
    param_[i] = param[i];
  }

  resizeGL(width(), height());

  return (true);
}

void WidgetLineChart::SetMinValue(double lower, double upper) {
  conf_min_value_lower_ = lower;
  conf_min_value_upper_ = upper;
}

/******************************************************************************/
/** 槽函数(数据更新了)
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       槽函数(数据更新了)
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::UpdateWindow() {
  updateGL();
}

/******************************************************************************/
/** 初始化GL窗口
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       初始化GL窗口
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::initializeGL() {
  //initializeOpenGLFunctions();
  makeCurrent();

  //qglClearColor(QColor(0, 0, 0, 0));
  //qglClearColor(QColor(50, 50, 50, 0));
  qglClearColor(QColor(210, 210, 210, 0));
  //glShadeModel(GL_FLAT);

  //开启抗锯齿
  //glEnable(GL_POINT_SMOOTH);
  //glEnable(GL_LINE_SMOOTH);

  //glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  //glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  //glEnable(GL_BLEND);
  //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  //glDisable(GL_DEPTH_TEST);

  //    glShadeModel(GL_SMOOTH);

  //    glClearDepth(1.0);
  //    glEnable(GL_DEPTH_TEST);
  //    glDepthFunc(GL_LEQUAL);
  //    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

  //    //GLuint filter = 0;
  //    GLuint fog_mode[] = {GL_EXP, GL_EXP2, GL_LINEAR };
  //    GLuint fog_filter = 0;
  //    GLfloat fog_color[4] = { 0.5, 0.5, 0.5, 1.0 };
  //    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
  //    glFogi(GL_FOG_MODE, fog_mode[fog_filter]);
  //    glFogfv(GL_FOG_COLOR, fog_color);
  //    glFogf(GL_FOG_DENSITY, 0.1f);
  //    glHint(GL_FOG_HINT, GL_DONT_CARE);
  //    glFogf(GL_FOG_START, 1.0f);
  //    glFogf(GL_FOG_END, 5.0f);
  //    glEnable(GL_FOG);
}

/******************************************************************************/
/** 窗口尺寸改变处理函数
 ******************************************************************************
 *  @param      w               (IN)          窗口宽度
 *              h               (IN)          窗口高度
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       窗口尺寸改变处理函数
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::resizeGL(int w, int h) {
  (void)w;
  (void)h;

  UpdateViewWindow();

  updateGL();
}

/******************************************************************************/
/** 绘制窗口
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       绘制窗口
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::paintGL() {
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

/******************************************************************************/
/** 鼠标滚轮处理函数
 ******************************************************************************
 *  @param      event           (IN)          滚轮事件
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       鼠标滚轮处理函数
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::wheelEvent(QWheelEvent *event) {
  //    double numDegrees = -event->delta() / 8.0;
  //    double numSteps = numDegrees / 15.0;
  //    scene_scaling_ *= std::pow(1.125, numSteps);
  //    updateGL();
}

/******************************************************************************/
/** 鼠标点击处理函数
 ******************************************************************************
 *  @param      event           (IN)          鼠标事件
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       鼠标点击处理函数
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::mousePressEvent(QMouseEvent *event) {
  //    if (event->button() == Qt::LeftButton) {
  //        mouse_last_position_ = event->pos();
  //    }
}

/******************************************************************************/
/** 鼠标移动处理函数
 ******************************************************************************
 *  @param      event           (IN)          鼠标事件
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       鼠标移动处理函数
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::mouseMoveEvent(QMouseEvent *event) {
  //    GLfloat dx = GLfloat(event->x() - mouse_last_position_.x());
  //    GLfloat dy = GLfloat(event->y() - mouse_last_position_.y());
  //    if (event->buttons() & Qt::LeftButton) {
  //        scene_move_dx_ -= dx * scene_screen_scale_;
  //        scene_move_dy_ += dy * scene_screen_scale_;
  //        updateGL();
  //    }
  //    mouse_last_position_ = event->pos();
}

/******************************************************************************/
/** 更新视口
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       更新视口
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::UpdateViewWindow() {
  GLdouble view_left = GetWindowWidthInterval();
  GLdouble view_bottom = GetWindowHeightInterval();
  GLdouble view_right = GetWindowWidthInterval() + GetWindowWidth();
  GLdouble view_top = GetWindowHeightInterval() + GetWindowHeight();

  bool find = false;
  double min_value = data_max_value_;
  double max_value = data_min_value_;
  for (int i = 0; i < channel_number_; ++i) {
    common::RingBuffer<double, MAX_DATA_NUM>::iterator its =
        data_buff_[i]->begin();
    common::RingBuffer<double, MAX_DATA_NUM>::iterator ite =
        data_buff_[i]->end();
    its = data_buff_[i]->begin();
    for (; its != ite; ++its) {
      if ((*its) < min_value) {
        find = true;
        min_value = (*its);
      }
      if ((*its) > max_value) {
        max_value = (*its);
      }
    }
  }

  if (!find) {
    min_value = data_min_value_;
    max_value = data_max_value_;
  } else {
    //if ((min_value < 0) && (max_value > 0)) {
    //  if (std::abs(min_value) < std::abs(max_value)) {
    //    min_value = -max_value;
    //  } else {
    //    max_value = -min_value;
    //  }
    //}
    if ((data_min_value_ < 0) && (data_max_value_ > 0)) {
      if (std::abs(min_value) < std::abs(max_value)) {
        min_value = -std::abs(max_value) - 0.05*std::abs(max_value);
        max_value = -min_value;
      } else {
        max_value = std::abs(min_value) + 0.05*std::abs(min_value);
        min_value = -max_value;
      }
    }
  }
  if (min_value > conf_min_value_lower_) {
    min_value = conf_min_value_lower_;
  }
  if (max_value < conf_min_value_upper_) {
    max_value = conf_min_value_upper_;
  }

  scale_division_axis_x_ =
      (static_cast<double>(GetWindowWidth()) /
       static_cast<double>(GetWindowHeight())) *
      (max_value - min_value) /
      max_data_number_;

  scene_area_.left = 0;
  scene_area_.right = max_data_number_ * scale_division_axis_x_;
  scene_area_.bottom = min_value;
  scene_area_.top = max_value;

  for (int i = 0; i < max_data_number_; ++i) {
    coordinate_axis_x_[i] = i * scale_division_axis_x_;
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
}

/******************************************************************************/
/** 绘制坐标轴
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       绘制坐标轴
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::DrawCoordinateAxis() {
  if (channel_number_ > 0) {
    qglColor(QColor(100, 100, 100));
    glLineWidth(1.0);
    glBegin(GL_LINES);
    {
      double mid_y = (scene_area_.bottom + scene_area_.top) / 2.0;
      glVertex2d(scene_area_.left, mid_y);
      glVertex2d(scene_area_.right, mid_y);
    }
    glEnd();

    qglColor(QColor(180, 180, 180));
    glBegin(GL_LINES);
    for (int i = 0; i < MAX_DATA_NUM; ++i) {
      //double mid_x = (scene_area_.left + scene_area_.right) / 2.0;
      glVertex2d(coordinate_axis_x_[i], scene_area_.bottom);
      glVertex2d(coordinate_axis_x_[i], scene_area_.top);
    }
    glEnd();
  }
}

/******************************************************************************/
/** 绘制坐标轴文本
 ******************************************************************************
 *  @param      None
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       绘制坐标轴文本
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::DrawYAxisCoordinateValue() {
  qglColor(QColor(128, 128, 128));

  QFont font("Arial");
  double mid_y = (scene_area_.bottom + scene_area_.top) / 2.0;
  char str_buff[64] = { 0 };
  double x = width() / 2;
  std::snprintf(str_buff, sizeof(str_buff)-1, "%0.2f", mid_y);
  renderText(x, height() / 2 + 2, 0, str_buff, font);
  std::snprintf(str_buff, sizeof(str_buff)-1, "%0.2f", scene_area_.bottom);
  renderText(x, 4, 0, str_buff, font);
  std::snprintf(str_buff, sizeof(str_buff)-1, "%0.2f", scene_area_.top);
  renderText(x, height()-14, 0, str_buff, font);
}

/******************************************************************************/
/** 绘制数据
 ******************************************************************************
 *  @param      pNewData_        (IN)         待绘制的数据
 *
 *  @retval     None
 *
 *  @version    001 2015.12.16   pengc         新规作成
 *
 *  <Function>
 *       绘制数据
 *
 *  <Attention>
 *       None
 *
 */
void WidgetLineChart::DrawDatas() {
  for (int i = 0; i < channel_number_; ++i) {
    common::RingBuffer<double, MAX_DATA_NUM>::iterator its =
        data_buff_[i]->begin();
    common::RingBuffer<double, MAX_DATA_NUM>::iterator ite =
        data_buff_[i]->end();
    qglColor(QColor(param_[i].color.R, param_[i].color.G, param_[i].color.B));
    glLineWidth(param_[i].line_width);
    glPointSize(param_[i].point_size);
    int index = 0;
    //glBegin(GL_POINTS);
    glBegin(GL_LINE_STRIP);
    index = 0;
    //its = data_buff_[i]->begin();
    for (; its != ite; ++its) {
      glVertex2d(coordinate_axis_x_[index++], *its);
    }
    glEnd();
  }
}

void WidgetLineChart::PushBack(double* new_data) {
  if (nullptr == new_data) {
    return;
  }

  for (int i = 0; i < channel_number_; ++i) {
    data_buff_[i]->PushBackOverride(new_data[i]);
  }
}


} // hmi
} // phoenix

