/******************************************************************************
 ** 图形显示窗口
 ******************************************************************************
 *
 *  图形显示窗口
 *
 *  @file       widget_map.cc
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#include "widget_map.h"

#include <string>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"

#include "utils/com_utils.h"
#include "math/math_utils.h"
#include "curve/cubic_polynomial_curve1d.h"
#include "vehicle_model_wrapper.h"

#include "draw_concave_polygon_gl.h"


namespace phoenix {
namespace hmi {


WidgetMap::WidgetMap(QWidget* parent)
  : QGLWidget(parent) {
  view_param_.qu_rot.SetIdentity();
  view_param_.mat_rot.SetIdentity();
  view_param_.vec_move.SetZeros();
  view_param_.prev_mouse_pos.x = 0;
  view_param_.prev_mouse_pos.y = 0;
  view_param_.first_click_mouse_pos.x = 0;
  view_param_.first_click_mouse_pos.y = 0;
  view_param_.scaling = 1.0;

  SetDisplayOriginCoor(0, 0, 0);

  // Transform
#if 0
  view_param_.v_eye_center_pos += -10 * view_param_.v_eye_center_y;
#endif
  // Rotate
#if 1
  common::Quaternion<Float64_t> qu_rot1;
  qu_rot1.ConstructFromAngleAxis(common::com_deg2rad(90.0),
                                 common::Vector3d(0.0, 0.0, 1.0));
  common::Quaternion<Float64_t> qu_rot2;
  qu_rot2.ConstructFromAngleAxis(common::com_deg2rad(-76.0),
                                 common::Vector3d(1.0, 0.0, 0.0));
  view_param_.qu_rot = qu_rot2 * qu_rot1;
  view_param_.qu_rot.NormalizeInplace();
  view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);
#endif

  view_switch_button_.min().set_x(10);
  view_switch_button_.min().set_y(60);
  view_switch_button_.max().set_x(50);
  view_switch_button_.max().set_y(100);
  vew_mode_ = VIEW_MODE_2;

  ConstructPolarPlotPoints();

  fuzzy_level_string_.push_back("负大");
  fuzzy_level_string_.push_back("负中");
  fuzzy_level_string_.push_back("负小");
  fuzzy_level_string_.push_back("负零");
  fuzzy_level_string_.push_back("正零");
  fuzzy_level_string_.push_back("正小");
  fuzzy_level_string_.push_back("正中");
  fuzzy_level_string_.push_back("正大");
}

WidgetMap::~WidgetMap() {
}

void WidgetMap::LoadIcon() {
  texture_id_of_icon_aerial_view_ = 0;
  texture_id_of_icon_3d_view_ = 0;
  texture_id_of_icon_curve_ = 0;
  texture_id_of_icon_ramp_ = 0;
  texture_id_of_icon_passing_ramp_ = 0;
  texture_id_of_icon_tunnel_ = 0;

  // view icon
  QImage icon_aerial_view;
  bool ret = icon_aerial_view.load(":/images/icon_aerial_view.png");
  if (!ret) {
    return;
  }
  // icon_aerial_view = icon_aerial_view.scaled(icon_aerial_view.width(),
  //                                            icon_aerial_view.height());
  // std::cout << "### The size of aerial view icon is ("
  //           << icon_aerial_view.width()
  //           << ", " << icon_aerial_view.height()
  //           << "). ###" << std::endl;

  QImage icon_3d_view;
  ret = icon_3d_view.load(":/images/icon_3d_view.png");
  if (!ret) {
    return;
  }
  // icon_3d_view = icon_3d_view.scaled(icon_3d_view.width(),
  //                                    icon_3d_view.height());
  // std::cout << "### The size of 3d view icon is ("
  //           << icon_3d_view.width()
  //           << ", " << icon_3d_view.height()
  //           << "). ###" << std::endl;

  QImage icon_curve;
  ret = icon_curve.load(":/images/icon_curve.png");
  if (!ret) {
    return;
  }

  QImage icon_ramp;
  ret = icon_ramp.load(":/images/icon_ramp.png");
  if (!ret) {
    return;
  }

  QImage icon_passing_ramp;
  ret = icon_passing_ramp.load(":/images/icon_passing_ramp.png");
  if (!ret) {
    return;
  }

  QImage icon_tunnel;
  ret = icon_tunnel.load(":/images/icon_tunnel.png");
  if (!ret) {
    return;
  }


  // 生成纹理
  GLenum gl_err = 0;
  const Char_t* gl_err_str = Nullptr_t;

  // glGenTextures(GLsizei n, GLuint *textures)函数说明
  // n：用来生成纹理的数量
  // textures：存储纹理索引
  // glGenTextures函数根据纹理参数返回n个纹理索引。纹理名称集合不必是一个连续的整数集合。
  // glGenTextures就是用来产生你要操作的纹理对象的索引的，比如你告诉OpenGL，
  // 我需要5个纹理对象，它会从没有用到的整数里返回5个给你）
  // glBindTexture实际上是改变了OpenGL的这个状态，
  // 它告诉OpenGL下面对纹理的任何操作都是对它所绑定的纹理对象的，
  // 比如glBindTexture(GL_TEXTURE_2D,1)告诉OpenGL下面代码中对2D纹理的
  // 任何设置都是针对索引为1的纹理的。
  // 产生纹理函数假定目标纹理的面积是由glBindTexture函数限制的。
  // 先前调用glGenTextures产生的纹理索引集不会由后面调用的glGenTextures得到，
  // 除非他们首先被glDeleteTextures删除。你不可以在显示列表中包含glGenTextures。
  glGenTextures(1, &texture_id_of_icon_aerial_view_);
  if (texture_id_of_icon_aerial_view_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_aerial_view_);
    /* 设置纹理过滤方式 */
    // 纹理图象被使用到一个小于它的形状上，应该如何处理。
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    // 设置如果纹理图象被使用到一个大于它的形状上时，像素如何填充，
    // 可选择的设置有GL_NEAREST和GL_LINEAR，
    // 前者表示“使用纹理中坐标最接近的一个像素的颜色作为需要绘制的像素颜色”，
    // 后者表示“使用纹理中坐标最接近的若干个颜色，
    // 通过加权平均算法得到需要绘制的像素颜色",后者效果要比前者好
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // 指当纹理坐标的第一维坐标值大于1.0或小于0.0时，应该如何处理。
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    // 指当纹理坐标的第二维坐标值大于1.0或小于0.0时，应该如何处理。
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    // 指定二维纹理
    glTexImage2D(GL_TEXTURE_2D, 0,
                 4,
                 icon_aerial_view.width(), icon_aerial_view.height(),
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_aerial_view.bits());
  }
  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
  // std::cout << "### After generating texture for aerial view icon, texture_id="
  //           << texture_id_of_icon_aerial_view_
  //           << ", err_code=" << gl_err
  //           << ", and err_str=\"" << gl_err_str << "\"."
  //           << " ###" << std::endl;

  // 生成纹理
  glGenTextures(1, &texture_id_of_icon_3d_view_);
  if (texture_id_of_icon_3d_view_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_3d_view_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0,
                 4,
                 icon_3d_view.width(), icon_3d_view.height(),
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_3d_view.bits());
  }

  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
#if 0
  std::cout << "### After generating texture for 3d view icon, texture_id="
            << texture_id_of_icon_3d_view_
            << ", err_code=" << gl_err
            << ", and err_str=\"" << gl_err_str << "\"."
            << " ###" << std::endl;
#endif

  // 生成纹理
  glGenTextures(1, &texture_id_of_icon_curve_);
  if (texture_id_of_icon_curve_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_curve_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0,
                 4,
                 icon_curve.width(), icon_curve.height(),
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_curve.bits());
  }

  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
#if 1
  std::cout << "### After generating texture for curve icon, texture_id="
            << texture_id_of_icon_curve_
            << ", err_code=" << gl_err
            << ", and err_str=\"" << gl_err_str << "\"."
            << " ###" << std::endl;
#endif

  // 生成纹理
  glGenTextures(1, &texture_id_of_icon_ramp_);
  if (texture_id_of_icon_ramp_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_ramp_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0,
                 4,
                 icon_ramp.width(), icon_ramp.height(),
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_ramp.bits());
  }

  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
#if 1
  std::cout << "### After generating texture for ramp icon, texture_id="
            << texture_id_of_icon_ramp_
            << ", err_code=" << gl_err
            << ", and err_str=\"" << gl_err_str << "\"."
            << " ###" << std::endl;
#endif

  // 生成纹理
  glGenTextures(1, &texture_id_of_icon_passing_ramp_);
  if (texture_id_of_icon_passing_ramp_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_passing_ramp_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0,
                 4,
                 icon_passing_ramp.width(), icon_passing_ramp.height(),
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_passing_ramp.bits());
  }

  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
#if 1
  std::cout << "### After generating texture for passing ramp icon, texture_id="
            << texture_id_of_icon_passing_ramp_
            << ", err_code=" << gl_err
            << ", and err_str=\"" << gl_err_str << "\"."
            << " ###" << std::endl;
#endif

  // 生成纹理
  glGenTextures(1, &texture_id_of_icon_tunnel_);
  if (texture_id_of_icon_tunnel_ > 0) {
    glBindTexture(GL_TEXTURE_2D, texture_id_of_icon_tunnel_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexImage2D(GL_TEXTURE_2D, 0,
                 4,
                 icon_tunnel.width(), icon_tunnel.height(),
                 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE,
                 icon_tunnel.bits());
  }

  gl_err = glGetError();
  gl_err_str = (const Char_t*)gluErrorString(gl_err);
#if 1
  std::cout << "### After generating texture for tunnel icon, texture_id="
            << texture_id_of_icon_tunnel_
            << ", err_code=" << gl_err
            << ", and err_str=\"" << gl_err_str << "\"."
            << " ###" << std::endl;
#endif
}

void WidgetMap::ChangeView() {
  switch (vew_mode_) {
  case (VIEW_MODE_1):
    vew_mode_ = VIEW_MODE_2;
    break;

  case (VIEW_MODE_2):
    vew_mode_ = VIEW_MODE_1;
    break;

  default:
    vew_mode_ = VIEW_MODE_1;
    break;
  }

  if (VIEW_MODE_2 == vew_mode_) {
    view_param_.qu_rot.SetIdentity();
    view_param_.mat_rot.SetIdentity();
    view_param_.vec_move.SetZeros();
    view_param_.prev_mouse_pos.x = 0;
    view_param_.prev_mouse_pos.y = 0;
    view_param_.first_click_mouse_pos.x = 0;
    view_param_.first_click_mouse_pos.y = 0;
    view_param_.scaling = 1.0;

    // Rotate
    common::Quaternion<Float64_t> qu_rot1;
    qu_rot1.ConstructFromAngleAxis(common::com_deg2rad(90.0),
                                   common::Vector3d(0.0, 0.0, 1.0));
    common::Quaternion<Float64_t> qu_rot2;
    qu_rot2.ConstructFromAngleAxis(common::com_deg2rad(-76.0),
                                   common::Vector3d(1.0, 0.0, 0.0));
    view_param_.qu_rot = qu_rot2 * qu_rot1;
    view_param_.qu_rot.NormalizeInplace();
    view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);
  } else {
    view_param_.qu_rot.SetIdentity();
    view_param_.mat_rot.SetIdentity();
    view_param_.vec_move.SetZeros();
    view_param_.prev_mouse_pos.x = 0;
    view_param_.prev_mouse_pos.y = 0;
    view_param_.first_click_mouse_pos.x = 0;
    view_param_.first_click_mouse_pos.y = 0;
    view_param_.scaling = 1.0;

    // Rotate
    common::Quaternion<Float64_t> qu_rot1;
    qu_rot1.ConstructFromAngleAxis(common::com_deg2rad(90.0),
                                   common::Vector3d(0.0, 0.0, 1.0));
    view_param_.qu_rot = qu_rot1;
    view_param_.qu_rot.NormalizeInplace();
    view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);
  }
}

void WidgetMap::DrawViewSwitchButton() {
  GLuint texture_id = 0;
  switch (vew_mode_) {
  case (VIEW_MODE_1):
    texture_id = texture_id_of_icon_aerial_view_;
    // std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "1");
    break;

  case (VIEW_MODE_2):
    texture_id = texture_id_of_icon_3d_view_;
    // std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "2");
    break;

  default:
    texture_id = texture_id_of_icon_aerial_view_;
    // std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "1");
    break;
  }

#if 0
  glLineWidth(1.0);
  qglColor(QColor(150, 150, 150));

  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(view_switch_button_.min().x(),
        height() - view_switch_button_.min().y(), 0.0);
    glVertex3d(view_switch_button_.max().x(),
        height() - view_switch_button_.min().y(), 0.0);
    glVertex3d(view_switch_button_.max().x(),
        height() - view_switch_button_.max().y(), 0.0);
    glVertex3d(view_switch_button_.min().x(),
        height() - view_switch_button_.max().y(), 0.0);
  }
  glEnd();
#else
  DrawIcon(texture_id, view_switch_button_);
#endif

  // renderText(static_cast<int>(0.5*(view_switch_button_.min().x() +
  //                               view_switch_button_.max().x()))-5
  //            , static_cast<int>(0.5*(view_switch_button_.min().y() +
  //                                 view_switch_button_.max().y()))+5
  //            , string_buffer_, QFont("AnyStyle", 15));
}

void WidgetMap::wheelEvent(QWheelEvent *event)  {
  Float64_t num_degrees = -event->delta() / 8.0;
  Float64_t num_steps = num_degrees / 15.0;
  view_param_.scaling *= std::pow(1.125f, num_steps);
  if (view_param_.scaling < 0.0001) {
    view_param_.scaling = 0.0001;
  } else if (view_param_.scaling > 10000.0) {
    view_param_.scaling = 10000.0;
  }
  updateGL();
}

void WidgetMap::mousePressEvent(QMouseEvent *event) {
  if ((event->button() == Qt::LeftButton) ||
      (event->button() == Qt::RightButton)) {
    view_param_.prev_mouse_pos.x = event->pos().x();
    view_param_.prev_mouse_pos.y = event->pos().y();

    view_param_.first_click_mouse_pos.x = event->pos().x();
    view_param_.first_click_mouse_pos.y = event->pos().y();
  }

  if ((event->button() == Qt::LeftButton) &&
      (event->flags() | QEvent::MouseButtonPress)) {
    if (view_switch_button_.IsPointIn(event->pos().x(), event->pos().y())) {
      ChangeView();
    }
  }
}

void WidgetMap::mouseMoveEvent(QMouseEvent *event) {
  Float64_t dx = static_cast<Float64_t>(
        event->x() - view_param_.prev_mouse_pos.x) / width();
  Float64_t dy = -static_cast<Float64_t>(
        event->y() - view_param_.prev_mouse_pos.y) / height();

  if (event->buttons() & Qt::LeftButton) {
    // translation
    Float64_t scale = 40 * view_param_.scaling;
    view_param_.vec_move(0) += scale * dx;
    view_param_.vec_move(1) += scale * dy;
    updateGL();
  } else if (event->buttons() & Qt::RightButton) {
#if 0
    // rotation
    Float64_t theta = common::com_atan2(
          common::com_abs(dy), common::com_abs(dx));
    Float64_t rotation_angle = 2.0 * common::com_sqrt(dx*dx + dy*dy);
    if (dy >= 0 && dx >= 0) {
      // First phase limit
      rotation_angle = -rotation_angle;
    } else if (dy >= 0 && dx < 0) {
      // Second phase limit
      theta = -theta;
    } else if (dy < 0 && dx < 0) {
      // Third phase limit
    } else {
      // Fourth phase limit
      theta = -theta;
      rotation_angle = -rotation_angle;
    }
    theta = common::NormalizeAngle(theta-common::com_deg2rad(90.0));

    common::Vector3d rot_axis;
    rot_axis(0) = common::com_cos(theta);
    rot_axis(1) = common::com_sin(theta);
    rot_axis(2) = 0;
    common::Quaternion<Float64_t> qu_rot1;
    qu_rot1.ConstructFromAngleAxis(rotation_angle, rot_axis);
    view_param_.qu_rot = qu_rot1 * view_param_.qu_rot;
    view_param_.qu_rot.NormalizeInplace();
    view_param_.qu_rot.ConvertToRotationMatrix(&view_param_.mat_rot);

    updateGL();
#endif
  }
  view_param_.prev_mouse_pos.x = event->pos().x();
  view_param_.prev_mouse_pos.y = event->pos().y();
}

void WidgetMap::resizeGL(Int32_t w, Int32_t h) {
  (void)w;
  (void)h;
  updateGL();
}

void WidgetMap::initializeGL() {
  makeCurrent();

  //qglClearColor(QColor(50, 50, 50, 0));
  qglClearColor(QColor(210, 210, 210, 0));
  glShadeModel(GL_FLAT);

  // 开启抗锯齿
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glEnable(GL_DEPTH_TEST);

  LoadIcon();

#if 0
  // 开启深度Z缓存
  glEnable(GL_DEPTH_TEST);
  // 设置只有正面多边形进行光照计算
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  //启用光照
  glEnable(GL_LIGHTING);
  //1.0：光源位置；
  GLfloat ambient_light[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  //设置光照模型，提供环境光
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient_light);
  //启用颜色追踪
  glEnable(GL_COLOR_MATERIAL);
  // 设置正面的环境光和散射光属性以便追踪glColor所设置的颜色：
  // 材料的环境和散射反射属性就和glColor函数所设置的当前颜色相同
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
  // 镜面光成分：非常亮的白色光源
  GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f};
  // 为光源添加镜面光成分
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  // 镜面反射的RGBA，1：反射几乎所有的入射光
  GLfloat specref[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  // 为材料添加镜面反射成分
  glMaterialfv(GL_FRONT, GL_SPECULAR, specref);
  //0~128 设置镜面指数，指定了镜面加亮的大小和集中性。0：均匀加亮；值越大，加亮越明显
  glMateriali(GL_FRONT, GL_SHININESS, 128);
  // OpenGL支持至少8种独立光源，
  //GLfloat ambient_light_2[] = { 0.3f, 0.3f, 0.3f, 1.0f };
  // GLfloat diffuse_light[] = { 0.7f, 0.7f, 0.7f, 1.0f };
  //1.0：光源位置；0.0：光源在无限远处，沿向量指定方向照射过来
  GLfloat light_pos[] = { -50.f, 50.0f, 100.0f, 0.0f };
  //设置光照0
  //glLightfv(GL_LIGHT0, GL_DIFFUSE, ambient_light_2);
  //glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
  //指定位置和照射方向：(ChangeSize内)
  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
  //从光点发散出来的光锥的发散角度
  glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 50.0f);
  GLfloat spot_dir[] = { 0.0f, 0.0f, -1.0f };
  glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_dir);
#endif
#if 0
  glEnable(GL_CULL_FACE);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

  //启用颜色追踪
  glEnable(GL_COLOR_MATERIAL);
  // 设置正面的环境光和散射光属性以便追踪glColor所设置的颜色：
  // 材料的环境和散射反射属性就和glColor函数所设置的当前颜色相同
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  GLfloat ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
  GLfloat diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
  GLfloat specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };

  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

  GLfloat position[] = { 2.0f, 2.0f, 2.0f, 1.0f };
  glLightfv(GL_LIGHT0, GL_POSITION, position);

  GLfloat green[] = { 0.0f, 0.2f, 0.1f, 1.0f };
  GLfloat yellow[] = { 0.7f, 0.6f, 0.1f, 1.0f };
  GLfloat white[] = { 1.0f, 1.0f, 1.0f, 1.0f };
  GLfloat gray[] = { 0.2f, 0.2f, 0.2f, 1.0f };
  // 自发光
  //glMaterialfv(GL_FRONT, GL_EMISSION, green);
  // 环境光
  glMaterialfv(GL_FRONT, GL_AMBIENT, white);
  // 漫反射光
  glMaterialfv(GL_FRONT, GL_DIFFUSE , white);
  // 镜面反射光
  glMaterialfv(GL_FRONT, GL_SPECULAR, white);
  glMaterialf(GL_FRONT, GL_SHININESS, 10.0f);
#endif
}

// 绘制窗口
void WidgetMap::paintGL() {
  Int32_t w = width();
  Int32_t h = height();

  // Get module status
  UpdateModuleInfo();

  //SetDisplayOriginCoor(module_info_.filtered_pose.x,
  //    module_info_.filtered_pose.y, 0.0);

  makeCurrent();

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  Float64_t range = 0.2;
  if (w <= h) {
    Float64_t ratio = static_cast<Float64_t>(h) / static_cast<Float64_t>(w);
    Float64_t range_mul_ratio = range * ratio;
    // glOrtho(-range, range, -range_mul_ratio, range_mul_ratio, -100.0, 100.0);
    glFrustum(-range, range, -range_mul_ratio, range_mul_ratio, 1.0, 2000.0);
  } else {
    Float64_t ratio = static_cast<Float64_t>(w) / static_cast<Float64_t>(h);
    Float64_t range_mul_ratio = range * ratio;
    // glOrtho(-range_mul_ratio, range_mul_ratio, -range, range, -100.0, 100.0);
    glFrustum(-range_mul_ratio, range_mul_ratio, -range, range, 1.0, 2000.0);
  }

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  GLdouble mat_modelview[16];
  mat_modelview[0]  = view_param_.mat_rot(0, 0);
  mat_modelview[1]  = view_param_.mat_rot(1, 0);
  mat_modelview[2]  = view_param_.mat_rot(2, 0);
  mat_modelview[3]  = 0.0;
  mat_modelview[4]  = view_param_.mat_rot(0, 1);
  mat_modelview[5]  = view_param_.mat_rot(1, 1);
  mat_modelview[6]  = view_param_.mat_rot(2, 1);
  mat_modelview[7]  = 0.0;
  mat_modelview[8]  = view_param_.mat_rot(0, 2);
  mat_modelview[9]  = view_param_.mat_rot(1, 2);
  mat_modelview[10] = view_param_.mat_rot(2, 2);
  mat_modelview[11] = 0.0;
  mat_modelview[12] = view_param_.vec_move(0);
  mat_modelview[13] = view_param_.vec_move(1);
  mat_modelview[14] = -100.0* view_param_.scaling;
  mat_modelview[15] = 1.0;
  glLoadMatrixd(mat_modelview);

  // Draw map
  DrawMap();
  // Draw vehicle
  DrawVehicle();

  // 绘制网格
  phoenix::common::AABBox2d grid_range;
  grid_range.min().set_x(-100);
  grid_range.min().set_y(-100);
  grid_range.max().set_x(100);
  grid_range.max().set_y(100);
  DrawGrid(grid_range, 1.0);
  //DrawPolarPlot();

  // 绘制坐标
  DrawCoordinateAxis(0.0, 0.0, 0.0, 1.0);

  // 视图变换
  // 定义视口
  glViewport(0, 0, width(), height());
  // 投影变换
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // 定义正投裁剪区域)
  // gluOrtho2D(/*left*/, /*right*/, /*bottom*/, /*top*/);
  gluOrtho2D(0, w, 0, h);
  // glOrtho(0, w, 0, h, 0, 100);
  // 在执行模型或视图变换之前，必须以GL_MODELVIEW为参数调用glMatrixMode()函数
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  DrawViewSwitchButton();
  DrawSpeedRestrictionMark();
  DrawTrafficLight();
  DrawVelocityPedal();
  DrawAccelerationPedal();
  DrawModuleStatusInfo();
  DrawChassisInfo();
  DrawSteeringWheelAnglePedal();
  DrawLongitudinalCtlValuePedal();

  // Scene storys
  DrawSceneStory();
}

void WidgetMap::DrawCoordinateAxis(
    Float64_t x, Float64_t y, Float64_t z, Float64_t len) {
  glLineWidth(3.0f);
  glBegin(GL_LINES);
  {
    glColor3d(1.0, 0.0, 0.0);
    glVertex3d(x, y, z);
    glVertex3d(x+len, y, z);

    glColor3d(0.0, 1.0, 0.0);
    glVertex3d(x, y, z);
    glVertex3d(x, y+len, z);

    glColor3d(0.0, 0.0, 1.0);
    glVertex3d(x, y, z);
    glVertex3d(x, y, z+len);
  }
  glEnd();
  glColor3d(1.0, 0.0, 0.0);
  renderText(x+len, y, z, "x");
  glColor3d(0.0, 1.0, 0.0);
  renderText(x, y+len, z, "y");
  glColor3d(0.0, 0.0, 1.0);
  renderText(x, y, z+len, "z");


  qglColor(QColor(0, 0, 255, 100));
  glLineWidth(1.0f);
  glBegin(GL_LINES);
  {
    glVertex3d(0.0, -10.0, 0.2);
    glVertex3d(0.0, 10.0, 0.2);

    glVertex3d(15.0, -12.0, 0.2);
    glVertex3d(15.0, 12.0, 0.2);

    glVertex3d(30.0, -15.0, 0.2);
    glVertex3d(30.0, 15.0, 0.2);

    glVertex3d(50.0, -20.0, 0.2);
    glVertex3d(50.0, 20.0, 0.2);

    glVertex3d(80.0, -25.0, 0.2);
    glVertex3d(80.0, 25.0, 0.2);

    glVertex3d(100.0, -30.0, 0.2);
    glVertex3d(100.0, 30.0, 0.2);
  }
  glEnd();
  renderText(0.0, 10.0, 0.2, "0m");
  renderText(15.0, 10.0, 0.2, "15m");
  renderText(30.0, 10.0, 0.2, "30m");
  renderText(50.0, 10.0, 0.2, "50m");
  renderText(80.0, 10.0, 0.2, "80m");
  renderText(100.0, 10.0, 0.2, "100m");

  qglColor(QColor(0, 0, 255, 100));
  glLineWidth(1.0f);
  glBegin(GL_LINES);
  {
    Float64_t ang = common::com_deg2rad(10.0);
    Float64_t len = 20.0;
    glVertex3d(0.0, 0.0, 0.2);
    glVertex3d(len*common::com_cos(ang), len*common::com_sin(ang), 0.2);

    ang = common::com_deg2rad(-10.0);
    len = 20.0;
    glVertex3d(0.0, 0.0, 0.2);
    glVertex3d(len*common::com_cos(ang), len*common::com_sin(ang), 0.2);
  }
  glEnd();
}

void WidgetMap::DrawGrid(
    const phoenix::common::AABBox2d& range, Float64_t interval) {
  glLineWidth(1.0);
  //glColor3d(0.3, 0.3, 0.3);
  qglColor(QColor(160, 160, 160, 100));

  Float64_t tmp = range.min().y();
  glBegin(GL_LINES);
  while (tmp < range.max().y()) {
    glVertex3d(range.min().x(), tmp, 0.0);
    glVertex3d(range.max().x(), tmp, 0.0);
    tmp += interval;
  }
  glVertex3d(range.min().x(), tmp, 0.0);
  glVertex3d(range.max().x(), tmp, 0.0);
  glEnd();

  tmp = range.min().x();
  glBegin(GL_LINES);
  while (tmp < range.max().x()) {
    glVertex3d(tmp, range.min().y(), 0.0);
    glVertex3d(tmp, range.max().y(), 0.0);
    tmp += interval;
  }
  glVertex3d(tmp, range.min().y(), 0.0);
  glVertex3d(tmp, range.max().y(), 0.0);
  glEnd();
}

void WidgetMap::ConstructPolarPlotPoints() {
  Int32_t radius_sample_num = 50;
  Float64_t radius_sample_interval = 2.0;

  Int32_t angle_sample_num = 180;
  Float64_t angle_sample_interval = common::com_deg2rad(2.0);

  polar_plot_points.clear();

  for (Int32_t i = 0; i < radius_sample_num; ++i) {
    Float64_t radius = (i+1) * radius_sample_interval;

    std::vector<common::Vec2d> points;
    for (Int32_t j = 0; j < angle_sample_num; ++j) {
      Float64_t angle = j * angle_sample_interval;

      common::Vec2d p;
      p.set_x(radius * common::com_cos(common::NormalizeAngle(angle)));
      p.set_y(radius * common::com_sin(common::NormalizeAngle(angle)));

      points.push_back(p);
    }

    polar_plot_points.push_back(points);
  }
}

void WidgetMap::DrawPolarPlot() {
  glLineWidth(1.0);
  qglColor(QColor(160, 160, 160, 100));

  Int32_t radius_sample_num = polar_plot_points.size();
  Int32_t angle_sample_num = 0;
  if (!polar_plot_points.empty()) {
    angle_sample_num = polar_plot_points.front().size();
  }

  for (Int32_t i = 0; i < radius_sample_num; ++i) {
    const std::vector<common::Vec2d>& points = polar_plot_points[i];

    glBegin(GL_LINE_LOOP);
    for (Int32_t j = 0; j < angle_sample_num; ++j) {
      const common::Vec2d& p = points[j];
      DrawVertex(p.x(), p.y(), 0.0);
    }
    glEnd();
  }

  if (!polar_plot_points.empty()) {
    const std::vector<common::Vec2d>& end_points = polar_plot_points.back();

    glBegin(GL_LINES);
    for (Int32_t i = 0; i < angle_sample_num; ++i) {
      const common::Vec2d& end_p = end_points[i];

      DrawVertex(0.0, 0.0, 0.0);
      DrawVertex(end_p.x(), end_p.y(), 0.0);
    }
    glEnd();
  }
}

// 画箭
void WidgetMap::DrawArrow(Float64_t x, Float64_t y,
    Float64_t heading, Float64_t len, Float64_t height) {
  phoenix::common::Vec2d end_p;
  phoenix::common::Vec2d arrow_p1;
  phoenix::common::Vec2d arrow_p2;
  end_p(0) = x + len*common::com_cos(heading);
  end_p(1) = y + len*common::com_sin(heading);
  arrow_p1(0) = end_p(0) +
      0.5f*common::com_cos(phoenix::common::NormalizeAngle(heading+COM_PI*0.8));
  arrow_p1(1) = end_p(1) +
      0.5f*common::com_sin(phoenix::common::NormalizeAngle(heading+COM_PI*0.8));
  arrow_p2(0) = end_p(0) +
      0.5f*common::com_cos(phoenix::common::NormalizeAngle(heading-COM_PI*0.8));
  arrow_p2(1) = end_p(1) +
      0.5f*common::com_sin(phoenix::common::NormalizeAngle(heading-COM_PI*0.8));

  glBegin(GL_LINES);
  {
    DrawVertex(x, y, height);
    DrawVertex(end_p(0), end_p(1), height);

    DrawVertex(end_p(0), end_p(1), height);
    DrawVertex(arrow_p1(0), arrow_p1(1), height);

    DrawVertex(end_p(0), end_p(1), height);
    DrawVertex(arrow_p2(0), arrow_p2(1), height);
  }
  glEnd();
}

void WidgetMap::DrawAABB_2D(
    const phoenix::common::AABBox2d& aabb, Float64_t height) {
  phoenix::common::Vec2d corner[4];
  corner[0] = aabb.min();
  corner[1].set_x(aabb.max().x());
  corner[1].set_y(aabb.min().y());
  corner[2] = aabb.max();
  corner[3].set_x(aabb.min().x());
  corner[3].set_y(aabb.max().y());

  if (common::com_abs(height) < 0.1) {
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
  } else {
#if 1
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();

    glBegin(GL_LINES);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);

      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();
#else
    glBegin(GL_QUADS);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);

      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), 0.0);

      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
#endif
  }
}

void WidgetMap::DrawOBB_2D(
    const phoenix::common::OBBox2d& obb, Float64_t height) {
  phoenix::common::Vec2d corner[4];
  corner[0] = obb.center() + obb.extents().x() * obb.unit_direction_x() +
      obb.extents().y() * obb.unit_direction_y();
  corner[1] = obb.center() - obb.extents().x() * obb.unit_direction_x() +
      obb.extents().y() * obb.unit_direction_y();
  corner[2] = obb.center() - obb.extents().x() * obb.unit_direction_x() -
      obb.extents().y() * obb.unit_direction_y();
  corner[3] = obb.center() + obb.extents().x() * obb.unit_direction_x() -
      obb.extents().y() * obb.unit_direction_y();

  if (common::com_abs(height) < 0.1) {
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
  } else {
#if 1
    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    {
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();

    glBegin(GL_LINES);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);

      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), height);
    }
    glEnd();
#else
    glBegin(GL_QUADS);
    {
      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[0](0), corner[0](1), height);

      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[3](0), corner[3](1), 0.0);

      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[2](0), corner[2](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);

      DrawVertex(corner[3](0), corner[3](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), 0.0);

      DrawVertex(corner[1](0), corner[1](1), height);
      DrawVertex(corner[1](0), corner[1](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), 0.0);
      DrawVertex(corner[2](0), corner[2](1), height);

      DrawVertex(corner[0](0), corner[0](1), 0.0);
      DrawVertex(corner[0](0), corner[0](1), height);
      DrawVertex(corner[3](0), corner[3](1), height);
      DrawVertex(corner[3](0), corner[3](1), 0.0);
    }
    glEnd();
#endif
  }
}

void WidgetMap::Update() {
  updateGL();
}

void WidgetMap::DrawBarrier(
    Float64_t x, Float64_t y, Float64_t heading,
    Float64_t width, Float64_t height) {
  Float64_t cos_heading = common::com_cos(heading);
  Float64_t sin_heading = common::com_sin(heading);

  phoenix::common::Vec2d certer_pt(x, y);
  phoenix::common::Vec2d direction_vec(sin_heading, -cos_heading);
  phoenix::common::Vec2d left_pt = certer_pt + 0.5*width*direction_vec;
  phoenix::common::Vec2d right_pt = certer_pt - 0.5*width*direction_vec;

  glLineWidth(2.0);
  glBegin(GL_LINES);
  {
    DrawVertex(left_pt.x(), left_pt.y(), 0.0);
    DrawVertex(left_pt.x(), left_pt.y(), height);
    DrawVertex(right_pt.x(), right_pt.y(), 0.0);
    DrawVertex(right_pt.x(), right_pt.y(), height);

    DrawVertex(left_pt.x(), left_pt.y(), 0.0);
    DrawVertex(right_pt.x(), right_pt.y(), 0.0);

    DrawVertex(left_pt.x(), left_pt.y(), height);
    DrawVertex(right_pt.x(), right_pt.y(), height);

    DrawVertex(left_pt.x(), left_pt.y(), 0.0);
    DrawVertex(right_pt.x(), right_pt.y(), height);

    DrawVertex(left_pt.x(), left_pt.y(), height);
    DrawVertex(right_pt.x(), right_pt.y(), 0.0);
  }
  glEnd();
}

void WidgetMap::DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
    Float64_t heading, Float64_t z, Int32_t count) {
  Float64_t delta_theta = 2.0*COM_PI / count;

  Float64_t cos_heading = common::com_cos(heading);
  Float64_t sin_heading = common::com_sin(heading);

  glBegin(GL_LINE_LOOP);
  for (Int32_t i = 0; i < count; ++i) {
    Float64_t theta = i * delta_theta;
    Float64_t px = a * common::com_cos(theta);
    Float64_t py = b * common::com_sin(theta);

    Float64_t ppx = x + px * cos_heading - py * sin_heading;
    Float64_t ppy = y + px * sin_heading + py * cos_heading;

    DrawVertex(ppx, ppy, z);
  }
  glEnd();
}

void WidgetMap::DrawIcon(GLuint texture_id, const common::AABBox2d& area) {
  // 激活二维纹理贴图
  glEnable(GL_TEXTURE_2D);
  // 设置纹理贴图方式(直接覆盖原来颜色或与原来颜色混合)
  // 直接使用纹理覆盖模式
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  // 指定当前使用的纹理(第一次使用和第二次使用含义不同)
  glBindTexture(GL_TEXTURE_2D, texture_id);

  //清除背景颜色干扰
  glColor3ub(255,255,255);
  // 为每个顶点指定纹理坐标，类似指定法线一样glNormal()
  glBegin(GL_QUADS);
  {
    glTexCoord2d(0.0, 0.0);
    glVertex2d(area.min().x(), height() - area.min().y());

    glTexCoord2d(1.0, 0.0);
    glVertex2d(area.max().x(), height() - area.min().y());

    glTexCoord2d(1.0, 1.0);
    glVertex2d(area.max().x(), height() - area.max().y());

    glTexCoord2d(0.0, 1.0);
    glVertex2d(area.min().x(), height() - area.max().y());
  }
  glEnd();
  // 关闭二维纹理贴图
  glDisable(GL_TEXTURE_2D);
}

void WidgetMap::DrawVehicle() {
#if 0
  Float64_t heading = module_info_.filtered_pose.heading;
  phoenix::common::Vec2d pos(
      module_info_.filtered_pose.x, module_info_.filtered_pose.y);
  phoenix::common::OBB_2D obb;

  const Float64_t half_veh_width =
      module_info_.motion_plan_config.vehicle_width * 0.5;
  const Float64_t half_veh_length =
      module_info_.motion_plan_config.vehicle_length * 0.5;
#else
  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  veh_model::VehicleModelWrapper veh_model;
  Float32_t vehicle_width = veh_model.GetVehicleWidth();
  if (ad_msg::VEH_TRAILER_STATUS_CONNECTED ==
      module_info_.chassis_info.trailer_status) {
    vehicle_width =
        common::Max(veh_model.GetVehicleWidth(), veh_model.GetTrailerWidth());
  }
  const Float64_t half_veh_width = 0.5F * vehicle_width;
  const Float64_t half_veh_length = 0.5F * veh_model.GetVehicleLength();
  const Float64_t veh_height = 2.0;
  const Float64_t dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();
#endif

  obb.set_unit_direction(phoenix::common::CosLookUp(heading),
      phoenix::common::SinLookUp(heading));
  obb.set_center(pos + dist_of_localization_to_center * obb.unit_direction_x());
  obb.set_extents(half_veh_length, half_veh_width);

  glLineWidth(4.0);

  // DrawOBB_2D(obb);
  phoenix::common::Vec2d corner[4];
  corner[0] = obb.center() + obb.extents().x() * obb.unit_direction_x() +
      obb.extents().y() * obb.unit_direction_y();
  corner[1] = obb.center() - obb.extents().x() * obb.unit_direction_x() +
      obb.extents().y() * obb.unit_direction_y();
  corner[2] = obb.center() - obb.extents().x() * obb.unit_direction_x() -
      obb.extents().y() * obb.unit_direction_y();
  corner[3] = obb.center() + obb.extents().x() * obb.unit_direction_x() -
      obb.extents().y() * obb.unit_direction_y();

  glColor3d(1.0, 1.0, 0.3);
  glBegin(GL_LINE_LOOP);
  {
    DrawVertex(corner[0](0), corner[0](1), 0.0);
    DrawVertex(corner[1](0), corner[1](1), 0.0);
    DrawVertex(corner[2](0), corner[2](1), 0.0);
    DrawVertex(corner[3](0), corner[3](1), 0.0);
  }
  glEnd();

  glColor3d(1.0, 0.5, 0.3);
  glBegin(GL_LINE_LOOP);
  {
    DrawVertex(corner[0](0), corner[0](1), veh_height);
    DrawVertex(corner[1](0), corner[1](1), veh_height);
    DrawVertex(corner[2](0), corner[2](1), veh_height);
    DrawVertex(corner[3](0), corner[3](1), veh_height);
  }
  glEnd();

  // glColor3d(0.0, 0.5, 1.0);
  glColor3d(0.2, 0.6, 1.0);
  glBegin(GL_LINES);
  {
    DrawVertex(corner[0](0), corner[0](1), 0.0);
    DrawVertex(corner[0](0), corner[0](1), veh_height);

    DrawVertex(corner[1](0), corner[1](1), 0.0);
    DrawVertex(corner[1](0), corner[1](1), veh_height);

    DrawVertex(corner[2](0), corner[2](1), 0.0);
    DrawVertex(corner[2](0), corner[2](1), veh_height);

    DrawVertex(corner[3](0), corner[3](1), 0.0);
    DrawVertex(corner[3](0), corner[3](1), veh_height);
  }
  glEnd();

  glColor3d(1.0, 1.0, 0.3);
  DrawEllipse(pos.x(), pos.y(), 0.5, 0.5, heading, 0.0, 3);
}

void WidgetMap::UpdateModuleInfo() {
  framework::SharedData* shared_data = framework::SharedData::instance();

  shared_data->GetModuleStatusList(&module_info_.module_status_list);

  shared_data->GetGnss(&module_info_.gnss);
  shared_data->GetFilteredGnssInfo(&module_info_.filtered_gnss_info);
  shared_data->GetImu(&module_info_.imu);
  shared_data->GetChassis(&module_info_.chassis_info);
  shared_data->GetChassisCtlCmd(&module_info_.chassis_ctl_cmd_info);
  shared_data->GetRelativePosList(&module_info_.relative_pos_list);
  shared_data->GetGnssTrackList(&module_info_.gnss_track_list);
  shared_data->GetGnssFilteredTrackList(&module_info_.gnss_filtered_track_list);
  shared_data->GetUtmTrackList(&module_info_.utm_track_list);
  shared_data->GetUtmFilteredTrackList(&module_info_.utm_filtered_track_list);
  shared_data->GetOdomTrackList(&module_info_.odom_track_list);
  shared_data->GetOdomFilteredTrackList(&module_info_.odom_filtered_track_list);
  shared_data->GetLaneMarkCameraList(&module_info_.lane_mark_camera_list);
  shared_data->GetLaneInfoCameraList(&module_info_.lane_info_camera_list);
  shared_data->GetObstacleCameraList(&module_info_.obstacle_camera_list);
  shared_data->GetObstacleRadarFrontList(&module_info_.obstacle_radar_front_list);
  shared_data->GetObstacleRadarRearList(&module_info_.obstacle_radar_rear_list);
  shared_data->GetObstacleRadarFrontLeftList(&module_info_.obstacle_radar_front_left_list);
  shared_data->GetObstacleRadarFrontRightList(&module_info_.obstacle_radar_front_right_list);
  shared_data->GetObstacleRadarRearLeftList(&module_info_.obstacle_radar_rear_left_list);
  shared_data->GetObstacleRadarRearRightList(&module_info_.obstacle_radar_rear_right_list);
  shared_data->GetSrr2DetectionsFrontLeftList(&module_info_.srr2_detections_front_left_list);
  shared_data->GetSrr2DetectionsFrontRightList(&module_info_.srr2_detections_front_right_list);
  shared_data->GetSrr2DetectionsRearLeftList(&module_info_.srr2_detections_rear_left_list);
  shared_data->GetSrr2DetectionsRearRightList(&module_info_.srr2_detections_rear_right_list);
  shared_data->GetObstacleLidarList0(&module_info_.obstacle_lidar_list_0);
  shared_data->GetObstacleLidarList1(&module_info_.obstacle_lidar_list_1);
  shared_data->GetObstacleTrackedInfoList(
        &module_info_.obstacle_tracked_info_list);
  shared_data->GetObstacleList(&module_info_.obstacle_list);

  shared_data->GetDrivingMapInfo(&module_info_.driving_map_info);
  shared_data->GetTrajectoryPlanningInfo(
        &module_info_.trajectory_planning_info);

  shared_data->GetActionPlanningResult(
        &module_info_.action_planning_result);
  shared_data->GetTrajectoryPlanningResult(
        &module_info_.trajectory_planning_result);
  shared_data->GetVelocityPlanningResult(
        &module_info_.velocity_planning_result);

  shared_data->GetTrafficSignalList(&module_info_.traffic_signal_list);
  shared_data->GetTrafficLightList(&module_info_.traffic_light_list);
}

void WidgetMap::DrawMap() {
  DrawLaneMarkCameraList();
  DrawLaneInfoCameraList();

  DrawDrivingMap();

  DrawTrajectoryPlanningInfo();
  DrawVelocityPlanningInfo();

  DrawRelativePosList();

  DrawObstacleCameraList();
  DrawObstacleRadarFrontList();
  DrawObstacleRadarRearList();
  DrawObstacleRadarFrontLeftList();
  DrawObstacleRadarFrontRightList();
  DrawObstacleRadarRearLeftList();
  DrawObstacleRadarRearRightList();
  DrawObstacleLidarList0();
  DrawObstacleLidarList1();
  DrawObstacleTrackedInfoList();
  DrawObstacleList();

  DrawSrr2DetectionsFrontLeftList();
  DrawSrr2DetectionsFrontRightList();
  DrawSrr2DetectionsRearLeftList();
  DrawSrr2DetectionsRearRightList();
}

void WidgetMap::DrawRelativePosList() {
  const ad_msg::RelativePosList& pos_list = module_info_.relative_pos_list;
  glColor3d(0.0, 0.0, 1.0);
  glLineWidth(1.0f);

#if 1
  for (Int32_t i = 0; i < pos_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = pos_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.3);
  }
#endif

#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
  // Show utm track list
  const ad_msg::RelativePosList& utm_track_list = module_info_.utm_track_list;
  glColor3d(0.0, 1.0, 1.0);
  for (Int32_t i = 0; i < utm_track_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = utm_track_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.4);
  }

  const ad_msg::RelativePosList& utm_filtered_track_list =
      module_info_.utm_filtered_track_list;
  glColor3d(1.0, 1.0, 0.0);
  for (Int32_t i = 0; i < utm_filtered_track_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = utm_filtered_track_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.5);
  }

  if (utm_track_list.relative_pos_num > 0) {
    glColor3d(1.0, 0.0, 0.0);
    const ad_msg::RelativePos& pos =
        utm_track_list.relative_pos[utm_track_list.relative_pos_num-1];
    DrawEllipse(pos.x, pos.y, 1.0, 1.0, pos.heading, 0.4, 3);
  }
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
  // Show odom track list
  const ad_msg::RelativePosList& odom_track_list = module_info_.odom_track_list;
  glColor3d(0.0, 1.0, 0.0);
  for (Int32_t i = 0; i < odom_track_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = odom_track_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.4);
  }

  const ad_msg::RelativePosList& odom_filtered_track_list =
      module_info_.odom_filtered_track_list;
  glColor3d(1.0, 0.0, 0.0);
  for (Int32_t i = 0; i < odom_filtered_track_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = odom_filtered_track_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.5);
  }

  if (odom_track_list.relative_pos_num > 0) {
    glColor3d(1.0, 0.0, 0.0);
    const ad_msg::RelativePos& pos =
        odom_track_list.relative_pos[odom_track_list.relative_pos_num-1];
    DrawEllipse(pos.x, pos.y, 1.0, 1.0, pos.heading, 0.4, 3);
  }
#else
  // Show gnss track list
  const ad_msg::RelativePosList& gnss_track_list = module_info_.gnss_track_list;
  glColor3d(0.0, 1.0, 1.0);
  for (Int32_t i = 0; i < gnss_track_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = gnss_track_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.4);
  }

  const ad_msg::RelativePosList& gnss_filtered_track_list =
      module_info_.gnss_filtered_track_list;
  glColor3d(1.0, 1.0, 0.0);
  for (Int32_t i = 0; i < gnss_filtered_track_list.relative_pos_num; ++i) {
    const ad_msg::RelativePos& pos = gnss_filtered_track_list.relative_pos[i];

    DrawArrow(pos.x, pos.y, pos.heading, 0.5, 0.5);
  }

  if (gnss_track_list.relative_pos_num > 0) {
    glColor3d(1.0, 0.0, 0.0);
    const ad_msg::RelativePos& pos =
        gnss_track_list.relative_pos[gnss_track_list.relative_pos_num-1];
    DrawEllipse(pos.x, pos.y, 1.0, 1.0, pos.heading, 0.4, 3);
  }
#endif
}

void WidgetMap::DrawLaneMarkCameraList() {
  const ad_msg::LaneMarkCameraList& lane_mark_list =
      module_info_.lane_mark_camera_list;
  const Float32_t sample_step_len = 1.0F;

  glColor3d(1.0, 1.0, 1.0);
  glLineWidth(3.0f);

  Int32_t lane_mark_num = lane_mark_list.lane_mark_num;
  for (Int32_t i = 0; i < lane_mark_num; ++i) {
    const ad_msg::LaneMarkCamera& lane_mark =
        lane_mark_list.lane_marks[i];

    if (lane_mark.quality < 2) {
      continue;
    }

    Float32_t lane_mark_len =
        lane_mark.view_range_end - lane_mark.view_range_start;
    if (lane_mark_len < sample_step_len) {
      continue;
    }

    Int32_t sample_size = common::com_round(lane_mark_len / sample_step_len);
    common::CubicPolynomialCurve1d<Float64_t> curve;
    curve.SetCoefficient(lane_mark.c0, lane_mark.c1,
                         lane_mark.c2, lane_mark.c3);

    if ((1 == lane_mark.id) || (-1 == lane_mark.id)) {
      qglColor(QColor(0, 255, 0));
    } else {
      qglColor(QColor(220, 220, 220));
    }
    if (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_SOLID !=
        lane_mark.lane_mark_type) {
      // glLineStipple(2, 0x030F);
      glLineStipple(3, 0x00FF);
      glEnable(GL_LINE_STIPPLE);
    }
    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < sample_size; ++j) {
      Float32_t x = j * sample_step_len;
      Float32_t y = static_cast<Float32_t>(curve.Evaluate(0, x));

      DrawVertex(x, y, 0.1);
    }
    glEnd();
    if (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_SOLID !=
        lane_mark.lane_mark_type) {
      glDisable(GL_LINE_STIPPLE);
    }
  }
}

void WidgetMap::DrawLaneInfoCameraList() {
  const ad_msg::LaneInfoCameraList& lane_info_list =
      module_info_.lane_info_camera_list;

  glColor3d(1.0, 1.0, 0.0);
  glLineWidth(1.0f);

  Int32_t lane_info_num = lane_info_list.boundary_line_num;
  for (Int32_t i = 0; i < lane_info_num; ++i) {
    const ad_msg::LaneBoundaryLineCamera& boundary_line =
        lane_info_list.boundary_lines[i];
#if 0
    Int32_t lane_mark_type = boundary_line.type;
    if (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_DASHED ==
        lane_mark_type) {
      // glLineStipple(2, 0x030F);
      glLineStipple(3, 0x00FF);
      glEnable(GL_LINE_STIPPLE);
    }
#endif
    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < boundary_line.curve_point_num; ++j) {
      DrawVertex(boundary_line.curve[j].x, boundary_line.curve[j].y, 0.15);
    }
    glEnd();
#if 0
    if (ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_DASHED ==
        lane_mark_type) {
      glDisable(GL_LINE_STIPPLE);
    }
#endif
  }

//  glColor3d(0.0, 1.0, 1.0);
//  glLineWidth(1.0f);
//  Int32_t entral_line_num = lane_line_list.central_line_num;
//  for (Int32_t i = 0; i < entral_line_num; ++i) {
//    const ad_msg::LaneCentralLineCamera& central_line =
//        lane_line_list.central_lines[i];

//    glBegin(GL_LINE_STRIP);
//    for (Int32_t j = 0; j < central_line.curve_point_num; ++j) {
//      DrawVertex(central_line.curve[j].x, central_line.curve[j].y, 0.15);
//    }
//    glEnd();
//  }
}

void WidgetMap::DrawObstacleCameraList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_camera_list;

  glColor3d(0.0, 0.6, 0.6);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.2F);

    com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
                 "id(%d) x(%0.1f) vx(%0.1f)",
                 obstacle.id,
                 obstacle.x,
                 obstacle.v_x*3.6);
    renderText(obstacle.x-view_param_.origin_coor.x,
               obstacle.y-view_param_.origin_coor.y,
               0.0 , string_buffer_, QFont("System", 10));
  }
}

void WidgetMap::DrawObstacleRadarFrontList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_radar_front_list;

  glColor3d(0.6, 0.6, 0.0);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.0F);
  }
}

void WidgetMap::DrawObstacleRadarRearList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_radar_rear_list;

  glColor3d(0.0, 0.6, 0.6);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.0F);
  }
}

void WidgetMap::DrawObstacleRadarFrontLeftList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_radar_front_left_list;

  qglColor(QColor(255, 0, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.0F);
  }
}

void WidgetMap::DrawObstacleRadarFrontRightList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_radar_front_right_list;

  qglColor(QColor(0, 255, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.0F);
  }
}

void WidgetMap::DrawObstacleRadarRearLeftList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_radar_rear_left_list;

  qglColor(QColor(0, 0, 255));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.0F);
  }
}

void WidgetMap::DrawObstacleRadarRearRightList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_radar_rear_right_list;

  qglColor(QColor(0, 255, 255));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                0.0F);
  }
}

void WidgetMap::DrawSrr2DetectionsFrontLeftList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.srr2_detections_front_left_list;

  qglColor(QColor(255, 0, 0));
  glLineWidth(2.0f);
  glPointSize(4.0F);

  glBegin(GL_POINTS);
  Int32_t obstacle_num = obstacle_list.obstacle_num;
  //std::cout << "#### obstacle_num=" << obstacle_num << std::endl;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    DrawVertex(obstacle.x, obstacle.y, 0.0F);
  }
  glEnd();
}

void WidgetMap::DrawSrr2DetectionsFrontRightList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.srr2_detections_front_right_list;

  qglColor(QColor(0, 255, 0));
  glLineWidth(2.0f);
  glPointSize(4.0F);

  glBegin(GL_POINTS);
  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    DrawVertex(obstacle.x, obstacle.y, 0.0F);
  }
  glEnd();
}

void WidgetMap::DrawSrr2DetectionsRearLeftList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.srr2_detections_rear_left_list;

  qglColor(QColor(0, 0, 255));
  glLineWidth(2.0f);
  glPointSize(4.0F);

  glBegin(GL_POINTS);
  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    DrawVertex(obstacle.x, obstacle.y, 0.0F);
  }
  glEnd();
}

void WidgetMap::DrawSrr2DetectionsRearRightList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.srr2_detections_rear_right_list;

  qglColor(QColor(0, 255, 255));
  glLineWidth(2.0f);

  glBegin(GL_POINTS);
  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    DrawVertex(obstacle.x, obstacle.y, 0.0F);
  }
  glEnd();
}

void WidgetMap::DrawObstacleLidarList0() {
  const ad_msg::ObstacleLidarList& obstacle_list =
      module_info_.obstacle_lidar_list_0;

  qglColor(QColor(0, 255, 255));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleLidar& obstacle = obstacle_list.obstacles[i];

    common::OBBox2d obb;
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_extents(obstacle.obb.half_length, obstacle.obb.half_width);

    DrawOBB_2D(obb, 1.0F);
  }
}

void WidgetMap::DrawObstacleLidarList1() {
  const ad_msg::ObstacleLidarList& obstacle_list =
      module_info_.obstacle_lidar_list_1;

  qglColor(QColor(255, 255, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleLidar& obstacle = obstacle_list.obstacles[i];

    common::OBBox2d obb;
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_extents(obstacle.obb.half_length, obstacle.obb.half_width);

    DrawOBB_2D(obb, 1.0F);
  }
}

void WidgetMap::DrawObstacleTrackedInfoList() {
  const ad_msg::ObstacleTrackedInfoList& obstacle_list =
      module_info_.obstacle_tracked_info_list;

  glColor3d(1.0, 1.0, 1.0);
  glLineWidth(1.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleTrackedInfo& obstacle = obstacle_list.obstacles[i];

    Float32_t half_length = common::Max(obstacle.obb.half_length, 0.5F);
    Float32_t half_width = common::Max(obstacle.obb.half_width, 0.5F);

    common::OBBox2d obb;
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_extents(half_length, half_width);
    DrawOBB_2D(obb, 0.0);

//    glBegin(GL_LINES);
//    {
//      DrawVertex(0.0, 0.0, 0.2);
//      DrawVertex(obstacle.x, obstacle.y, 0.2);
//    }
//    glEnd();
  }
}

void WidgetMap::DrawObstacleList() {
  veh_model::VehicleModelWrapper veh_model;
  const ad_msg::ObstacleList& obstacle_list =
      module_info_.obstacle_list;

  glColor3d(1.0, 0.0, 1.0);
  glLineWidth(1.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::Obstacle& obstacle = obstacle_list.obstacles[i];

    if (obstacle.confidence < 60) {
      qglColor(QColor(128, 128, 128, 128));
    } else {
      if (obstacle.dynamic) {
        qglColor(QColor(255, 0, 255));
      } else {
        glColor3d(0.0, 0.0, 1.0);
        qglColor(QColor(0, 0, 255));
      }
    }

    Float32_t half_length = common::Max(obstacle.obb.half_length, 0.1F);
    Float32_t half_width = common::Max(obstacle.obb.half_width, 0.1F);

    common::OBBox2d obb;
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_extents(half_length, half_width);
    DrawOBB_2D(obb, 1.0);
    Float32_t side_threshold_back =
        veh_model.GetDistOfLocalizationToRear() + 2.0F*obstacle.obb.half_length + 2.0F;
    if (ad_msg::VEH_TRAILER_STATUS_CONNECTED == module_info_.chassis_info.trailer_status) {
      side_threshold_back += veh_model.GetTrailerLength();
    }

    // if (obstacle.dynamic) {
    if ((obstacle.confidence >= 60) &&
        /*((obstacle.x-veh_model.GetDistOfLocalizationToFront()) > 2.0F) &&*/
        (common::com_abs(obstacle.y) < 15.0F)/* && (obstacle.dynamic)*/) {
      if (obstacle.x < 0.01F) {
        Float32_t t_gap = 0.0F;
        Float32_t v_diff = obstacle.v - module_info_.chassis_info.v;
        Float32_t ttc = 100.0F;
        if (v_diff > 0.01F) {
          ttc = -(obstacle.x + side_threshold_back) / v_diff;
          t_gap = (obstacle.x + side_threshold_back) /obstacle.v;
        } else {
          Float32_t v_plan = common::Max(20.0F/3.6F, module_info_.chassis_info.v);
          t_gap = (obstacle.x + side_threshold_back) / v_plan;
        }
        com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
                     "id(%d) x(%0.1f) vx(%0.1f), t_gap(%0.01f), ttc(%0.1f)",
                     obstacle.id,
                     obstacle.x + side_threshold_back,
                     obstacle.v_x*3.6,
                     t_gap,
                     ttc);
        renderText(obstacle.obb.x-view_param_.origin_coor.x,
                   obstacle.obb.y-view_param_.origin_coor.y,
                   0.0 , string_buffer_, QFont("System", 10));
      } else {
        com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
                     "id(%d) x(%0.1f) vx(%0.1f)",
                     obstacle.id,
                     obstacle.x - veh_model.GetDistOfLocalizationToFront(),
                     obstacle.v_x*3.6);
        renderText(obstacle.obb.x-view_param_.origin_coor.x,
                   obstacle.obb.y-view_param_.origin_coor.y,
                   0.0 , string_buffer_, QFont("System", 10));
      }

    }

    if (obstacle.tracked_path_point_num > 1) {
      glBegin(GL_LINE_STRIP);
      for (Int32_t j = 0; j < obstacle.tracked_path_point_num; ++j) {
        DrawVertex(obstacle.tracked_path[j].x, obstacle.tracked_path[j].y, 0.2);
      }
      glEnd();
    }
    if (obstacle.tracked_path_point_num > 0) {
      glPointSize(3.0);
      glBegin(GL_POINTS);
      for (Int32_t j = 0; j < obstacle.tracked_path_point_num; ++j) {
        DrawVertex(obstacle.tracked_path[j].x, obstacle.tracked_path[j].y, 0.2);
      }
      glEnd();
    }
  }
}


void WidgetMap::DrawDrivingMap() {
  DrawRoadArea();

  DrawRoadLaneLine();

  DrawMapTrafficLights();

  DrawReferenceLines();

  DrawObstaclesOfDrivingMap();

  // Nearest point to current vehicle position
  const phoenix::common::PathPoint& nearest_point_to_veh_on_lane =
      module_info_.driving_map_info.nearest_point_to_veh_on_lane;
  glColor3d(0.0, 0.0, 1.0);
  glLineWidth(1.0f);
  DrawEllipse(
        nearest_point_to_veh_on_lane.point.x(),
        nearest_point_to_veh_on_lane.point.y(),
        0.5, 0.5,
        nearest_point_to_veh_on_lane.heading,
        0.0, 6);
  DrawArrow(
        nearest_point_to_veh_on_lane.point.x(),
        nearest_point_to_veh_on_lane.point.y(),
        nearest_point_to_veh_on_lane.heading,
        2.0, 0.0);

  // Following target
  const driv_map::DrivingMapInfo::FollowingTarget& following_target =
      module_info_.driving_map_info.following_target;
  if (following_target.valid) {
    glLineWidth(3.0f);
    qglColor(QColor(255, 0, 0));
    DrawEllipse(following_target.obj_x, following_target.obj_y,
                1.0, 1.0, 0.0, 2.0, 3);
  }

#if 1
  // Planning storys
  qglColor(QColor(0, 0, 255));
  glLineWidth(3.0);
  const ad_msg::PlanningStoryList& story_list =
      module_info_.driving_map_info.planning_storys;
  for (Int32_t i = 0; i < story_list.story_num; ++i) {
    const ad_msg::PlanningStory& story = story_list.storys[i];

    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < story.ref_line.point_num; ++j) {
      DrawVertex(story.ref_line.points[j].x, story.ref_line.points[j].y, 0.5);
    }
    glEnd();
  }
#endif
}

void WidgetMap::DrawRoadArea() {
  Float64_t z_level = -0.1;

  // 道路边界采样点
  const phoenix::common::StaticVector<driv_map::RoadBoundary,
      phoenix::common::Path::kMaxPathPointNum>& road_boundary =
      module_info_.driving_map_info.road_boundary;
  Int32_t road_boundary_point_num = road_boundary.Size();
  // 道路边界Obj列表
  const common::StaticVector<common::OBBox2d,
      driv_map::MAX_OBJECT_NUM_IN_OBJ_SPACE>& road_boundary_obj_list =
      module_info_.driving_map_info.road_boundary_obj_list;;

#if 0
  // 绘制道路边界
  qglColor(QColor(255, 255, 255));
  glLineWidth(3.0);
  glBegin(GL_LINE_STRIP);
  for (Int32_t i = 0; i < road_boundary_point_num; ++i) {
    const phoenix::common::PathPoint& path_point =
        road_boundary[i].right_boundary_point;
    DrawVertex(path_point.point.x() - view_param_.origin_coor.x,
               path_point.point.y() - view_param_.origin_coor.y,
               z_level - view_param_.origin_coor.z);
  }
  glEnd();
  glBegin(GL_LINE_STRIP);
  for (Int32_t i = 0; i < road_boundary_point_num; ++i) {
    const phoenix::common::PathPoint& path_point =
        road_boundary[i].left_boundary_point;
    DrawVertex(path_point.point.x() - view_param_.origin_coor.x,
               path_point.point.y() - view_param_.origin_coor.y,
               z_level - view_param_.origin_coor.z);
  }
  glEnd();
#endif

#if 1
  // 绘制道路边界OBB
  qglColor(QColor(0, 0, 255));
  for (Int32_t i = 0; i < road_boundary_obj_list.Size(); ++i) {
    DrawOBB_2D(road_boundary_obj_list[i], 0.0);
  }
#endif

  // 绘制道路区域
  //qglColor(QColor(114, 123, 133));
  qglColor(QColor(164, 173, 183));
  DrawConcavePolygon concave;
  concave.Paint();
  {
    for (Int32_t i = 0; i < road_boundary_point_num; ++i) {
      const phoenix::common::PathPoint& path_point =
          road_boundary[i].right_boundary_point;
      concave.DrawVertex(path_point.point.x() - view_param_.origin_coor.x,
                         path_point.point.y() - view_param_.origin_coor.y,
                         z_level - view_param_.origin_coor.z);
    }

    for (Int32_t i = (road_boundary_point_num-1); i >= 0; --i) {
      const phoenix::common::PathPoint& path_point =
          road_boundary[i].left_boundary_point;
      concave.DrawVertex(path_point.point.x() - view_param_.origin_coor.x,
                         path_point.point.y() - view_param_.origin_coor.y,
                         z_level - view_param_.origin_coor.z);
    }
  }
  concave.EndPaint();
}

void WidgetMap::DrawRoadLaneLine() {
  Float64_t z_level = 0.05;
  // Float64_t z_level = 0.3;

  const common::StaticVector<driv_map::LaneInfo,
      driv_map::MAX_LANE_NUM>& lane_table =
      module_info_.driving_map_info.map.lane_table;
  Int32_t lane_table_size = lane_table.Size();

  for (Int32_t i = 0; i < lane_table_size; ++i) {
    const driv_map::LaneInfo& lane = lane_table[i];

    Int32_t central_curve_size = lane.central_curve.Size();
    if (central_curve_size > 1) {
      //glColor3d(0.0, 1.0, 0.0);
      qglColor(QColor(250, 200, 230));
      glLineStipple(3, 0x3F07);
      glEnable(GL_LINE_STIPPLE);
      glLineWidth(1.0f);
      glBegin(GL_LINE_STRIP);
      for (Int32_t j = 0; j < central_curve_size; ++j) {
        const phoenix::common::Vec2d& point = lane.central_curve[j];
        DrawVertex(point.x(), point.y(), z_level);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
    }

    Int32_t left_boundary_curve_size = lane.left_boundary.curve.Size();
    if (left_boundary_curve_size > 1) {
      switch (lane.quality) {
      case (driv_map::LANE_QUALITY_INVALID):
        qglColor(QColor(255, 0, 0));
        break;
      case (driv_map::LANE_QUALITY_BAD):
        qglColor(QColor(255, 0, 0));
        break;
      case (driv_map::LANE_QUALITY_NOT_GOOD):
        qglColor(QColor(200, 200, 200));
        break;
      case (driv_map::LANE_QUALITY_GOOD):
        qglColor(QColor(255, 255, 255));
        break;
      default:
        qglColor(QColor(128, 128, 128));
        break;
      }

      //qglColor(QColor(255, 255, 255));
      glLineWidth(3.0f);
      Int32_t boundary_type = lane.left_boundary.curve.Front().type;
      if ((phoenix::driv_map::LANE_BOUNDARY_TYPE_DOTTED_YELLOW ==
           boundary_type) ||
          (phoenix::driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE ==
           boundary_type)) {
        //glLineStipple(3, 0x3F07);
        glLineStipple(3, 0x00FF);
        glEnable(GL_LINE_STIPPLE);
      }

      glBegin(GL_LINE_STRIP);
      for (Int32_t j = 0; j < left_boundary_curve_size; ++j) {
        const phoenix::driv_map::LaneInfo::Boundary::BoundaryAssociation&
            boundary_point = lane.left_boundary.curve[j];
        DrawVertex(boundary_point.point.x(), boundary_point.point.y(), z_level);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
    }

    Int32_t right_boundary_curve_size = lane.right_boundary.curve.Size();
    if (right_boundary_curve_size > 1) {
      switch (lane.quality) {
      case (driv_map::LANE_QUALITY_INVALID):
        qglColor(QColor(255, 0, 0));
        break;
      case (driv_map::LANE_QUALITY_BAD):
        qglColor(QColor(255, 0, 0));
        break;
      case (driv_map::LANE_QUALITY_NOT_GOOD):
        qglColor(QColor(200, 200, 200));
        break;
      case (driv_map::LANE_QUALITY_GOOD):
        qglColor(QColor(255, 255, 255));
        break;
      default:
        qglColor(QColor(128, 128, 128));
        break;
      }

      //qglColor(QColor(255, 255, 255));
      glLineWidth(3.0f);
      Int32_t boundary_type = lane.right_boundary.curve.Front().type;
      if ((phoenix::driv_map::LANE_BOUNDARY_TYPE_DOTTED_YELLOW ==
           boundary_type) ||
          (phoenix::driv_map::LANE_BOUNDARY_TYPE_DOTTED_WHITE ==
           boundary_type)) {
        //glLineStipple(3, 0x3F07);
        glLineStipple(3, 0x00FF);
        glEnable(GL_LINE_STIPPLE);
      }

      glBegin(GL_LINE_STRIP);
      for (Int32_t j = 0; j < right_boundary_curve_size; ++j) {
        const phoenix::driv_map::LaneInfo::Boundary::BoundaryAssociation&
            boundary_point = lane.right_boundary.curve[j];
        DrawVertex(boundary_point.point.x(), boundary_point.point.y(), z_level);
      }
      glEnd();
      glDisable(GL_LINE_STIPPLE);
    }
  }
}

void WidgetMap::DrawMapTrafficLights() {
  const common::StaticVector<driv_map::MapTrafficLight,
      driv_map::MAX_MAP_TRAFFIC_LIGHT_NUM>& map_traffic_light_table =
      module_info_.driving_map_info.map.map_traffic_light_table;

  Float64_t z_level = -0.1;
  qglColor(QColor(128, 128, 128));
  glLineWidth(3.0F);

  glBegin(GL_LINES);
  Int32_t map_signal_num = map_traffic_light_table.Size();
  for (Int32_t i = 0; i < map_signal_num; ++i) {
    const driv_map::MapTrafficLight& map_signal = map_traffic_light_table[i];

    //std::cout << "  Get stop_line[(" << map_signal.stop_line.start().x()
    //          << ", " << map_signal.stop_line.start().y()
    //          << "), (" << map_signal.stop_line.end().x()
    //          << ", " << map_signal.stop_line.end().y()
    //          << ")]." << std::endl;

    DrawVertex(map_signal.stop_line.start().x(),
               map_signal.stop_line.start().y(), z_level);
    DrawVertex(map_signal.stop_line.end().x(),
               map_signal.stop_line.end().y(), z_level);
  }
  glEnd();
}

void WidgetMap::DrawReferenceLines() {
  Float64_t z_level = 0.0;

  // Reference lines
  glPointSize(4.0f);
  const phoenix::common::StaticVector<
      phoenix::driv_map::DrivingMapInfo::ReferenceLineInfo,
      phoenix::driv_map::MAX_REFERENCE_LINE_NUM>& reference_lines =
      module_info_.driving_map_info.reference_lines;
  Int32_t reference_lines_size = reference_lines.Size();

  for (Int32_t i = 0; i < reference_lines_size; ++i) {
    const phoenix::driv_map::DrivingMapInfo::ReferenceLineInfo& ref_line =
        reference_lines[i];

    glLineStipple(2, 0x030F);
    glEnable(GL_LINE_STIPPLE);
    if (module_info_.driving_map_info.current_reference_line_index == i) {
      glLineWidth(3.0f);
      glColor3d(0.8, 0.3, 0.2);
    } else {
      glLineWidth(1.0f);
      glColor3d(0.8, 0.3, 0.2);
    }
    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < ref_line.smooth_curve.Size(); ++j) {
      DrawVertex(ref_line.smooth_curve[j].point.x(),
                 ref_line.smooth_curve[j].point.y(),
                 z_level);
    }
    glEnd();
    glDisable(GL_LINE_STIPPLE);

    glColor3d(1.0, 0.0, 1.0);
    glBegin(GL_POINTS);
    for (Int32_t j = 0; j < ref_line.curve.Size(); ++j) {
      DrawVertex(ref_line.curve[j].point.x(),
                 ref_line.curve[j].point.y(),
                 z_level);
    }
    glEnd();
  }
}

void WidgetMap::DrawObstaclesOfDrivingMap() {
  // Obstacles
  glLineWidth(2.0);
  const phoenix::common::StaticVector<
      phoenix::driv_map::DrivingMapInfo::ObstacleInfo,
      phoenix::driv_map::MAX_OBJECT_NUM_IN_OBJ_SPACE>& obstacle_list =
      module_info_.driving_map_info.obstacle_list;
  for (Int32_t i = 0; i < obstacle_list.Size(); ++i) {
    const phoenix::driv_map::DrivingMapInfo::ObstacleInfo& obj =
        obstacle_list[i];

    double height = 2.0F;
    if (ad_msg::OBJ_TYPE_CURB == obj.type) {
     height = 3.0F;
    }

    if (obj.uncertain) {
      qglColor(QColor(128, 128, 128, 128));
      DrawOBB_2D(obj.obb, height);

      continue;
    }

    if (obj.dynamic) {
      // dynamic
      if (obj.ignore) {
        qglColor(QColor(128, 128, 128));
      } else {
        qglColor(QColor(0, 0, 200));
      }
      DrawOBB_2D(obj.obb, height);

      qglColor(QColor(0, 0, 255));
      glLineWidth(2.0);
      for (Int32_t j = 0; j < obj.pred_trajectory.Size(); ++j) {
        for (Int32_t k = 0; k < obj.pred_trajectory[j].Size(); ++k) {
          const phoenix::common::TrajectoryPoint& p = obj.pred_trajectory[j][k];
          DrawArrow(p.path_point.point.x(),
                    p.path_point.point.y(),
                    p.path_point.heading,
                    obj.obb.extents().x(), 1.5);
        }
      }

      //com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      //              "s: %0.1f, l: %0.1f, v: %0.1f"
      //              , obj.s_ref, obj.l_ref, obj.v*3.6);
      //glColor3d(0.0, 0.0, 1.0);
      //renderText(obj.obb.center().x()+1.0-view_param_.origin_coor.x
      //           , obj.obb.center().y()+1.0-view_param_.origin_coor.y
      //           , 2.0 , string_buffer_, QFont("AnyStyle", 10));
    } else {
      // static
      if (obj.ignore) {
        qglColor(QColor(128, 128, 128));
        DrawOBB_2D(obj.obb, height);
      } else {
        qglColor(QColor(0, 200, 0));
        DrawOBB_2D(obj.obb, height);

        //glColor3d(0.0, 0.0, 1.0);
        //com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
        //              "s: %0.1f, l: %0.1f"
        //              , obj.s_ref, obj.l_ref);
        //renderText(obj.obb.center().x()+1.0-view_param_.origin_coor.x
        //           , obj.obb.center().y()+1.0-view_param_.origin_coor.y
        //           , 2.0 , string_buffer_, QFont("AnyStyle", 10));
      }
    }
  }

  // Risky obstacles
  const phoenix::common::StaticVector<
      phoenix::driv_map::CollisionTestResult::ObjInfo,
      phoenix::driv_map::MAX_OBJECT_NUM_IN_OBJ_SPACE>&
      risky_obj_list = module_info_.driving_map_info.risky_obj_list;
  glLineWidth(3.0);
  for (Int32_t i = 0; i < risky_obj_list.Size(); ++i) {
    const phoenix::driv_map::CollisionTestResult::ObjInfo& obj = risky_obj_list[i];
    const phoenix::driv_map::DrivingMapInfo::ObstacleInfo& obstacle =
        obstacle_list[obj.obj_list_index];
    if (obj.risk_value > 90) {
      qglColor(QColor(255, 0, 0));
    } else {
      if (obj.dynamic_distance < 0.5) {
        qglColor(QColor(255, 128, 0));
      } else {
        qglColor(QColor(255, 255, 0));
      }
    }
    Float64_t r = common::Max(obstacle.obb.extents().x(),
                              obstacle.obb.extents().y()) + 0.5;
    DrawEllipse(obstacle.obb.center().x(),
                obstacle.obb.center().y(),
                r, r, 0.0, 0.2, 20);

#if 0
    if (obstacle.dynamic) {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
                    "s: %0.1f, dist: %0.1f, v: %0.1f"
                    , obstacle.s_ref, obj.static_distance, obstacle.v*3.6);
      renderText(obstacle.obb.center().x()+1.0-view_param_.origin_coor.x
                 , obstacle.obb.center().y()+1.0-view_param_.origin_coor.y
                 , 0.0 , string_buffer_, QFont("AnyStyle", 15));
    } else {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
                    "s: %0.1f, dist: %0.1f"
                    , obstacle.s_ref, obj.static_distance);
      renderText(obstacle.obb.center().x()+1.0-view_param_.origin_coor.x
                 , obstacle.obb.center().y()+1.0-view_param_.origin_coor.y
                 , 0.0 , string_buffer_, QFont("AnyStyle", 15));
    }
#endif
  }
}

void WidgetMap::DrawTrajectoryPlanningInfo() {

  DrawTargetTrajectory();

  const planning::TrajectoryPlanningInfo& trj_planning_info =
      module_info_.trajectory_planning_info;
  // Road graph of trajectory planner
  // Links
#if 1
  glLineWidth(1.0f);
  for (Int32_t i = 0;
       i < trj_planning_info.road_graph.road_graph_links.Size(); ++i) {
    const planning::TrajectoryPlanningInfo::RoadGraph::Link& link =
        trj_planning_info.road_graph.road_graph_links[i];

    if (link.is_collision_with_road_boundary) {
      qglColor(QColor(255, 0, 0));
    } else {
      //glColor3d(0.4, 0.4, 0.4);
      //qglColor(QColor(194, 203, 213));
      //qglColor(QColor(255, 0, 0));
      qglColor(QColor(200, 240, 230));
    }

    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < link.sample_points.Size(); ++j) {
      const common::PathPoint& path_point = link.sample_points[j];
      DrawVertex(path_point.point.x(), path_point.point.y(), 0.0);
    }
    glEnd();
  }
#endif

  // Optimal trajectory
  glColor3d(0.0, 0.0, 1.0);
  glLineWidth(2.0f);
  const common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum>& optimal_trj =
      trj_planning_info.optimal_trajectory_sample_points;
  glBegin(GL_LINE_STRIP);
  Int32_t optimal_trj_size = optimal_trj.Size();
  for (Int32_t i = 0; i < optimal_trj_size; ++i) {
    const common::PathPoint& path_point = optimal_trj[i];
    DrawVertex(path_point.point.x(), path_point.point.y(), 0.0);
  }
  glEnd();

  // previous reference trajectory
  glColor3d(0.0, 0.0, 0.0);
  glLineWidth(2.0f);
  const common::StaticVector<common::PathPoint,
      common::Path::kMaxPathPointNum>& prev_ref_trj =
      trj_planning_info.prev_ref_line_sample_points;
  glBegin(GL_LINE_STRIP);
  Int32_t prev_ref_trj_size = prev_ref_trj.Size();
  for (Int32_t i = 0; i < prev_ref_trj_size; ++i) {
    const common::PathPoint& path_point = prev_ref_trj[i];
    DrawVertex(path_point.point.x(), path_point.point.y(), 0.0);
  }
  glEnd();

  const planning::TrajectoryPlanningResult& trj_planning_ret =
      module_info_.trajectory_planning_result ;

  // Leading point
  glLineWidth(1.0f);
  glColor3d(1.0, 0.0, 0.0);
  DrawEllipse(trj_planning_ret.leading_pos.x,
              trj_planning_ret.leading_pos.y,
              0.5, 0.5, 0.0, 0.2, 12);
  DrawArrow(trj_planning_ret.leading_pos.x,
            trj_planning_ret.leading_pos.y,
            trj_planning_ret.leading_pos.heading,
            4.0, 0.2);

  // sample step length
  glColor3d(1.0, 1.0, 0.0);
  glLineWidth(2.0f);
  glBegin(GL_LINES);
  DrawVertex(trj_planning_info.leading_length_for_lka, -5.0, 0.0);
  DrawVertex(trj_planning_info.leading_length_for_lka, 5.0, 0.0);

  qglColor(QColor(0, 0, 255, 100));
  glLineWidth(1.0f);
  Float32_t accumulation = 0.0F;
  for (Int32_t i = 0;
       i < trj_planning_info.lon_graph_samples_step_len.Size(); ++i) {
    accumulation += trj_planning_info.lon_graph_samples_step_len[i];

    DrawVertex(accumulation, -6.0, 0.0);
    DrawVertex(accumulation, 6.0, 0.0);
  }
  glEnd();

  // lateral err predicted point
  qglColor(QColor(0, 0, 255));
  glPointSize(8.0F);
  glBegin(GL_POINTS);
  DrawVertex(trj_planning_info.lat_err_pred_point.x,
             trj_planning_info.lat_err_pred_point.y, 0.3);
  glEnd();
  DrawArrow(trj_planning_info.lat_err_pred_point.x,
            trj_planning_info.lat_err_pred_point.y,
            trj_planning_info.lat_err_pred_point.h,
            2.0, 0.3);
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
               "0: err(%0.2f) err_v(%0.2f)",
               trj_planning_info.lat_err_sample[0].lat_err_smooth,
               trj_planning_info.lat_err_sample[0].lat_err_v_smooth);
  renderText(0.0-view_param_.origin_coor.x,
             0.0-view_param_.origin_coor.y,
             0.0 , string_buffer_, QFont("System", 10));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
               "1: err(%0.2f) err_v(%0.2f)",
               trj_planning_info.lat_err_sample[1].lat_err_smooth,
               trj_planning_info.lat_err_sample[1].lat_err_v_smooth);
  renderText(-1.0-view_param_.origin_coor.x,
             0.0-view_param_.origin_coor.y,
             0.0 , string_buffer_, QFont("System", 10));
}

void WidgetMap::DrawTargetTrajectory() {
  const phoenix::common::StaticVector<phoenix::common::PathPoint,
      phoenix::common::Path::kMaxPathPointNum>& tar_trajectory =
      module_info_.trajectory_planning_result.target_trajectory;
  Int32_t tar_trajectory_size = tar_trajectory.Size();

  glLineWidth(1.0f);
  glColor3d(0.0, 1.0, 0.0);
  for (Int32_t i = 0; i < tar_trajectory_size; ++i) {
    const phoenix::common::PathPoint& path_point = tar_trajectory[i];
    DrawArrow(path_point.point.x(), path_point.point.y(),
              path_point.heading, 1.0, 0.1);
  }

  left_boundary_of_target_trajectory_.Clear();
  right_boundary_of_target_trajectory_.Clear();
  phoenix::common::Vec2d left_point;
  phoenix::common::Vec2d right_point;
  phoenix::common::Vec2d cross_point;
  phoenix::common::LineSegment2d prev_left_seg;
  phoenix::common::LineSegment2d prev_right_seg;
  Float32_t half_width = 1.25f;
  Float32_t cross_ratio = 0.0f;
  for (Int32_t i = 0; i < tar_trajectory_size; ++i) {
    const phoenix::common::PathPoint& path_point = tar_trajectory[i];

    left_point.set_x(path_point.point.x() -
                     half_width *
                     phoenix::common::com_sin(path_point.heading));
    left_point.set_y(path_point.point.y() +
                     half_width *
                     phoenix::common::com_cos(path_point.heading));

    right_point.set_x(path_point.point.x() +
                      half_width *
                      phoenix::common::com_sin(path_point.heading));
    right_point.set_y(path_point.point.y() -
                      half_width *
                      phoenix::common::com_cos(path_point.heading));

    bool left_point_valid = true;
    bool right_point_valid = true;
    if (0 == i) {
      prev_left_seg.set_start(path_point.point);
      prev_left_seg.set_end(left_point);

      prev_right_seg.set_start(path_point.point);
      prev_right_seg.set_end(right_point);
    } else {
      if (phoenix::common::OverlapTestSegToSeg_2D(
            path_point.point, left_point,
            prev_left_seg.start(), prev_left_seg.end(),
            &cross_point, &cross_ratio)) {
        left_point_valid = false;
      } else {
        prev_left_seg.set_start(path_point.point);
        prev_left_seg.set_end(left_point);
      }

      if (phoenix::common::OverlapTestSegToSeg_2D(
            path_point.point, right_point,
            prev_right_seg.start(), prev_right_seg.end(),
            &cross_point, &cross_ratio)) {
        right_point_valid = false;
      } else {
        prev_right_seg.set_start(path_point.point);
        prev_right_seg.set_end(right_point);
      }
    }

    if (left_point_valid) {
      left_boundary_of_target_trajectory_.PushBack(left_point);
    }

    if (right_point_valid) {
      right_boundary_of_target_trajectory_.PushBack(right_point);
    }
  }

  Int32_t left_boundary_point_num =
      left_boundary_of_target_trajectory_.Size();
  Int32_t right_boundary_point_num =
      right_boundary_of_target_trajectory_.Size();

#if 0
  glBegin(GL_LINE_STRIP);
  for (Int32_t i = 0; i < left_boundary_point_num; ++i) {
    const phoenix::common::Vec2d& point =
        left_boundary_of_target_trajectory_[i];
    DrawVertex(point.x(), point.y(), 0.0);
  }
  glEnd();
  glBegin(GL_LINE_STRIP);
  for (Int32_t i = 0; i < right_boundary_point_num; ++i) {
    const phoenix::common::Vec2d& point =
        right_boundary_of_target_trajectory_[i];
    DrawVertex(point.x(), point.y(), 0.0);
  }
  glEnd();
#endif

  switch (module_info_.trajectory_planning_info.road_builed_type) {
  case (planning::ROAD_BUILED_TYPE_HD_MAP):
    // 目标轨迹：HDMap
    qglColor(QColor(150, 251, 120, 200));
    break;
  case (planning::ROAD_BUILED_TYPE_CAMERA_LANE):
    // 目标轨迹：识别的车道线
    qglColor(QColor(150, 251, 120, 200));
    break;
  case (planning::ROAD_BUILED_TYPE_FOLLOWING):
    // 目标轨迹：跟随前车
    qglColor(QColor(200, 251, 120, 200));
    break;
  case (planning::ROAD_BUILED_TYPE_PRED_PATH):
    // 目标轨迹：不受控
    qglColor(QColor(250, 251, 120, 200));
    break;
  case (planning::ROAD_BUILED_TYPE_MIXED_HD_MAP):
    // 目标轨迹：Mixed HDMap
    qglColor(QColor(150, 251, 120, 200));
    break;
  default:
    // 目标轨迹：内部异常
    qglColor(QColor(250, 251, 120, 200));
    break;
  }

#if 0
  DrawConcavePolygon concave;
  concave.Paint();
  {
    for (Int32_t i = 0; i < right_boundary_point_num; ++i) {
      const phoenix::common::Vec2d& point =
          right_boundary_of_target_trajectory_[i];
      concave.DrawVertex(point.x() - view_param_.origin_coor.x,
                         point.y() - view_param_.origin_coor.y,
                         0.0 - view_param_.origin_coor.z);
    }

    for (Int32_t i = (left_boundary_point_num-1); i >= 0; --i) {
      const phoenix::common::Vec2d& point =
          left_boundary_of_target_trajectory_[i];
      concave.DrawVertex(point.x() - view_param_.origin_coor.x,
                         point.y() - view_param_.origin_coor.y,
                         0.0 - view_param_.origin_coor.z);
    }
  }
  concave.EndPaint();
#endif
}

void WidgetMap::DrawVelocityPlanningInfo() {
  const planning::VelocityPlanningResult& v_plan =
      module_info_.velocity_planning_result;

  veh_model::VehicleModelWrapper veh_model;

  Float64_t tar_pos_x_bar = v_plan.tar_pos.x + veh_model.GetDistOfLocalizationToFront();
  Float64_t tar_pos_y_bar = v_plan.tar_pos.y;

  Float64_t tar_pos_x_txt = v_plan.tar_pos.x + 1.0 - view_param_.origin_coor.x + veh_model.GetDistOfLocalizationToFront();
  Float64_t tar_pos_y_txt = v_plan.tar_pos.y + 1.0 - view_param_.origin_coor.y;

  glColor3d(1.0, 0.0, 0.0);
  switch (v_plan.tar_type) {
  case (planning::VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS):
    break;

  case (planning::VELOCITY_PLANNING_TARGET_TYPE_CURVATURE):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  Curvature Limit (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_OBSTACLE):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  Obstacles (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_FOLLOWING):
    glColor3d(0.0, 0.5, 1.0);
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  跟随 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    if (v_plan.tar_a < -0.1F) {
      glColor3d(1.0, 0.0, 0.0);
    }
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_LOW_SPEED_FOLLOWING):
    glColor3d(0.0, 0.5, 1.0);
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  低速跟随 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    if (v_plan.tar_a < -0.1F) {
      glColor3d(1.0, 0.0, 0.0);
    }
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_AEB_WARN):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  AEB_WARN (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_AEB_ACTION):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  AEB (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (planning::VELOCITY_PLANNING_TARGET_TYPE_SCENE_STORY):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  Story (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (planning::VELOCITY_PLANNING_TARGET_TYPE_TRAFFIC_LIGHT):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  红绿灯 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (planning::VELOCITY_PLANNING_TARGET_TYPE_ISL):
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  限速牌 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_TUNNEL):
    glColor3d(0.0, 0.5, 1.0);
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  隧道 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    if (v_plan.tar_a < -0.1F) {
      glColor3d(1.0, 0.0, 0.0);
    }
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_RAMP):
    glColor3d(0.0, 0.5, 1.0);
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  匝道 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    if (v_plan.tar_a < -0.1F) {
      glColor3d(1.0, 0.0, 0.0);
    }
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  case (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_LANE_SPEED_LIMIT):
    glColor3d(0.0, 0.5, 1.0);
    DrawBarrier(tar_pos_x_bar, tar_pos_y_bar, v_plan.tar_pos.heading);
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "  道路限速 (v:%0.1f, a:%0.2f, s:%0.1f)",
                  v_plan.tar_v*3.6, v_plan.tar_a, v_plan.tar_pos.s);
    if (v_plan.tar_a < -0.1F) {
      glColor3d(1.0, 0.0, 0.0);
    }
    renderText(tar_pos_x_txt, tar_pos_y_txt, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
    break;

  default:
    break;
  }

  if ((planning::VELOCITY_PLANNING_TARGET_TYPE_INVALID != v_plan.tar_type) &&
      (planning::VELOCITY_PLANNING_TARGET_TYPE_USER_SETTINGS != v_plan.tar_type)) {
    switch (v_plan.tar_obj.obj_dec_status) {
    case (0):
      qglColor(QColor(0, 0, 255));
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "Free");
      break;
    case (1):
      qglColor(QColor(255, 255, 0));
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "Req Decl");
      break;
    case (2):
      qglColor(QColor(255, 0, 0));
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "In Decl");
      break;
    case (3):
      qglColor(QColor(0, 255, 255));
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "Req Accl");
      break;
    default:
      qglColor(QColor(255, 0, 0));
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "Unknown");
      break;
    }
    renderText(tar_pos_x_txt, tar_pos_y_txt + 2.0, 0.0,
               string_buffer_, QFont("AnyStyle", 12));
  }
}


struct CirclePoint{
  Float64_t x;
  Float64_t y;
  Float64_t angle;
};

static std::vector<CirclePoint> GetVelocityPedalShape(double r) {
  std::vector<CirclePoint> shape;

  Int32_t point_num = 100;
  Float64_t start_angle = common::com_deg2rad(-45.0);
  Float64_t end_angle = common::com_deg2rad(180.0+45.0);
  Float64_t delta_angle = (end_angle-start_angle) / point_num;
  CirclePoint point;
  for (Int32_t i = 0; i <= point_num; ++i) {
    Float64_t angle = start_angle+i*delta_angle;
    point.x = (r*common::com_cos(common::NormalizeAngle(angle)));
    point.y = (r*common::com_sin(common::NormalizeAngle(angle)));
    point.angle = angle;

    shape.push_back(point);
  }

  return (shape);
}

static std::vector<CirclePoint> GetSpeedRestrictionMarkShape(double r) {
  std::vector<CirclePoint> shape;

  Int32_t point_num = 100;
  Float64_t start_angle = common::com_deg2rad(0.0);
  Float64_t end_angle = common::com_deg2rad(360.0);
  Float64_t delta_angle = (end_angle-start_angle) / point_num;
  CirclePoint point;
  for (Int32_t i = 0; i <= point_num; ++i) {
    Float64_t angle = start_angle+i*delta_angle;
    point.x = (r*common::com_cos(common::NormalizeAngle(angle)));
    point.y = (r*common::com_sin(common::NormalizeAngle(angle)));
    point.angle = angle;

    shape.push_back(point);
  }

  return (shape);
}

void WidgetMap::DrawSpeedRestrictionMark() {
  static std::vector<CirclePoint> steering_shape =
      GetSpeedRestrictionMarkShape(50.0);

  const ad_msg::TrafficSignalList& traffic_signal_list =
      module_info_.traffic_signal_list;

  if (!traffic_signal_list.msg_head.valid) {
    return;
  }

  Int32_t tar_idx = -1;
  for (Int32_t i = 0; i < traffic_signal_list.speed_restriction_num; ++i) {
    const ad_msg::TrafficSignalSpeedRestriction& cur_signal =
        traffic_signal_list.speed_restrictions[i];

    if (ad_msg::TrafficSignalSpeedRestriction::START_RESTRICTION ==
        cur_signal.type) {
      if (tar_idx < 0) {
        tar_idx = i;
      } else {
        if (cur_signal.speed >
            traffic_signal_list.speed_restrictions[tar_idx].speed) {
          tar_idx = i;
        }
      }
    }
  }

  Int32_t h = height();
  Int32_t offset_x = 180;
  Int32_t offset_y = h - 105;

  if (tar_idx >= 0) {
    qglColor(QColor(255, 0, 0));
    glLineWidth(4.0);
    glBegin(GL_LINE_STRIP);
    for (const auto& point : steering_shape) {
      glVertex3d(offset_x+point.x, offset_y+point.y, 0.0);
    }
    glEnd();

    qglColor(QColor(0, 0, 0));
    com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%03d"
        , (Int32_t)(traffic_signal_list.speed_restrictions[tar_idx].speed*3.6F));
    renderText(offset_x-45, h - offset_y+20,
               string_buffer_, QFont("AnyStyle", 40));
  }
}

void WidgetMap::DrawTrafficLight() {
  const ad_msg::TrafficLightList& traffic_light_list =
      module_info_.traffic_light_list;

  if (!traffic_light_list.msg_head.valid) {
    return;
  }

  Int32_t w = width();
  Int32_t h = height();
  Int32_t offset_x = w - 260;
  Int32_t offset_y = h - 95;//270;

  bool traffic_light_is_red = false;
  for (Int32_t i = 0; i < traffic_light_list.traffic_light_num; ++i) {
    if (ad_msg::TrafficLight::RED ==
        traffic_light_list.traffic_lights[i].color) {
      traffic_light_is_red = true;
      break;
    } else if (ad_msg::TrafficLight::YELLOW ==
               traffic_light_list.traffic_lights[i].color) {
      traffic_light_is_red = true;
      break;
    } else {
      // nothing to do
    }
  }
  if (traffic_light_is_red) {
    // 红灯
    qglColor(QColor(0, 100, 255));
    com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "红灯");
    renderText(offset_x, h-offset_y, string_buffer_, QFont("AnyStyle", 20));
  } else {
    bool traffic_light_is_green = false;
    for (Int32_t i = 0; i < traffic_light_list.traffic_light_num; ++i) {
      if (ad_msg::TrafficLight::GREEN ==
          traffic_light_list.traffic_lights[i].color) {
        traffic_light_is_green = true;
        break;
      }
    }
    if (traffic_light_is_green) {
      // 绿灯
      qglColor(QColor(0, 100, 255));
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "绿灯");
      renderText(offset_x, h-offset_y, string_buffer_, QFont("AnyStyle", 20));
    }
  }
}

void WidgetMap::DrawVelocityPedal() {
  static std::vector<CirclePoint> steering_shape =
      GetVelocityPedalShape(90.0);

  Int32_t w = width();
  Int32_t h = height();
  Int32_t r = 90;
  Int32_t offset_x = w - 100;
  Int32_t offset_y = h - 200;//270;

  const ad_msg::Chassis& chassis_info = module_info_.chassis_info;
  const planning::VelocityPlanningResult& velocity_planning_result =
      module_info_.velocity_planning_result;

  qglColor(QColor(255, 255, 255));
  glLineWidth(1.0);

  glBegin(GL_LINE_STRIP);
  for (const auto& point : steering_shape) {
    glVertex3d(offset_x+point.x, offset_y+point.y, 0.0);
  }
  glEnd();

  Float64_t max_velocity = 200.0/3.6;
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%d", static_cast<Int32_t>(max_velocity*3.6));
  renderText(offset_x+0.85*r, h-offset_y+0.73*r,
      string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%d", 0);
  renderText(offset_x-0.85*r-10, h-offset_y+0.73*r,
      string_buffer_, QFont("AnyStyle", 10));

  // dest velocity
  glLineWidth(2.0);
  qglColor(QColor(0, 100, 255));
  phoenix::common::Vec2d arrow_end_point;
  Float64_t angle = phoenix::common::com_deg2rad(180.0+45.0) -
      phoenix::common::com_deg2rad(270.0)
      *((velocity_planning_result.tar_v) / max_velocity);
  arrow_end_point(0) = offset_x + r*common::com_cos(common::NormalizeAngle(angle));
  arrow_end_point(1) = offset_y + r*common::com_sin(common::NormalizeAngle(angle));
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(arrow_end_point(0), arrow_end_point(1), 0.0);
  }
  glEnd();


  // current velocity
  qglColor(QColor(100, 255, 0, 150));
  angle = phoenix::common::com_deg2rad(180.0+45.0) -
      phoenix::common::com_deg2rad(270.0)
      *((chassis_info.v) / max_velocity);
  arrow_end_point(0) = offset_x + r*common::com_cos(common::NormalizeAngle(angle));
  arrow_end_point(1) = offset_y + r*common::com_sin(common::NormalizeAngle(angle));
#if 0
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(arrow_end_point(0), arrow_end_point(1), 0.0);
  }
  glEnd();
#else
  glBegin(GL_POLYGON);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(arrow_end_point(0), arrow_end_point(1), 0.0);
    for (const auto& point : steering_shape) {
      if (point.angle > angle) {
        glVertex3d(offset_x+point.x, offset_y+point.y, 0.0);
      }
    }
  }
  glEnd();
#endif

  qglColor(QColor(0, 100, 255));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "t:%03d -- "
      , static_cast<Int32_t>(common::com_round(
                               velocity_planning_result.tar_v*3.6F)));
  renderText(offset_x-60, h-offset_y-10, string_buffer_, QFont("AnyStyle", 20));
  qglColor(QColor(255, 100, 0));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%03d",
      static_cast<Int32_t>(phoenix::common::com_round(chassis_info.v*3.6F)));
  renderText(offset_x+20, h-offset_y-10, string_buffer_, QFont("AnyStyle", 20));
}

void WidgetMap::DrawAccelerationPedal() {
  Int32_t w = width();
  Int32_t h = height();

  Int32_t offset_x = w - 70;
  Int32_t offset_y = h - 360;//430;

  Int32_t half_pedal_width = 15;
  Int32_t pedal_height = 80;

  const ad_msg::Chassis& chassis_info = module_info_.chassis_info;
  const planning::VelocityPlanningResult& velocity_planning_result =
      module_info_.velocity_planning_result;

  Int32_t target = common::com_round(
      pedal_height * velocity_planning_result.tar_a * 0.1);
  Int32_t current = phoenix::common::com_round(
      pedal_height * chassis_info.a * 0.1);

  // acceleration
  qglColor(QColor(255, 255, 255));
  glLineWidth(1.0);
  // border
  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(offset_x-half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y+pedal_height, 0.0);
    glVertex3d(offset_x-half_pedal_width, offset_y+pedal_height, 0.0);
  }
  glEnd();

  // deceleration
  qglColor(QColor(255, 255, 255));
  glLineWidth(1.0);
  // border
  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(offset_x-half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y-pedal_height, 0.0);
    glVertex3d(offset_x-half_pedal_width, offset_y-pedal_height, 0.0);
  }
  glEnd();

  // target
  glLineWidth(2.0);
  if (velocity_planning_result.tar_a > -0.02F) {
    qglColor(QColor(0, 100, 255));
  } else {
    qglColor(QColor(255, 0, 0));
  }
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x-half_pedal_width, offset_y+target, 0.0);
    glVertex3d(offset_x+half_pedal_width+25, offset_y+target, 0.0);
  }
  glEnd();
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.2f",
      velocity_planning_result.tar_a);
  renderText(offset_x+half_pedal_width+5, h-offset_y-target-5,
      string_buffer_, QFont("AnyStyle", 12));

  // current
  glLineWidth(1.0);
  if (chassis_info.a > -0.2F) {
    qglColor(QColor(100, 255, 0, 150));
  } else {
    qglColor(QColor(255, 0, 0, 150));
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(offset_x-half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width-1, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width-1, offset_y+current, 0.0);
    glVertex3d(offset_x-half_pedal_width, offset_y+current, 0.0);
  }
  glEnd();
  if (chassis_info.a > -0.2F) {
    qglColor(QColor(0, 100, 255));
    com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.2f",
        chassis_info.a);
    renderText(offset_x-15, h-offset_y-current-1,
        string_buffer_, QFont("AnyStyle", 12));
  } else {
    qglColor(QColor(255, 0, 0));
    com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.2f",
        chassis_info.a);
    renderText(offset_x-15, h-offset_y-current+15,
        string_buffer_, QFont("AnyStyle", 12));
  }
}

void WidgetMap::DrawLongitudinalCtlValuePedal() {
  Int32_t w = width();
  Int32_t h = height();

  Int32_t offset_x = w - 73;
  Int32_t offset_y = h - 580;//20;

  Int32_t pedal_width = 30;
  Int32_t pedal_height = 120;

  Int32_t interval = 10;

  qglColor(QColor(255, 255, 255));
  glLineWidth(1.0);

  // Brake pedal
  offset_x -= pedal_width+0.2*interval;
  // border
  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width, offset_y+pedal_height, 0.0);
    glVertex3d(offset_x, offset_y+pedal_height, 0.0);
  }
  glEnd();

  // pedal
  qglColor(QColor(255, 0, 0));
  int pedal_value =
      module_info_.chassis_info.brake_pedal_value*0.01*pedal_height;
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x-20, offset_y+pedal_value, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y+pedal_value, 0.0);
  }
  glEnd();
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%d", module_info_.chassis_info.brake_pedal_value);
  renderText(offset_x+pedal_width-48, h-offset_y-pedal_value-3,
      string_buffer_, QFont("AnyStyle", 10));

  // control value
  qglColor(QColor(255, 0, 0, 150));
  int ctl_value = module_info_.chassis_ctl_cmd_info.brake_value*0.01*pedal_height;
  glBegin(GL_QUADS);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y+ctl_value, 0.0);
    glVertex3d(offset_x, offset_y+ctl_value, 0.0);
  }
  glEnd();
  qglColor(QColor(255, 0, 0));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%0.1f", module_info_.chassis_ctl_cmd_info.brake_value);
  renderText(offset_x+pedal_width-28, h-offset_y-ctl_value-2,
      string_buffer_, QFont("AnyStyle", 10));


  // Accelerator pedal
  qglColor(QColor(255, 255, 255));
  offset_x += pedal_width+interval;
  // border
  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width, offset_y+pedal_height, 0.0);
    glVertex3d(offset_x, offset_y+pedal_height, 0.0);
  }
  glEnd();

  // pedal
  qglColor(QColor(0, 0, 255));
  pedal_value = module_info_.chassis_info.acc_pedal_value*0.01*pedal_height;
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x, offset_y+pedal_value, 0.0);
    glVertex3d(offset_x+pedal_width+20, offset_y+pedal_value, 0.0);
  }
  glEnd();
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%d", module_info_.chassis_info.acc_pedal_value);
  renderText(offset_x+pedal_width+5, h-offset_y-pedal_value-3,
      string_buffer_, QFont("AnyStyle", 10));

  // control value
  qglColor(QColor(0, 0, 255, 150));
  ctl_value = module_info_.chassis_ctl_cmd_info.acc_value*0.01*pedal_height;
  glBegin(GL_QUADS);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y+ctl_value, 0.0);
    glVertex3d(offset_x, offset_y+ctl_value, 0.0);
  }
  glEnd();
  qglColor(QColor(0, 0, 255));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%0.1f", module_info_.chassis_ctl_cmd_info.acc_value);
  renderText(offset_x+pedal_width-28, h-offset_y-ctl_value-2,
      string_buffer_, QFont("AnyStyle", 10));
}

void WidgetMap::DrawSteeringWheelAnglePedal() {
  Int32_t w = width();
  Int32_t h = height();

  Int32_t offset_x = w - 100;
  Int32_t offset_y = h - 100;

  Int32_t pedal_height = 30;
  Int32_t half_pedal_width = 90;

  veh_model::VehicleModelWrapper veh_model;
  Float32_t max_steering_wheel_angle = veh_model.GetMaxSteeringAngle();
  const ad_msg::Chassis& chassis_info = module_info_.chassis_info;
  const ad_msg::ChassisCtlCmd& chassis_ctl_cmd_info = module_info_.chassis_ctl_cmd_info;
  Float32_t  tar_steering_wheel_angle = chassis_ctl_cmd_info.steering_wheel_angle; // 规划的方向盘转角(待给定)

  Int32_t target = common::com_round(
      half_pedal_width * tar_steering_wheel_angle/max_steering_wheel_angle);

  Int32_t current = phoenix::common::com_round(
      half_pedal_width * chassis_info.steering_wheel_angle/max_steering_wheel_angle);

  // 右边方向盘
  qglColor(QColor(255, 255, 255));
  glLineWidth(1.0);
  // border
  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x-half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x-half_pedal_width, offset_y+pedal_height, 0.0);
    glVertex3d(offset_x, offset_y+pedal_height, 0.0);
  }
  glEnd();

  // 左边方向盘
  qglColor(QColor(255, 255, 255));
  glLineWidth(1.0);
  // border
  glBegin(GL_LINE_LOOP);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y+pedal_height, 0.0);
    glVertex3d(offset_x, offset_y+pedal_height, 0.0);
  }
  glEnd();

  // target
  glLineWidth(2.0);
  if (tar_steering_wheel_angle >= 0) {
    qglColor(QColor(0, 100, 255));
  } else {
    qglColor(QColor(250, 255, 80));
  }
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x - target, offset_y, 0.0);
    glVertex3d(offset_x - target, offset_y + pedal_height + 0, 0.0);
  }
  glEnd();
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.1f",
      tar_steering_wheel_angle*180.0F/3.14F);
  renderText(offset_x - 15.0F - target, h - offset_y - 35.0F,
      string_buffer_, QFont("AnyStyle", 12));

  // current
  glLineWidth(1.0);
  if (chassis_info.steering_wheel_angle >= 0) {
    qglColor(QColor(100, 255, 0, 150));
  } else {
    qglColor(QColor(250, 255, 50, 150));
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(offset_x, offset_y, 0.0F);
    glVertex3d(offset_x, offset_y + pedal_height, 0.0F);
    glVertex3d(offset_x - current, offset_y+pedal_height, 0.0F);
    glVertex3d(offset_x - current, offset_y, 0.0F);
  }
  glEnd();
  if (chassis_info.steering_wheel_angle >= 0) {
    qglColor(QColor(0, 100, 255));
    com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.1f",
        chassis_info.steering_wheel_angle*180.0F/3.14F);
    renderText(offset_x - 25.0F - current, h - offset_y - 10.0F,
        string_buffer_, QFont("AnyStyle", 12));
  } else {
    qglColor(QColor(255, 100, 0));
    com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.1f",
        chassis_info.steering_wheel_angle*180.0F/3.14F);
    renderText(offset_x - 25.0F - current, h - offset_y - 10.0F,
        string_buffer_, QFont("AnyStyle", 12));
  }
}

void WidgetMap::DrawModuleStatusInfo() {
  QColor background_normal(82, 235, 233, 200);
  QColor background_warn(255, 255, 0, 200);
  QColor background_err(255, 0, 0, 200);
  QColor background_timeout(255, 128, 128, 200);

  Int32_t draw_counter = 0;
  auto func_draw_lable = [this, &draw_counter](const Char_t* str) {
    Float64_t lable_width = 20;
    Float64_t lable_length = 100;
    Float64_t lable_x_left = 10.0;
    Float64_t lable_y_up = height() - 200.0 - draw_counter*25.0;
    Int32_t text_x = 12;
    Int32_t text_y = 215 + draw_counter*25;
    draw_counter++;
    glBegin(GL_QUADS);
    {
      glVertex3d(lable_x_left, lable_y_up, 0.0);
      glVertex3d(lable_x_left, lable_y_up-lable_width, 0.0);
      glVertex3d(lable_x_left+lable_length, lable_y_up-lable_width, 0.0);
      glVertex3d(lable_x_left+lable_length, lable_y_up, 0.0);
    }
    glEnd();

    qglColor(QColor(0, 0, 0));
    renderText(text_x, text_y, str, QFont("AnyStyle", 6));
  };

  Char_t str_buff[256] = { 0 };
  for (Int32_t i = 0;
       i < module_info_.module_status_list.module_status_num;
       ++i) {
    const ad_msg::ModuleStatus& module =
        module_info_.module_status_list.module_status_list[i];

    switch (module.sub_module_id) {
    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_HDMAP): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "HDMap: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "HDMap: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "HDMap: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_ROUTING): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "Route: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Route: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "Route: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_GNSS): {
      Int32_t gnss_status = module_info_.gnss.utm_status;
#if (HD_MAP_TYPE == HD_MAP_TYPE_APOLLO)
      gnss_status = module_info_.gnss.utm_status;
#elif (HD_MAP_TYPE == HD_MAP_TYPE_D17)
      gnss_status = module_info_.gnss.odom_status;
#else
      gnss_status = module_info_.gnss.gnss_status;
#endif
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "GNSS: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "GNSS: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else if (ad_msg::Gnss::STATUS_GOOD != gnss_status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "GNSS: Bad Signal(%d),[%dms]"
                      , gnss_status, module.param[0]);
        qglColor(background_warn);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "GNSS: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_IMU): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "IMU: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "IMU: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1, 
                     "IMU: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_CHASSIS): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "Chassis: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Chassis: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Chassis: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_LANE_MARK_CAMERA_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                      "CAM Lane: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAM Lane: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAM Lane: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_CAMERA_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAM Obj: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAM Obj: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAM Obj: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD F: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD F: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD F: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_LEFT_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD FL: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD FL: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD FL: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_FRONT_RIGHT_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD FR: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD FR: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD FR: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_LEFT_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD RL: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD RL: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD RL: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_RADAR_REAR_RIGHT_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD RR: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD RR: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "RAD RR: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_LIDAR_LIST_0): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Lidar_0: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Lidar_0: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Lidar_0: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OBSTACLE_LIDAR_LIST_1): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Lidar_1: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Lidar_1: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Lidar_1: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_RECV_MSG_OUTSIDE_OBSTACLE_LIST): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "OBJ: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "OBJ: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "OBJ: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (ad_msg::SUB_MODULE_ID_MOTION_PLANNING_DO_WORK): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Planning: Timeout,[%dms]", module.param[1]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Planning: Err(%d),[%dms]",
                     module.status, module.param[1]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "Planning: OK,[%dms]", module.param[1]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    default:
      break;
    }
  }
}

void WidgetMap::DrawChassisInfo() {
  Int32_t w = width();
  Int32_t h = height();

  QColor background_normal(82, 235, 233, 200);
  QColor background_warn(255, 255, 0, 200);
  QColor background_err(255, 0, 0, 200);

  qglColor(QColor(0, 100, 255));

  // Driving Status
  switch (module_info_.chassis_info.driving_mode) {
  case (ad_msg::VEH_DRIVING_MODE_MANUAL):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "M");
    break;
  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "A");
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "U");
    break;
  }
  renderText(10, 40, string_buffer_, QFont("AnyStyle", 30));
  // 方向盘控制状态
  switch (module_info_.chassis_info.eps_status) {
  case (ad_msg::VEH_EPS_STATUS_MANUAL):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "转向: 不受控");
    qglColor(background_warn);
    break;
  case (ad_msg::VEH_EPS_STATUS_ROBOTIC):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "转向: 受控");
    qglColor(background_normal);
    break;
  case (ad_msg::VEH_EPS_STATUS_MANUAL_INTERRUPT):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "转向: 人工介入");
    qglColor(background_warn);
    break;
  case (ad_msg::VEH_EPS_STATUS_ERROR):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "转向: 故障");
    qglColor(background_err);
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "转向: 未知");
    qglColor(background_warn);
    break;
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(10, height()-114, 0.0);
    glVertex3d(10, height()-114-19, 0.0);
    glVertex3d(10+100, height()-114-19, 0.0);
    glVertex3d(10+100, height()-114, 0.0);
  }
  glEnd();
  qglColor(QColor(0, 100, 255));
  renderText(10, 130, string_buffer_, QFont("AnyStyle", 12));

  // 油门系统控制状态
  switch (module_info_.chassis_info.throttle_sys_status) {
  case (ad_msg::VEH_THROTTLE_SYS_STATUS_MANUAL):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "油门: 不受控");
    qglColor(background_warn);
    break;
  case (ad_msg::VEH_THROTTLE_SYS_STATUS_ROBOTIC):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "油门: 受控");
    qglColor(background_normal);
    break;
  case (ad_msg::VEH_THROTTLE_SYS_STATUS_ERROR):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "油门: 故障");
    qglColor(background_err);
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "油门: 未知");
    qglColor(background_warn);
    break;
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(10, height()-114-20, 0.0);
    glVertex3d(10, height()-114-20-19, 0.0);
    glVertex3d(10+100, height()-114-20-19, 0.0);
    glVertex3d(10+100, height()-114-20, 0.0);
  }
  glEnd();
  qglColor(QColor(0, 100, 255));
  renderText(10, 150, string_buffer_, QFont("AnyStyle", 12));

  // 制动系统控制状态
  switch (module_info_.chassis_info.ebs_status) {
  case (ad_msg::VEH_EBS_STATUS_MANUAL):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "制动: 不受控");
    qglColor(background_warn);
    break;
  case (ad_msg::VEH_EBS_STATUS_ROBOTIC):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "制动: 受控");
    qglColor(background_normal);
    break;
  case (ad_msg::VEH_EBS_STATUS_ERROR):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "制动: 故障");
    qglColor(background_err);
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "制动: 未知");
    qglColor(background_warn);
    break;
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(10, height()-114-20-20, 0.0);
    glVertex3d(10, height()-114-20-20-19, 0.0);
    glVertex3d(10+100, height()-114-20-20-19, 0.0);
    glVertex3d(10+100, height()-114-20-20, 0.0);
  }
  glEnd();
  qglColor(QColor(0, 100, 255));
  renderText(10, 170, string_buffer_, QFont("AnyStyle", 12));

  qglColor(QColor(0, 100, 255));

  // 当前档位
  switch (module_info_.chassis_info.gear) {
  case (ad_msg::VEH_GEAR_P):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "P(%d)",
                 module_info_.chassis_info.gear_number);
    break;
  case (ad_msg::VEH_GEAR_N):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "N(%d)",
                 module_info_.chassis_info.gear_number);
    break;
  case (ad_msg::VEH_GEAR_R):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "R(%d)",
                 module_info_.chassis_info.gear_number);
    break;
  case (ad_msg::VEH_GEAR_D):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "D(%d)",
                 module_info_.chassis_info.gear_number);
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "I");
  }
  renderText(w-96, 40, string_buffer_, QFont("AnyStyle", 30));

  // 当前车速
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "%0.1fkm/h", module_info_.chassis_info.v*3.6F);
  renderText(w/2-80, 40, string_buffer_, QFont("AnyStyle", 30));

  if (phoenix::planning::VELOCITY_PLANNING_TARGET_TYPE_PCC == module_info_.velocity_planning_result.tar_type) {
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "PACC MODE: ON");
    renderText(10, height()-90, string_buffer_, QFont("AnyStyle", 12));
  } else {
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "PACC MODE: OFF");
    renderText(10, height()-90, string_buffer_, QFont("AnyStyle", 12));
  }

  if (module_info_.velocity_planning_result.release_throttle) {
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "Release Throttle: ON");
    renderText(10, height()-65, string_buffer_, QFont("AnyStyle", 12));
  } else {
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "Release Throttle: OFF");
    renderText(10, height()-65, string_buffer_, QFont("AnyStyle", 12));
  }

  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "CAM 车道宽: %0.1f",
                module_info_.lane_info_camera_list.left_width +
                module_info_.lane_info_camera_list.right_width);
  renderText(10, height()-40, string_buffer_, QFont("AnyStyle", 12));

  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "leading: %0.1f", module_info_.trajectory_planning_info.leading_length_for_lka);
  renderText(10, height()-15, string_buffer_, QFont("AnyStyle", 12));

#if 0
  switch (module_info_.trajectory_planning_info.road_builed_type) {
  case (planning::ROAD_BUILED_TYPE_HD_MAP):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "目标轨迹：HDMap");
    break;
  case (planning::ROAD_BUILED_TYPE_CAMERA_LANE):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "目标轨迹：识别的车道线");
    break;
  case (planning::ROAD_BUILED_TYPE_FOLLOWING):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "目标轨迹：跟随前车");
    break;
  case (planning::ROAD_BUILED_TYPE_PRED_PATH):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "目标轨迹：不受控");
    break;
  case (planning::ROAD_BUILED_TYPE_MIXED_HD_MAP):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "目标轨迹：Mixed HDMap");
    break;
  default:
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "目标轨迹：内部异常");
    break;
  }
#else
  switch (module_info_.driving_map_info.driving_map_type) {
  case (driv_map::DRIVING_MAP_TYPE_HD_MAP):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "驾驶地图：HDMap");
    break;
  case (driv_map::DRIVING_MAP_TYPE_CAMERA_LANE):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "驾驶地图：识别的车道线");
    break;
  case (driv_map::DRIVING_MAP_TYPE_FOLLOWING_PATH):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "驾驶地图：跟随前车");
    break;
  case (driv_map::DRIVING_MAP_TYPE_PRED_PATH):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "驾驶地图：不受控");
    break;
  case (driv_map::DRIVING_MAP_TYPE_MIXED_HD_MAP):
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "驾驶地图：Mixed HDMap");
    break;
  default:
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "驾驶地图：内部异常");
    break;
  }
#endif
  renderText(width()/2-50, height()-15, string_buffer_, QFont("AnyStyle", 12));


  if (!module_info_.trajectory_planning_result.target_trajectory_slope.Empty()) {
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                  "Slope = %0.1f",
                  module_info_.trajectory_planning_result.target_trajectory_slope.Front().slope);
    renderText(width()/2-30, height()-80, string_buffer_, QFont("AnyStyle", 14));
  }

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "SMTO: x=%0.3f, y=%0.3f, h=%0.2f",
                module_info_.filtered_gnss_info.x_odom,
                module_info_.filtered_gnss_info.y_odom,
                common::com_rad2deg(module_info_.filtered_gnss_info.heading_odom));
  renderText(width()/2-150, height()-55, string_buffer_, QFont("AnyStyle", 12));
#else
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "GPS: lat=%0.6f, lon=%0.6f, h=%0.2f",
                module_info_.gnss.latitude,
                module_info_.gnss.longitude,
                common::com_rad2deg(module_info_.gnss.heading_gnss));
  renderText(width()/2-150, height()-55, string_buffer_, QFont("AnyStyle", 12));
#endif
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "ODOM: x=%0.3f, y=%0.3f, h=%0.2f",
                module_info_.gnss.x_odom,
                module_info_.gnss.y_odom,
                common::com_rad2deg(module_info_.gnss.heading_odom));
  renderText(width()/2-150, height()-35, string_buffer_, QFont("AnyStyle", 12));
#else
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "UTM: x=%0.3f, y=%0.3f, h=%0.2f",
                module_info_.gnss.x_utm,
                module_info_.gnss.y_utm,
                common::com_rad2deg(module_info_.gnss.heading_utm));
  renderText(width()/2-150, height()-35, string_buffer_, QFont("AnyStyle", 12));
#endif
}

void WidgetMap::DrawSceneStory() {
  // 地理围栏
  const common::StaticVector<ad_msg::SceneStory,
      ad_msg::SceneStoryList::MAX_SCENE_STORYS_NUM>& story_list =
      module_info_.driving_map_info.scene_storys;

  bool in_tunnel = false;
  bool in_curve = false;
  bool passing_ramp = false;
  bool in_ramp = false;
  for (Int32_t i = 0; i < story_list.Size(); ++i) {
    const ad_msg::SceneStory& story = story_list[i];

    // 隧道
    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_TUNNEL == story.type) {
      if (story.area.DistanceToArea() < 20.0F) {
        in_tunnel = true;
      }
    }
    // 弯道半径 < 500m
    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_CURVE == story.type) {
      if (story.area.DistanceToArea() < 20.0F) {
        in_curve = true;
      }
    }
    // 路过匝道
    if (ad_msg::SCENE_STORY_TYPE_PASSING_RAMP == story.type) {
      if (story.area.DistanceToArea() < 20.0F) {
        passing_ramp = true;
      }
    }
    // 经过匝道
    if (ad_msg::SCENE_STORY_TYPE_CLOSE_TO_RAMP == story.type) {
      if (story.area.DistanceToArea() < 20.0F) {
        in_ramp = true;
      }
    }
  }

  Int32_t x_offset = 80;
  if (in_curve) {
    common::AABBox2d area;
    area.min().set_x(10+x_offset);
    area.min().set_y(60);
    area.max().set_x(50+x_offset);
    area.max().set_y(100);
    DrawIcon(texture_id_of_icon_curve_, area);
  }

  x_offset += 50;
  if (in_tunnel) {
    common::AABBox2d area;
    area.min().set_x(10+x_offset);
    area.min().set_y(60);
    area.max().set_x(50+x_offset);
    area.max().set_y(100);
    DrawIcon(texture_id_of_icon_tunnel_, area);
  }

  x_offset += 50;
  if (passing_ramp) {
    common::AABBox2d area;
    area.min().set_x(10+x_offset);
    area.min().set_y(60);
    area.max().set_x(50+x_offset);
    area.max().set_y(100);
    DrawIcon(texture_id_of_icon_passing_ramp_, area);
  }

  x_offset += 50;
  if (in_ramp) {
    common::AABBox2d area;
    area.min().set_x(10+x_offset);
    area.min().set_y(60);
    area.max().set_x(50+x_offset);
    area.max().set_y(100);
    DrawIcon(texture_id_of_icon_ramp_, area);
  }
}


} // hmi
} // phoenix

