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
#include "uisetting_manager.h"

namespace phoenix {
namespace hmi {

// 障碍物数据打印LOG宏定义
#define PROINTF_OBJECT_INFO_FLAG_MAXIEYE (1)
#define PROINTF_OBJECT_INFO_FLAG_VISUALCONTROL (0)
#define PROINTF_OBJECT_INFO_FLAG_430RADAR (0)
#define PROINTF_OBJECT_INFO_FLAG_ANNGICRADAR (0)
#define PROINTF_OBJECT_INFO_FLAG_LIDAR (0)
#define PROINTF_OBJECT_INFO_FLAG_FUSION (1)


#define LONG_EMPTY_STR "                                                                         "

/**
 * @class DrawConcavePolygon
 * @brief 画凹多边形
 *
 * @par Note:
 * @code
 *     通常可以采用一种叫做"分格化"的方法来画复杂的多边形。
 *     非凸多边形最简单的填充方法最简单的应该是GLU 网格化对象GLUtesselator
 *     (GLUT库或者libTess库)。要用分格化的方法画多边形，步骤如下：
 *     1. gluNewTess(); //创建一个新的分格化对象
 *     2. gluTessCallback(); //注册回调函数，完成分格化的一些操作，照着写就行了。
 *     3. gluTessProperty(); //设置一些分格化的属性值，如环绕数和环绕规则，
 *                             用来确定多边形的内部和外部
 *     4. gluTessBeginPolygon(); //开始画多边形
 *        gluTessBeginContour(tessobj);//设置多边形的边线 1
 *        gluTessEndContour(tessobj);//结束设置边线1
 *        gluTessBeginContour(tessobj);//，如果有边线2，设置多边形的边线 2
 *        gluTessEndContour(tessobj);//结束设置边线2
 *     5. gluTessEdnPolygon(); //结束画多边形
 *     6. gluDeleteTess(); //删除分格化对象
 *     当然也可以利用回调函数记录分格化的顶点和绘制类型，
 *     然后利用数组的绘制函数进行一次性的绘制，以提高绘制效率。
 * @endcode
 */
enum { MAX_CONCAVE_POLYGON_POINT_NUM = 30720 };
GLdouble s_concave_polygon_points_buff[MAX_CONCAVE_POLYGON_POINT_NUM][3];
class DrawConcavePolygon {
public:
  DrawConcavePolygon() {
    point_index_ = 0;
    points_buff_ = &s_concave_polygon_points_buff[0];
  }

  ~DrawConcavePolygon() {}

  void Paint() {
    tess_obj_ = gluNewTess();
    gluTessCallback(tess_obj_, GLU_TESS_BEGIN, (_GLUfuncptr)&PolyLine3DBegin);
    gluTessCallback(tess_obj_, GLU_TESS_VERTEX, (_GLUfuncptr)&PolyLine3DVertex);
    gluTessCallback(tess_obj_, GLU_TESS_END, (_GLUfuncptr)&PolyLine3DEnd);
    gluTessCallback(tess_obj_, GLU_TESS_ERROR, (_GLUfuncptr)&HandleErr);

    gluTessBeginPolygon(tess_obj_, NULL);
    gluTessBeginContour(tess_obj_);
  }

  void DrawVertex(GLdouble x, GLdouble y, GLdouble z = 0) {
    points_buff_[point_index_][0] = x;
    points_buff_[point_index_][1] = y;
    points_buff_[point_index_][2] = z;

    if (IsSamePoint(point_index_)) {
      return;
    }

    gluTessVertex(tess_obj_,
                  points_buff_[point_index_],
                  points_buff_[point_index_]);

    point_index_++;
  }

  void EndPaint() {
    gluTessEndContour(tess_obj_);
    gluTessEndPolygon(tess_obj_);
    gluDeleteTess(tess_obj_);
  }

private:
  bool IsSamePoint(Int32_t index) {
    if (index > 0) {
      GLdouble dx = fabs(points_buff_[index][0]-points_buff_[index-1][0]);
      GLdouble dy = fabs(points_buff_[index][1]-points_buff_[index-1][1]);
      if (dx < 1e-5 && dy < 1e-5) {
        return true;
      }
    }

    return false;
  }

  static void PolyLine3DBegin(GLenum type) {
    glBegin(type);
  }

  static void PolyLine3DVertex(GLdouble* vertex) {
    const GLdouble *pointer = (GLdouble*) vertex;
    glVertex3dv(pointer);
  }

  static void PolyLine3DEnd() {
    glEnd();
  }

  static void HandleErr(GLenum error_code) {
    const GLubyte * estring;
    // 打印错误类型
    estring = gluErrorString(error_code);
    // std::cout << "Tessellation Error: " << estring << std::endl;
    LOG_ERR << "Tessellation Error: " << (Char_t*)estring;
  }

private:
  GLUtesselator* tess_obj_;
  Int32_t point_index_;
  GLdouble (*points_buff_)[3];
};


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

  view_switch_button_.min().set_x(20);
  view_switch_button_.min().set_y(80);
  view_switch_button_.max().set_x(60);
  view_switch_button_.max().set_y(120);
  vew_mode_ = VIEW_MODE_2;
}

WidgetMap::~WidgetMap() {

}

void WidgetMap::LoadIcon() {
  texture_id_of_icon_aerial_view_ = 0;
  texture_id_of_icon_3d_view_ = 0;

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
  // std::cout << "### After generating texture for 3d view icon, texture_id="
  //           << texture_id_of_icon_3d_view_
  //           << ", err_code=" << gl_err
  //           << ", and err_str=\"" << gl_err_str << "\"."
  //           << " ###" << std::endl;
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
  // 激活二维纹理贴图
  glEnable(GL_TEXTURE_2D);
  // 设置纹理贴图方式(直接覆盖原来颜色或与原来颜色混合)
  // 直接使用纹理覆盖模式
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
  // 指定当前使用的纹理(第一次使用和第二次使用含义不同)
  glBindTexture(GL_TEXTURE_2D, texture_id);

  // std::cout << "#### texture_id=" << texture_id << "####" << std::endl;

  //清除背景颜色干扰
  glColor3ub(255,255,255);
  // 为每个顶点指定纹理坐标，类似指定法线一样glNormal()
  glBegin(GL_QUADS);
  {
    //glTexCoord2d(0.3, 0.1);
    glTexCoord2d(0.0, 0.0);
    glVertex2d(view_switch_button_.min().x(),
               height() - view_switch_button_.min().y());

    //glTexCoord2d(0.8, 0.1);
    glTexCoord2d(1.0, 0.0);
    glVertex2d(view_switch_button_.max().x(),
               height() - view_switch_button_.min().y());

    //glTexCoord2d(0.8, 0.7);
    glTexCoord2d(1.0, 1.0);
    glVertex2d(view_switch_button_.max().x(),
               height() - view_switch_button_.max().y());

    //glTexCoord2d(0.3, 0.7);
    glTexCoord2d(0.0, 1.0);
    glVertex2d(view_switch_button_.min().x(),
               height() - view_switch_button_.max().y());
  }
  glEnd();
  // 关闭二维纹理贴图
  glDisable(GL_TEXTURE_2D);
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

  // 以下绘制的信息：原点为自车后轴中心，向前为x，向左为y，向上为z
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

  // 以下绘制的信息：原点为左下角，向右为x，向上为y
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  DrawViewSwitchButton();

  DrawChassisInfo();
  DrawStatusInfo();
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
  for(Int32_t i = 0; i <= 15; i++) {
    glVertex3d(i*10, -10.0-i*2, 0.2);
    glVertex3d(i*10, 10.0+i*2, 0.2);
  }
  glEnd();
  renderText(0.0, 10.0, 0.2, "0m");
  renderText(10.0, 12.0, 0.2, "10m");
  renderText(20.0, 14.0, 0.2, "20m");
  renderText(30.0, 16.0, 0.2, "30m");
  renderText(40.0, 18.0, 0.2, "40m");
  renderText(50.0, 20.0, 0.2, "50m");
  renderText(60.0, 22.0, 0.2, "60m");
  renderText(70.0, 24.0, 0.2, "70m");
  renderText(80.0, 26.0, 0.2, "80m");
  renderText(90.0, 28.0, 0.2, "90m");
  renderText(100.0, 30.0, 0.2, "100m");
  renderText(110.0, 32.0, 0.2, "110m");
  renderText(120.0, 34.0, 0.2, "120m");
  renderText(130.0, 36.0, 0.2, "130m");
  renderText(140.0, 38.0, 0.2, "140m");
  renderText(150.0, 40.0, 0.2, "150m");

  qglColor(QColor(0, 0, 255, 100));
  glLineWidth(1.0f);
  glBegin(GL_LINES);
  for(Int32_t i = 1; i <= 7; i++) {
    glVertex3d(-i*10, -10.0-i*2, 0.2);
    glVertex3d(-i*10, 10.0+i*2, 0.2);
  }
  glEnd();
  renderText(-10.0, 12.0, 0.2, "-10m");
  renderText(-20.0, 14.0, 0.2, "-20m");
  renderText(-30.0, 16.0, 0.2, "-30m");
  renderText(-40.0, 18.0, 0.2, "-40m");
  renderText(-50.0, 20.0, 0.2, "-50m");
  renderText(-60.0, 22.0, 0.2, "-60m");
  renderText(-70.0, 24.0, 0.2, "-70m");

  // 绘制前视一体机识别范围
  // qglColor(QColor(0, 0, 255, 100));
  // glLineWidth(1.0f);
  // glBegin(GL_LINES);
  // {
  //   Float64_t ang = common::com_deg2rad(28.5);
  //   Float64_t len = 100.0;
  //   glVertex3d(0.0, 0.0, 0.2);
  //   glVertex3d(len*common::com_cos(ang), len*common::com_sin(ang), 0.2);

  //   ang = common::com_deg2rad(-28.5);
  //   len = 100.0;
  //   glVertex3d(0.0, 0.0, 0.2);
  //   glVertex3d(len*common::com_cos(ang), len*common::com_sin(ang), 0.2);
  // }
  // glEnd();

  // 绘制弧线
  // DrawEllipse_Camera(0.0, 0.0, 10, 10, common::com_deg2rad(28.5), common::com_deg2rad(331.5), 0.0, 0.0, 360);
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

// 画叉
void WidgetMap::DrawCross(Float64_t x, Float64_t y, Float64_t len) {
  phoenix::common::Vec2d begin1;
  phoenix::common::Vec2d begin2;
  phoenix::common::Vec2d end1;
  phoenix::common::Vec2d end2;

  len = 0.5*len;

  begin1(0) = x + len*common::com_cos(phoenix::common::NormalizeAngle(COM_PI*0.75));
  begin1(1) = y + len*common::com_sin(phoenix::common::NormalizeAngle(COM_PI*0.75));

  begin2(0) = x + len*common::com_cos(phoenix::common::NormalizeAngle(-COM_PI*0.75));
  begin2(1) = y + len*common::com_sin(phoenix::common::NormalizeAngle(-COM_PI*0.75));

  end1(0) = x + len*common::com_cos(phoenix::common::NormalizeAngle(-COM_PI*0.25));
  end1(1) = y + len*common::com_sin(phoenix::common::NormalizeAngle(-COM_PI*0.25));

  end2(0) = x + len*common::com_cos(phoenix::common::NormalizeAngle(COM_PI*0.25));
  end2(1) = y + len*common::com_sin(phoenix::common::NormalizeAngle(COM_PI*0.25));

  glLineWidth(5.0);
  glBegin(GL_LINES);
  
  {
    DrawVertex(begin1(0), begin1(1), 0);
    DrawVertex(end1(0), end1(1), 0);

    DrawVertex(begin2(0), begin2(1), 0);
    DrawVertex(end2(0), end2(1), 0);
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

void WidgetMap::DrawEllipse_Camera(Float64_t x, Float64_t y, Float64_t a, Float64_t b, Float64_t l_end, Float64_t r_end, 
                            Float64_t heading, Float64_t z, Int32_t count) {
  Float64_t delta_theta = 2.0*COM_PI / count;

  Float64_t cos_heading = common::com_cos(heading);
  Float64_t sin_heading = common::com_sin(heading);

  glBegin(GL_LINE_STRIP);
  for (Int32_t i = 0; i < count; ++i) {
    Float64_t theta = i * delta_theta;  // theta为弧度
    if (theta < l_end || theta > r_end) {
      Float64_t px = a * common::com_cos(theta);
      Float64_t py = b * common::com_sin(theta);

      Float64_t ppx = x + px * cos_heading - py * sin_heading;
      Float64_t ppy = y + px * sin_heading + py * cos_heading;

      DrawVertex(ppx, ppy, z);
    }
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
  /*
  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  const Float64_t half_veh_width = 1.25;
  const Float64_t half_veh_length = 3.0;
  const Float64_t veh_height = 2.0;
  */

  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  veh_model::VehicleModelWrapper veh_model;
  const Float64_t half_veh_width = 0.5F * veh_model.GetVehicleWidth();
  const Float64_t half_veh_length = 0.5F * veh_model.GetVehicleLength();
  const Float64_t veh_height = 2.0;
  const Float64_t dist_of_localization_to_center =
      veh_model.GetDistOfLocalizationToCenter();
#endif

  obb.set_unit_direction(phoenix::common::CosLookUp(heading),
                         phoenix::common::SinLookUp(heading));
  //obb.set_center(pos + 0.5 * obb.unit_direction_x());
  /**
   * @brief 
   * 车辆中心原点为车辆中心：obb.set_center(pos + 0 * obb.unit_direction_x());
   * 将车辆中心原点偏移到后轴中心：obb.set_center(pos + dist_of_localization_to_center * obb.unit_direction_x());
   */
  obb.set_center(pos + dist_of_localization_to_center * obb.unit_direction_x());
  obb.set_extents(half_veh_length, half_veh_width);

  glLineWidth(1.0);

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

  glColor3d(0.0, 0.5, 1.0);
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
  phoenix::framework::SharedData* shared_data =
      phoenix::framework::SharedData::instance();

  shared_data->GetChassis(&module_info_.chassis_info);
  shared_data->GetGnssData(&module_info_.gnss_info_);
  shared_data->GetImuData(&module_info_.imu_info_);
  shared_data->GetLaneMarkCameraList(&module_info_.lane_mark_camera_list);
  shared_data->GetLaneCurbCameraList(&module_info_.lane_curb_camera_list);
  shared_data->GetObstacleCameraList(&module_info_.obstacle_camera_list);
  shared_data->GetObstacleEsrFrontList(&module_info_.obstacle_esr_front_list);
  shared_data->GetObstacleMaxieyeCameraList(&module_info_.obstacle_maxieye_camera_list);
  shared_data->GetObstaclelidarFrontList(&module_info_.obstacle_lidar_list_front);
  shared_data->GetVisualControlLaneMarkCameraList(&module_info_.visual_control_lane_mark_camera_list);
  shared_data->GetObstacleAnngicRadarFrontLeftList(&module_info_.obstacle_anngic_radar_front_left_list);
  shared_data->GetObstacleAnngicRadarFrontRightList(&module_info_.obstacle_anngic_radar_front_right_list);
  shared_data->GetObstacleAnngicRadarRearLeftList(&module_info_.obstacle_anngic_radar_rear_left_list);
  shared_data->GetObstacleAnngicRadarRearRightList(&module_info_.obstacle_anngic_radar_rear_right_list);
  shared_data->GetObstacleVisualControlFrontList(&module_info_.obstacle_visual_control_front_list);
  shared_data->GetObstacleVisualControlFrontLeftList(&module_info_.obstacle_visual_control_front_left_list);
  shared_data->GetObstacleVisualControlFrontRightList(&module_info_.obstacle_visual_control_front_right_list);
  shared_data->GetObstacleVisualControlRearLeftList(&module_info_.obstacle_visual_control_rear_left_list);
  shared_data->GetObstacleVisualControlRearRightList(&module_info_.obstacle_visual_control_rear_right_list);
  shared_data->GetObstacleVisualControlRearList(&module_info_.obstacle_visual_control_rear_list);
  shared_data->GetObstaclesFusionList(&module_info_.obstacle_fusion_list);
  shared_data->GetADHMIObstaclesFusionList(&module_info_.adhmi_obstacle_fusion_list);

  shared_data->GetModuleStatusList(&module_info_.module_status_list);
  shared_data->GetADHMIClock(&module_info_.adhmi_clock);
  TimeStampToLocalTime();
}

void WidgetMap::DrawMap() {
  width_value = width();
  height_value = height();
  line_number = 0;
  tab_width = 400;

  UISettingManager * setting_manager = UISettingManager::instance();

  if (setting_manager->GetSettingMainLaneState())

  if(setting_manager->GetSettingMainLaneState()){DrawLaneMarkCameraList();}
  if(setting_manager->GetSettingLaneCurbState()){DrawLaneCurbCameraList();}
  if(setting_manager->GetSettingMainCameraState()){DrawObstacleCameraList();}
  if(setting_manager->GetSettingFrontRadarState()){DrawObstacleEsrFrontList();}
  if(setting_manager->GetSettingMainCameraState()){DrawObstacleMaxieyeCameraList();}
  if(setting_manager->GetSettingMainLidarState()){DrawObstacleLidarListFront();}
  if(setting_manager->GetSettingLeftFrontRadarState()){DrawObstacleAnngicRadarFrontLeftList();}
  if(setting_manager->GetSettingRightFrontRadarState()){DrawObstacleAnngicRadarFrontRightList();}
  if(setting_manager->GetSettingLeftBackRadarState()){DrawObstacleAnngicRadarRearLeftList();}
  if(setting_manager->GetSettingRightBackRadarState()){DrawObstacleAnngicRadarRearRightList();}
  if(setting_manager->GetSettingVCLaneState()){DrawVisualControlLaneMarkCameraList();}
  if(setting_manager->GetSettingVCFrontState()){DrawObstacleVisualControlFrontList();}
  if(setting_manager->GetSettingVCLeftFrontState()){DrawObstacleVisualControlFrontLeftList();}
  if(setting_manager->GetSettingVCRightFrontState()){DrawObstacleVisualControlFrontRightList();}
  if(setting_manager->GetSettingVCLeftBackState()){DrawObstacleVisualControlRearLeftList();}
  if(setting_manager->GetSettingVCRightBackState()){DrawObstacleVisualControlRearRightList();}
  DrawObstacleVisualControlRearList();
  if(setting_manager->GetSettingPCFusionState()){DrawObstacleFusionList();}
  if(setting_manager->GetSettingADECUFusionState()){DrawADHMIObstacleFusionList();}

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
      qglColor(QColor(255, 255, 255));
    } else {
      qglColor(QColor(220, 220, 220));
    }

    //Add the dash and solid visualization
    int j_interval = 1;
    if(lane_mark.lane_mark_type == ad_msg::LaneMarkCamera::LaneMarkType::LANE_MARK_TYPE_DASHED) {
        glPointSize(5.0f);
        glBegin(GL_POINTS);   //TODO： it is the temporary type for dash lane visualization. it need be changed to dash line instead of dash points.
        j_interval = 2;

    }else{
        glBegin(GL_LINE_STRIP);
    }

    for (Int32_t j = 0; j < sample_size; j= j + j_interval) {
       Float32_t x = j * sample_step_len;
       Float32_t y = static_cast<Float32_t>(curve.Evaluate(0, x));

       DrawVertex(x, y, 0.1);
     }

    glEnd();
  }
}

void WidgetMap::DrawLaneCurbCameraList() {
  const ad_msg::LaneMarkCameraList& lane_curb_list =
      module_info_.lane_curb_camera_list;
  const Float32_t sample_step_len = 0.8F;

  glColor3d(1.0, 1.0, 1.0);
  glLineWidth(3.0f);

  Int32_t lane_curb_num = lane_curb_list.lane_mark_num;
  for (Int32_t i = 0; i < lane_curb_num; ++i) {
    const ad_msg::LaneMarkCamera& lane_mark =
        lane_curb_list.lane_marks[i];
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

    // if ((1 == lane_mark.id) || (-1 == lane_mark.id)) {
    //   qglColor(QColor(255, 255, 255));
    // } else {
    //   qglColor(QColor(220, 220, 220));
    // }

    // qglColor(QColor(255, 255, 0));
    // qglColor(QColor(255, 0, 255));
    qglColor(QColor(84, 255, 159));

    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < sample_size; ++j) {
      Float32_t x = j * sample_step_len;
      Float32_t y = static_cast<Float32_t>(curve.Evaluate(0, x));
      DrawCross(x, y, 0.6);
    }
    glEnd();
  }
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
    // common::OBBox2d obb;
    // obb.set_unit_direction(1.0F, 0.0F);
    // obb.set_center(obstacle.x+half_length, obstacle.y);
    // obb.set_extents(half_length, half_width);
    // DrawOBB_2D(obb, 0.0);

#if 0
    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
#else
    Float32_t half_length = length * 0.5F;
    Float32_t heading = obstacle.heading;
    common::Vec2d pos(obstacle.x, obstacle.y);
    common::OBBox2d obb;

    obb.set_unit_direction(common::com_cos(heading), common::com_sin(heading));
    obb.set_center(pos + half_length * obb.unit_direction_x());
    obb.set_extents(half_length, half_width);
    DrawOBB_2D(obb, 1.0F);
#endif

    //    glBegin(GL_LINES);
    //    {
    //      DrawVertex(0.0, 0.0, 0.2);
    //      DrawVertex(obstacle.x, obstacle.y, 0.2);
    //    }
    //    glEnd();

    //    com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
    //                 "id(%d) vx(%0.1f)",
    //                 obstacle.id,
    //                 (obstacle.rel_v_x+module_info_.chassis_info.v)*3.6);
    //    renderText(obstacle.x-view_param_.origin_coor.x,
    //               obstacle.y-view_param_.origin_coor.y,
    //               0.0 , string_buffer_, QFont("System", 10));
  }
}

void WidgetMap::DrawObstacleEsrFrontList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_esr_front_list;
  const ad_msg::LaneMarkCameraList& lane_mark_camera_list_ = 
      module_info_.lane_mark_camera_list;

  glColor3d(0.6, 0.6, 0.0);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if(obstacle.Obj_ucObstacleProbability > 65) {
      glColor3d(0.6, 0.6, 0.0);
      DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                  obstacle.x+length, obstacle.y+half_width),
                  0.0F);
      qglColor(QColor(255, 0, 0));
      DrawCross(obstacle.x, obstacle.y, 0.6);
    } else {
      glColor3d(0.6, 0.6, 0.0);
      DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                  obstacle.x+length, obstacle.y+half_width),
                  0.0F);
    }

    if (UISettingManager::instance()->GetSettingFrontRadarTxtState())
    {
      // com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
      //               "Esr_id(%d),x(%.1f),y(%.1f),vx(%.2f),vy(%.2f),DC(%d),POE(%d),OP(%d)", \
      //               obstacle.id, \
      //               obstacle.x, \
      //               obstacle.y, \
      //               obstacle.v_x * 3.6, \
      //               obstacle.v_y * 3.6, \
      //               module_info_.chassis_info.v * 3.6, \
      //               obstacle.uiDynConfidence, \
      //               obstacle.uiProbabilityOfExistence, \
      //               obstacle.Obj_ucObstacleProbability);
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "Esr_id(%d),x(%.1f),y(%.1f),length(%.1f), width(%.1f), vx(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.v_x * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }
  }
}

void WidgetMap::DrawObstacleMaxieyeCameraList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_maxieye_camera_list;

  glColor3d(0.0, 0.6, 0.6);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingMainCameraTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "M_Cam_id(%d)t(%d)x(%.1f)y(%.1f)length(%.1f)width(%.1f)height(%.1f)vx(%.2f)vy(%.2f)age(%d)lane(%d)", \
                    obstacle.id, \
                    obstacle.type, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.height, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6, \
                    obstacle.age, \
                    obstacle.lane);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }
    if (obstacle.type == ad_msg::OBJ_TYPE_TRAFFIC_CONE)
    {
      glColor3d(1.0, 0.4, 1.0); 
    }
    else
    {
      glColor3d(0.0, 0.6, 0.6);
    }
     

#if 1
    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
#else
    Float32_t half_length = length * 0.5F;
    Float32_t heading = obstacle.heading;
    common::Vec2d pos(obstacle.x, obstacle.y);
    common::OBBox2d obb;

    obb.set_unit_direction(common::com_cos(heading), common::com_sin(heading));
    obb.set_center(pos + half_length * obb.unit_direction_x());
    obb.set_extents(half_length, half_width);
    DrawOBB_2D(obb, 1.0F);
#endif
  }
}

// 激光雷达为青色的框
void WidgetMap::DrawObstacleLidarListFront() {
  const ad_msg::ObstacleLidarList& obstacle_list =
      module_info_.obstacle_lidar_list_front;

  qglColor(QColor(0, 255, 255));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleLidar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = obstacle.obb.half_length * 2;
    Float32_t half_width = obstacle.obb.half_width;

    if (UISettingManager::instance()->GetSettingMainLidarState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "x(%.1f),y(%.1f),width(%.1f),length(%.2f)", \
                    obstacle.obb.x, \
                    obstacle.obb.y, \
                    obstacle.obb.half_width * 2, \
                    obstacle.obb.half_length * 2);
      renderText(obstacle.obb.x,obstacle.obb.y, 0, string_buffer_);
    }

#if 1
    DrawAABB_2D(common::AABBox2d(obstacle.obb.x, obstacle.obb.y-half_width,
                                 obstacle.obb.x+length, obstacle.obb.y+half_width),
                1.0F);
#else              
    common::OBBox2d obb;
    printf("x:%f, y:%f\n", obstacle.x, obstacle.y);
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_extents(obstacle.obb.half_length, obstacle.obb.half_width);

    DrawOBB_2D(obb, 1.0F);
#endif
  }
}

// 侧向毫米波为蓝色的点
void WidgetMap::DrawObstacleAnngicRadarFrontLeftList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_anngic_radar_front_left_list;

  //qglColor(QColor(0, 0, 255));
  qglColor(QColor(0, 64, 0));   //框的颜色
  glLineWidth(2.0f);  //框的线条宽度

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingLeftFrontRadarTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "R_FL_id(%d),x(%.1f),y(%.1f),length(%.1f),width(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }  
}

void WidgetMap::DrawObstacleAnngicRadarFrontRightList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_anngic_radar_front_right_list;

  //qglColor(QColor(0, 0, 255));
  qglColor(QColor(0, 128, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingRightFrontRadarTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "R_FR_id(%d),x(%.1f),y(%.1f),length(%.1f),width(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

void WidgetMap::DrawObstacleAnngicRadarRearLeftList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_anngic_radar_rear_left_list;

  //qglColor(QColor(0, 0, 255));
  qglColor(QColor(0, 192, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingLeftBackRadarTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "R_RL_id(%d),x(%.1f),y(%.1f),length(%.1f),width(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

void WidgetMap::DrawObstacleAnngicRadarRearRightList() {
  const ad_msg::ObstacleRadarList& obstacle_list =
      module_info_.obstacle_anngic_radar_rear_right_list;

  //qglColor(QColor(0, 0, 255));
  qglColor(QColor(0, 255, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleRadar& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingRightBackRadarTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "R_RR_id(%d),x(%.1f),y(%.1f),length(%.1f),width(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

// 环视车道线(本车道为黄色，其他车道为深黄)
void WidgetMap::DrawVisualControlLaneMarkCameraList() {
  const ad_msg::LaneMarkCameraList& lane_mark_list =
      module_info_.visual_control_lane_mark_camera_list;
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
      qglColor(QColor(230, 230, 0));
    } else {
      qglColor(QColor(200, 200, 0));
    }

    glBegin(GL_LINE_STRIP);
    for (Int32_t j = 0; j < sample_size; ++j) {
      Float32_t x = j * sample_step_len;
      Float32_t y = static_cast<Float32_t>(curve.Evaluate(0, x));

      DrawVertex(x, y, 0.1);
    }
    glEnd();
  }
}

// 前相机为青色
void WidgetMap::DrawObstacleVisualControlFrontList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_visual_control_front_list;

  //qglColor(QColor(32, 0, 0));
  qglColor(QColor(0, 255, 255));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingVCFrontTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "VCF_id(%d),x(%.1f),y(%.1f),vx(%.2f),vy(%.2f))", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

// 前左相机为紫色
void WidgetMap::DrawObstacleVisualControlFrontLeftList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_visual_control_front_left_list;

  //qglColor(QColor(64, 0, 0));
  qglColor(QColor(160, 32, 240));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingVCLeftFrontTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "VCFL_id(%d),x(%.1f),y(%.1f),type(%d),length(%.1f),width(%.1f),height(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.type, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.height, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

// 前右相机为淡紫
void WidgetMap::DrawObstacleVisualControlFrontRightList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_visual_control_front_right_list;

  //qglColor(QColor(96, 0, 0));
  qglColor(QColor(218, 112, 214));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingVCRightFrontTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "VCFR_id(%d),x(%.1f),y(%.1f),type(%d),length(%.1f),width(%.1f),height(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.type, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.height, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

// 后左相机为蓝色
void WidgetMap::DrawObstacleVisualControlRearLeftList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_visual_control_rear_left_list;

  //qglColor(QColor(128, 0, 0));
  qglColor(QColor(65, 105, 255));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingVCLeftBackTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "VCRL_id(%d),x(%.1f),y(%.1f),type(%d),length(%.1f),width(%.1f),height(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.type, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.height, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

// 后右相机为橙色
void WidgetMap::DrawObstacleVisualControlRearRightList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_visual_control_rear_right_list;

  //qglColor(QColor(160, 0, 0));
  qglColor(QColor(255, 97, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    if (UISettingManager::instance()->GetSettingVCRightBackTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "VCRR_id(%d),x(%.1f),y(%.1f),type(%d),length(%.1f),width(%.1f),height(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.type, \
                    obstacle.length, \
                    obstacle.width, \
                    obstacle.height, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

void WidgetMap::DrawObstacleVisualControlRearList() {
  const ad_msg::ObstacleCameraList& obstacle_list =
      module_info_.obstacle_visual_control_rear_list;

  qglColor(QColor(192, 0, 0));
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::ObstacleCamera& obstacle = obstacle_list.obstacles[i];

    Float32_t length = common::Max(obstacle.length, 0.5F);
    Float32_t half_width = 0.5F * common::Max(obstacle.width, 0.5F);

    /*
    if (UISettingManager::instance()->GetSettingVCFrontTxtState())
    {
      com_snprintf(string_buffer_, sizeof(string_buffer_)-1, \
                    "VCR_id(%d),x(%.1f),y(%.1f),vx(%.2f),vy(%.2f)", \
                    obstacle.id, \
                    obstacle.x, \
                    obstacle.y, \
                    obstacle.v_x * 3.6, \
                    obstacle.v_y * 3.6);
      renderText(obstacle.x,obstacle.y, 0, string_buffer_);
    }
    */

    DrawAABB_2D(common::AABBox2d(obstacle.x, obstacle.y-half_width,
                                 obstacle.x+length, obstacle.y+half_width),
                1.0F);
  }
}

//黄色的框
void WidgetMap::DrawObstacleFusionList() {
  veh_model::VehicleModelWrapper veh_model;
  const ad_msg::ObstacleList& obstacle_list =
      module_info_.obstacle_fusion_list;

  //qglColor(QColor(200, 255, 0));
  glColor3b(1.0, 0.0, 1.0);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  Int32_t in_lane_num = 0;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::Obstacle& obstacle = obstacle_list.obstacles[i];

    //printf("confidence:%d\n", obstacle.confidence);
    if (obstacle.confidence < 60) {
      qglColor(QColor(255,153,18, 128));
    } else {
        qglColor(QColor(255,153,18));
        //qglColor(QColor(255,153,18));

      /*if (obstacle.dynamic) {
        qglColor(QColor(255, 0, 255));
      } else {
        glColor3b(0.0, 0.0, 1.0);
        qglColor(QColor(200, 255, 0));
      }*/
    }

    if (UISettingManager::instance()->GetSettingPCFusionTxtState())
    {
      int confidence_limit = 0;//60
      // if (obstacle.dynamic) {
      if ((obstacle.confidence >= confidence_limit) /*&&*/
          /*((obstacle.x-veh_model.GetDistOfLocalizationToFront()) > 2.0F) &&*/
            /*(common::com_abs(obstacle.y) < 5.0F) && (obstacle.dynamic)*/)
        {
          Int8_t radar_type, camera_type, exist_lidar;
          Uint8_t radar_id, camera_id, lidar_id;
          bool ret =  ad_msg::Obstacle::DecodeFusionID(obstacle.id, radar_type, camera_type, exist_lidar,radar_id, camera_id, lidar_id);
        in_lane_num++;
        line_number++;

#if 1
        com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                     "FS%X",
                     obstacle.id);
        renderText(obstacle.x, obstacle.y, 0, string_buffer_, QFont("AnyStyle", 10));


        com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE - 1, "ID[%X-%dr%d-%dc%d][t:%2d,pt:%d,c:%3d](x:%.1f,y:%.1f,xv:%.1f,yv:%.1f)",
                     obstacle.id,radar_type,radar_id,camera_type,camera_id, obstacle.type, obstacle.perception_type, obstacle.confidence,
                     obstacle.x, obstacle.y, obstacle.v_x * 3.6, obstacle.v_y * 3.6);
        
        // renderText(width_value - tab_width-70, height_value - line_number * 13, LONG_EMPTY_STR, QFont("AnyStyle", 10));
        renderText(width_value - tab_width-70, height_value - line_number * 13, string_buffer_, QFont("AnyStyle", 10));
        
#else        
        com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                     "x(%0.1f)y(%0.1f)length(%0.1f)width(%0.1f)height(%0.1f)vx(%0.1f)vy(%0.1f)ID[%X-%dr%d-%dc%d]",
                     // obstacle.x - veh_model.GetDistOfLocalizationToFront(),
                     obstacle.x,
                     obstacle.y,
                     obstacle.obb.half_length * 2,
                     obstacle.obb.half_width * 2,
                     obstacle.height,
                     obstacle.v_x * 3.6,
                     obstacle.v_y * 3.6,
                     obstacle.id,
                     radar_type,
                     radar_id,
                     camera_type,
                     camera_id);
        // renderText(obstacle.obb.x,
        //            obstacle.obb.y,
        //            0.0, LONG_EMPTY_STR, QFont("System", 10));
        renderText(obstacle.obb.x,
                   obstacle.obb.y,
                   0.0, string_buffer_, QFont("System", 10));

        com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                     "t(%d),dy(%d),cf(%d),pt(%d)",
                     obstacle.type,
                     obstacle.dynamic,
                     obstacle.confidence,
                     obstacle.perception_type);
        renderText(obstacle.obb.x - 1,
                   obstacle.obb.y,
                   0.0, string_buffer_, QFont("System", 10));
#endif 
      }
    }

    common::OBBox2d obb;
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_extents(obstacle.obb.half_length, obstacle.obb.half_width);

    DrawOBB_2D(obb, 1.0F);
  }

  if (UISettingManager::instance()->GetSettingPCFusionTxtState())
  {
    qglColor(QColor(255,153,18));
    line_number += 1;
    com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                 "Sensing FON: %d, ILN: %d",
                 obstacle_num, in_lane_num);
    renderText(width_value - tab_width - 70, height_value - line_number * 13, string_buffer_,  QFont("AnyStyle", 15));
    line_number++;
  }
}

void WidgetMap::DrawADHMIObstacleFusionList() {
  veh_model::VehicleModelWrapper veh_model;
  const ad_msg::ObstacleList& obstacle_list =
      module_info_.adhmi_obstacle_fusion_list;

  //qglColor(QColor(200, 255, 0));
  glColor3b(1.0, 0.0, 1.0);
  glLineWidth(2.0f);

  Int32_t obstacle_num = obstacle_list.obstacle_num;
  Int32_t in_lane_num = 0;
  for (Int32_t i = 0; i < obstacle_num; ++i) {
    const ad_msg::Obstacle& obstacle = obstacle_list.obstacles[i];

    //printf("confidence:%d\n", obstacle.confidence);
    if (obstacle.confidence < 60) {
      //qglColor(QColor(128, 128, 128, 128));
      qglColor(QColor(128, 128, 0));
    } else {
        qglColor(QColor(0, 0, 255));

      /*if (obstacle.dynamic) {
        qglColor(QColor(255, 0, 255));
      } else {
        glColor3b(0.0, 0.0, 1.0);
        qglColor(QColor(200, 255, 0));
      }*/
    }

    if (UISettingManager::instance()->GetSettingADECUFusionTxtState())
    {
           int confidence_limit = 0;//60
      // if (obstacle.dynamic) {
      if ((obstacle.confidence >= confidence_limit) /*&&*/
          /*((obstacle.x-veh_model.GetDistOfLocalizationToFront()) > 2.0F) &&*/
            /*(common::com_abs(obstacle.y) < 5.0F) && (obstacle.dynamic)*/)
        {
          Int8_t radar_type, camera_type, exist_lidar;
          Uint8_t radar_id, camera_id, lidar_id;
          bool ret =  ad_msg::Obstacle::DecodeFusionID(obstacle.id, radar_type, camera_type, exist_lidar,radar_id, camera_id, lidar_id);

        in_lane_num++;
        line_number++;

#if 1
        com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                     "FA%X",
                     obstacle.id);
        renderText(obstacle.x, obstacle.y, 0, string_buffer_, QFont("AnyStyle", 10));


        com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE - 1, "ID[%X-%dr%d-%dc%d][t:%2d,pt:%d,c:%3d](x:%.1f,y:%.1f,xv:%.1f,yv:%.1f)",
                     obstacle.id,radar_type,radar_id,camera_type,camera_id, obstacle.type, obstacle.perception_type, obstacle.confidence,
                     obstacle.x, obstacle.y, obstacle.v_x * 3.6, obstacle.v_y * 3.6);
        
        // renderText(width_value - tab_width-70, height_value - line_number * 13, LONG_EMPTY_STR, QFont("AnyStyle", 10));
        renderText(width_value - tab_width-70, height_value - line_number * 13, string_buffer_, QFont("AnyStyle", 10));
        
#else        
        com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                     "x(%0.1f)y(%0.1f)length(%0.1f)width(%0.1f)height(%0.1f)vx(%0.1f)vy(%0.1f)ID[%X-%dr%d-%dc%d]",
                     // obstacle.x - veh_model.GetDistOfLocalizationToFront(),
                     obstacle.x,
                     obstacle.y,
                     obstacle.obb.half_length * 2,
                     obstacle.obb.half_width * 2,
                     obstacle.height,
                     obstacle.v_x * 3.6,
                     obstacle.v_y * 3.6,
                     obstacle.id,
                     radar_type,
                     radar_id,
                     camera_type,
                     camera_id);
        // renderText(obstacle.obb.x,
        //            obstacle.obb.y,
        //            0.0, LONG_EMPTY_STR, QFont("System", 10));
        renderText(obstacle.obb.x,
                   obstacle.obb.y,
                   0.0, string_buffer_, QFont("System", 10));

        com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                     "t(%d),dy(%d),cf(%d),pt(%d)",
                     obstacle.type,
                     obstacle.dynamic,
                     obstacle.confidence,
                     obstacle.perception_type);
        renderText(obstacle.obb.x - 1,
                   obstacle.obb.y,
                   0.0, string_buffer_, QFont("System", 10));
#endif 
      }
    }


    common::OBBox2d obb;
    obb.set_center(obstacle.obb.x, obstacle.obb.y);
    obb.set_unit_direction(obstacle.obb.heading);
    obb.set_extents(obstacle.obb.half_length, obstacle.obb.half_width);

    DrawOBB_2D(obb, 1.0F);
  }

  if (UISettingManager::instance()->GetSettingADECUFusionTxtState())
  {
    qglColor(QColor(0, 0, 255));
    line_number += 1;
    com_snprintf(string_buffer_, sizeof(string_buffer_) - 1,
                 "ADECU FON: %d, ILN: %d",
                 obstacle_num, in_lane_num);
    renderText(width_value - tab_width-70, height_value - line_number * 13, string_buffer_,  QFont("AnyStyle", 15));
    line_number++;
  }
}

void WidgetMap::DrawChassisInfo() {
  Int32_t w = width();
  Int32_t h = height();

  qglColor(QColor(0, 100, 255));

  //  // Driving Status
  //  switch (module_info_.chassis_info.driving_mode) {
  //  case (ad_msg::VEH_DRIVING_MODE_MANUAL):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Manual");
  //    break;
  //  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC_FULLY):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Auto");
  //    break;
  //  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC_ONLY_LATERAL):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Auto_lat");
  //    break;
  //  case (ad_msg::VEH_DRIVING_MODE_ROBOTIC_ONLY_LONGITUDINAL):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Auto_lon");
  //    break;
  //  default:
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Unknown");
  //    break;
  //  }
  //  renderText(10, 40, string_buffer_, QFont("AnyStyle", 30));

  //  // 当前档位
  //  switch (module_info_.chassis_info.gear) {
  //  case (ad_msg::VEH_GEAR_P):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "P");
  //    break;
  //  case (ad_msg::VEH_GEAR_N):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "N");
  //    break;
  //  case (ad_msg::VEH_GEAR_R):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "R");
  //    break;
  //  case (ad_msg::VEH_GEAR_D):
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "D");
  //    break;
  //  default:
  //    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "I");
  //  }
  //  renderText(w-40, 40, string_buffer_, QFont("AnyStyle", 30));

  // 当前车速
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
              "%0.1fkm/h", module_info_.chassis_info.v*3.6F);
  renderText(w/2-80, 40, string_buffer_, QFont("AnyStyle", 30));

  //com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Lat:  %f",
  //             module_info_.gnss_info_.latitude);
  //renderText(10, 20, string_buffer_, QFont("AnyStyle", 10));
  //com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "Lon: %f",
  //             module_info_.gnss_info_.longitude);
  //renderText(10, 40, string_buffer_, QFont("AnyStyle", 10));
  //com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "yaw_rate: %f",
  //             common::com_rad2deg(module_info_.imu_info_.yaw_rate));
  //renderText(10, 60, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "left_lane: %d, quality:%d",
               module_info_.lane_mark_camera_list.lane_marks[0].lane_mark_type, module_info_.lane_mark_camera_list.lane_marks[0].quality);
  renderText(10, 20, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "right_lane: %d, quality:%d",
               module_info_.lane_mark_camera_list.lane_marks[1].lane_mark_type, module_info_.lane_mark_camera_list.lane_marks[1].quality);
  renderText(10, 40, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "left_left_lane: %d, quality:%d",
               module_info_.lane_mark_camera_list.lane_marks[2].lane_mark_type, module_info_.lane_mark_camera_list.lane_marks[2].quality);
  renderText(10, 60, string_buffer_, QFont("AnyStyle", 10));
  com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "right_right_lane: %d, quality:%d",
               module_info_.lane_mark_camera_list.lane_marks[3].lane_mark_type, module_info_.lane_mark_camera_list.lane_marks[3].quality);
  renderText(10, 80, string_buffer_, QFont("AnyStyle", 10));
  //com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "pitch_rate: %f",
  //             module_info_.imu_info_.pitch_rate);
  //renderText(10, 80, string_buffer_, QFont("AnyStyle", 10));
  //com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "roll_rate: %f",
  //             module_info_.imu_info_.roll_rate);
  //renderText(10, 100, string_buffer_, QFont("AnyStyle", 10));
  
  renderText(10, h-15, local_time, QFont("AnyStyle", 12));
}


void WidgetMap::DrawStatusInfo() {
  QColor background_normal(82, 235, 233, 200);
  QColor background_warn(255, 255, 0, 200);
  QColor background_err(255, 0, 0, 200);
  QColor background_timeout(255, 128, 128, 200);

  Int32_t draw_counter = 0;
  auto func_draw_lable = [this, &draw_counter](const Char_t* str) {
    Float64_t lable_width = 20;
    Float64_t lable_length = 100;
    Float64_t lable_x_left = 10.0;
    Float64_t lable_y_up = height() - 200.0 - draw_counter*25.0 + 70;
    Int32_t text_x = 12;
    Int32_t text_y = 215 + draw_counter*25 - 70;
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

  char str_buff[256] = { 0 };
  for (Int32_t i = 0; i < module_info_.module_status_list.module_status_num; ++i) {
    const ad_msg::ModuleStatus& module =
        module_info_.module_status_list.module_status_list[i];
    switch (module.sub_module_id) {

    /*
    case (framework::INTERNAL_MODULE_ID_MSG_RECV_GNSS): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "GNSS: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "GNSS: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else if (ad_msg::Gnss::STATUS_GOOD !=
                 module_info_.gnss_info_.gnss_status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "GNSS: Bad Signal(%d),[%dms]"
                      , module_info_.gnss_info_.gnss_status, module.param[0]);
        qglColor(background_warn);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "GNSS: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_IMU): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "IMU: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "IMU: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "IMU: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;
    */

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_LANE): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Mob_Lane: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Mob_Lane: Err(%d),[%dms]",
                      module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "Mob_Lane: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_LANE_CURB): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Lane_Curb: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Lane_Curb: Err(%d),[%dms]",
                      module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "Lane_Curb: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (phoenix::framework::INTERNAL_MODULE_ID_MSG_RECV_MOBILEYE_OBJ): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Mob_Obj: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Mob_Obj: Err(%d),[%dms]",
                      module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "Mob_Obj: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (phoenix::framework::INTERNAL_MODULE_ID_MSG_RECV_ESR_FRONT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "ESR_F: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "ESR_F: Err(%d),[%dms]",
                      module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "ESR_F: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_CHASSIS): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "CHASSIS: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "CHASSIS: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "CHASSIS: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_MAXIEYE_CAMERA): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "Maxieye_Camera: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Maxieye_Camera: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "Maxieye_Camera: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_LIDAR_FRONT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "Lidar_Front: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "Lidar_Front: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "Lidar_Front: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_FRONT_LEFT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "AR_FL: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "AR_FL: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "AR_FL: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_FRONT_RIGHT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "AR_FR: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "AR_FR: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "AR_FR: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_REAR_LEFT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "AR_RL: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "AR_RL: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "AR_RL: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_ANNGIC_RADAR_REAR_RIGHT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "AR_RR: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "AR_RR: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "AR_RR: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_LANE): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_Lane: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_Lane: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_Lane: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_F: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_F: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_F: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT_LEFT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_FL: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_FL: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_FL: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_FRONT_RIGHT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_FR: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_FR: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_FR: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR_LEFT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_RL: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_RL: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_RL: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR_RIGHT): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_RR: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_RR: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_RR: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_VISUAL_CONTROL_REAR): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "VC_R: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "VC_R: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "VC_R: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_FUSION_OBJ): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "FUSION: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "FUSION: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "FUSION: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (framework::INTERNAL_MODULE_ID_MSG_RECV_MPU_STATE): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "MPU_STATE: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "MPU_STATE: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "MPU_STATE: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;
    case (framework::INTERNAL_MODULE_ID_MSG_RECV_ADHMI_FUSION_OBJ): {
      if (module.timeout) {
        std::snprintf(str_buff, sizeof(str_buff)-1, "ADHMI FUSION: Timeout,[%dms]",
                      module.param[0]);
        qglColor(background_timeout);
      } else if (ad_msg::MODULE_STATUS_OK != module.status) {
        std::snprintf(str_buff, sizeof(str_buff)-1,
                      "ADHMI FUSION: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        std::snprintf(str_buff, sizeof(str_buff)-1
                      , "ADHMI FUSION: OK,[%dms]", module.param[0]);
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

// 将时间戳转换为时间
void WidgetMap::TimeStampToLocalTime() {
  time_t now = module_info_.adhmi_clock.clock.sec;
  tm* p = localtime(&now);
  sprintf(local_time, "%04d/%02d/%02d %02d:%02d:%02d", p->tm_year+1900, p->tm_mon+1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
  //printf("%04d:%02d:%02d %02d:%02d:%02d.\n", p->tm_year+1900, p->tm_mon+1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
  //printf("local_time:%s\n", local_time);
}

} // hmi
} // phoenix

