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


namespace phoenix {
namespace hmi {


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

  view_switch_button_.min().set_x(10);
  view_switch_button_.min().set_y(60);
  view_switch_button_.max().set_x(50);
  view_switch_button_.max().set_y(100);
  vew_mode_ = VIEW_MODE_2;


  fuzzy_level_string_.push_back("右大");
  fuzzy_level_string_.push_back("右中");
  fuzzy_level_string_.push_back("右小");
  fuzzy_level_string_.push_back("右零");
  fuzzy_level_string_.push_back("左零");
  fuzzy_level_string_.push_back("左小");
  fuzzy_level_string_.push_back("左中");
  fuzzy_level_string_.push_back("左大");
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
    //glOrtho(-range, range, -range_mul_ratio, range_mul_ratio, -100.0, 100.0);
    glFrustum(-range, range, -range_mul_ratio, range_mul_ratio, 1.0, 2000.0);
  } else {
    Float64_t ratio = static_cast<Float64_t>(w) / static_cast<Float64_t>(h);
    Float64_t range_mul_ratio = range * ratio;
    //glOrtho(-range_mul_ratio, range_mul_ratio, -range, range, -100.0, 100.0);
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

  DrawVelocityPedal();
  DrawAccelerationPedal();
  DrawLongitudinalCtlValuePedal();
  DrawModuleStatusInfo();
  DrawChassisInfo();
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

void WidgetMap::DrawYawRatePath(
    Float64_t start_x, Float64_t start_y, Float64_t start_h,
    Float64_t curvature, Float64_t length, Float64_t z) {
  const Float64_t step_len = 0.5;
  Int32_t sample_size = length / step_len + 1;

  Float64_t sin_start_heading = std::sin(start_h);
  Float64_t cos_start_heading = std::cos(start_h);

  glBegin(GL_LINE_STRIP);
  if (std::abs(curvature) < 0.0001) {
    for (Int32_t i = 0; i < sample_size; ++i) {
      Float64_t delta = i * step_len;
      Float64_t x = start_x + delta * cos_start_heading;
      Float64_t y = start_y + delta * sin_start_heading;
      DrawVertex(x, y, z);
    }
  } else {
    Float64_t r = 1.0F / curvature;

    for (Int32_t i = 0; i < sample_size; ++i) {
      Float64_t delta = i * step_len;
      //tar_yaw_rate = tar_curvature * cur_v;
      Float64_t heading_changed = common::NormalizeAngle(
          start_h + curvature*delta);
      Float64_t sin_heading_changed = std::sin(heading_changed);
      Float64_t cos_heading_changed = std::cos(heading_changed);
      Float64_t x = start_x - r*sin_start_heading + r*sin_heading_changed;
      Float64_t y = start_y + r*cos_start_heading - r*cos_heading_changed;
      DrawVertex(x, y, z);
    }
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
  Float64_t heading = 0;
  phoenix::common::Vec2d pos(0, 0);
  phoenix::common::OBBox2d obb;

  const Float64_t half_veh_width = 1.25;
  const Float64_t half_veh_length = 3.0;
  const Float64_t veh_height = 2.0;
#endif

  obb.set_unit_direction(phoenix::common::CosLookUp(heading),
      phoenix::common::SinLookUp(heading));
  obb.set_center(pos + 0.5 * obb.unit_direction_x());
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

  //glColor3d(1.0, 1.0, 0.3);
  //DrawEllipse(pos.x(), pos.y(), 0.5, 0.5, heading, 0.0, 3);
}

void WidgetMap::UpdateModuleInfo() {
  Phoenix_SharedData_GetImu(&module_info_.imu);
  Phoenix_SharedData_GetChassis(&module_info_.chassis_info);
  Phoenix_SharedData_GetSpecialChassisInfo(&module_info_.special_chassis_info);
  Phoenix_SharedData_GetChassisCtlCmd(&module_info_.chassis_cmd);
  Phoenix_SharedData_GetPlanningResult(&module_info_.planning_result);
  Phoenix_SharedData_GetLateralControlInfo(&module_info_.lateral_control_info);
  Phoenix_SharedData_GetModuleStatusList(&module_info_.module_status_list);
}

void WidgetMap::DrawMap() {
  DrawTargetPath();
  DrawLateralCtlInfo();
}

void WidgetMap::DrawTargetPath() {
  const PlanningResult_t& planning_result = module_info_.planning_result;

  // start point
  // 坐标x, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
  Float32_t start_point_x = planning_result.tar_trj.curr_pos.x * 0.001F;
  // 坐标y, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
  Float32_t start_point_y = planning_result.tar_trj.curr_pos.y * 0.001F;
  // 航向角, PH = INT*0.00001 (rad), Range: (-3.14160, 3.14160 rad)
  Float32_t start_point_h = planning_result.tar_trj.curr_pos.h * 0.00001F;
  // 曲率, PH = INT*0.00001 (1/m), Range: [-1.00000, 1.00000]
  Float32_t start_point_c = planning_result.tar_trj.curr_pos.c * 0.00001F;
  // 相对于参考线的路径长, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
  Float32_t start_point_s = planning_result.tar_trj.curr_pos.s * 0.001F;
  // 相对于参考线的横向偏移, PH = INT*0.001 (m), Range: [-100.000, 100.000 m]
  Float32_t start_point_l = planning_result.tar_trj.curr_pos.l * 0.001F;

  // leading point
  // 坐标x, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
  Float32_t leading_point_x = planning_result.tar_trj.leading_pos.x * 0.001F;
  // 坐标y, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
  Float32_t leading_point_y = planning_result.tar_trj.leading_pos.y * 0.001F;
  // 航向角, PH = INT*0.00001 (rad), Range: (-3.14160, 3.14160 rad)
  Float32_t leading_point_h = planning_result.tar_trj.leading_pos.h * 0.00001F;
  // 曲率, PH = INT*0.00001 (1/m), Range: [-1.00000, 1.00000]
  Float32_t leading_point_c = planning_result.tar_trj.leading_pos.c * 0.00001F;
  // 相对于参考线的路径长, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
  Float32_t leading_point_s = planning_result.tar_trj.leading_pos.s * 0.001F;
  // 相对于参考线的横向偏移, PH = INT*0.001 (m), Range: [-100.000, 100.000 m]
  Float32_t leading_point_l = planning_result.tar_trj.leading_pos.l * 0.001F;

  // target trajectory
  std::vector<common::Vec2d> tar_path;
  Int32_t tar_trajectory_direction = planning_result.tar_trj.trj_direction;
  Int32_t tar_trajectory_points_num = planning_result.tar_trj.points_num;
  for (Int32_t index = 0; index < tar_trajectory_points_num; ++index) {
    // 坐标x, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
    Float32_t x = planning_result.tar_trj.points[index].x * 0.001F;
    // 坐标y, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
    Float32_t y = planning_result.tar_trj.points[index].y * 0.001F;
    // 航向角, PH = INT*0.00001 (rad), Range: (-3.14160, 3.14160 rad)
    Float32_t h = planning_result.tar_trj.points[index].h * 0.00001F;
    // 曲率, PH = INT*0.00001 (1/m), Range: [-1.00000, 1.00000]
    Float32_t c = planning_result.tar_trj.points[index].c * 0.00001F;
    // 相对于参考线的路径长, PH = INT*0.001 (m), Range: [-1000.000, 1000.000 m]
    Float32_t s = planning_result.tar_trj.points[index].s * 0.001F;

    tar_path.push_back(common::Vec2d(x, y));
    DrawVertex(x, y, 0.0);
  }

  glColor3d(1.0, 0.0, 0.0);
  glLineWidth(1.0f);
  glBegin(GL_LINE_STRIP);
  for (Int32_t i = 0; i < tar_path.size(); ++i) {
    DrawVertex(tar_path[i].x(), tar_path[i].y(), 0.0);
  }
  glEnd();
  glPointSize(3.0);
  glBegin(GL_POINTS);
  for (Int32_t i = 0; i < tar_path.size(); ++i) {
    DrawVertex(tar_path[i].x(), tar_path[i].y(), 0.0);
  }
  glEnd();


  glColor3d(1.0, 1.0, 0.0);
  glLineWidth(1.0f);
  //DrawEllipse(start_point_x, start_point_y, 0.3, 0.3, start_point_h, 0.0, 6);
  DrawEllipse(leading_point_x, leading_point_y, 0.3, 0.3, leading_point_h, 0.0, 6);
}



// 设置横向误差（左正右负）的等级（从负大到正大排列等级）
static const Float32_t s_lat_err_level[7] = {
/*       0            1          2        3         4          5           6        */
/*  0          1           2         3         4          5           6        7    */
/* 负大  ][    负中    ][   负小   ][   负零   ][   正零   ][   正小   ][    正中   ][ 正大  */
      -0.20F,      -0.10F,     -0.05F,      0.0F,      0.05F,     0.10F,       0.20F
};

// 设置横向误差变化速度（左正右负）的等级（从负大到正大排列等级）
static const Float32_t s_lat_err_v_level[7] = {
/*       0            1          2          3           4          5            6       */
/*   0         1           2           3          4           5           6        7    */
/* 负大  ][    负中    ][   负小   ][   负零   ][   正零   ][   正小   ][    正中   ][ 正大   */
       -0.10F,      -0.05F,    -0.02F,      0.0F,      0.02F,     0.05F,       0.10F
};

static Int32_t GetFuzzyLevelIndex(
    const Float32_t level_table[7], Float32_t value,
    Int32_t* membership_index, Float32_t* membership_grade) {
  Int32_t level_index = 0;
  Int32_t i = 0;
  Float32_t ratio = 0.0F;
  for (i = 0; i < 7; ++i) {
    if (value <= level_table[i]) {
      break;
    }
    level_index++;
  }

  if (level_index > 0) {
    ratio = (value - level_table[level_index-1]) /
        (level_table[level_index] - level_table[level_index-1]);
    if (ratio < 0.5F) {
      if (4 == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0F;
      } else {
        *membership_index = level_index - 1;
        *membership_grade = 0.5F - ratio;
      }
    } else {
      if (3 == level_index) {
        *membership_index = level_index;
        *membership_grade = 0.0F;
      } else {
        *membership_index = level_index + 1;
        *membership_grade = ratio - 0.5F;
      }
    }
    if (level_index >= 7) {
      *membership_index = level_index;
      *membership_grade = 0.0F;
    }
  } else {
    *membership_index = level_index;
    *membership_grade = 0.0F;
  }

  return (level_index);
}
void WidgetMap::DrawLateralCtlInfo() {
  const LateralControlInfo_t& ctl_info =
      module_info_.lateral_control_info;
  const LateralControlPidInfo_t& ctl_pid_info =
      ctl_info.lat_ctl_pid_info;

  glLineWidth(1.0f);

  glColor3d(0.0, 1.0, 0.0);
  glPointSize(3.0);
  glBegin(GL_POINTS);
  {
    DrawVertex(
          ctl_pid_info.trj_feed.cur_proj_on_trj.x,
          ctl_pid_info.trj_feed.cur_proj_on_trj.y, 0.0);
  }
  glEnd();
  DrawEllipse(
        ctl_info.curr_pos.x,
        ctl_info.curr_pos.y, 0.5, 0.5,
        ctl_info.curr_pos.heading, 0.0, 3);
  DrawArrow(
        ctl_info.curr_pos.x,
        ctl_info.curr_pos.y,
        ctl_info.curr_pos.heading, 2.0, 0.0);


  glColor3d(0.0, 1.0, 0.0);
  glLineWidth(1.0f);
  DrawEllipse(
        ctl_pid_info.trj_feed.goal_point_near.x,
        ctl_pid_info.trj_feed.goal_point_near.y, 0.5, 0.5,
        ctl_pid_info.trj_feed.goal_point_near.h, 0.0, 10);
  DrawArrow(
        ctl_pid_info.trj_feed.goal_point_near.x,
        ctl_pid_info.trj_feed.goal_point_near.y,
        ctl_pid_info.trj_feed.goal_point_near.h, 2.0, 0.0);
  glLineWidth(3.0f);
  DrawYawRatePath(
        ctl_info.curr_pos.x,
        ctl_info.curr_pos.y,
        ctl_info.curr_pos.heading,
        ctl_pid_info.trj_feed.feed_value_near,
        ctl_pid_info.trj_feed.goal_dist_near,
        0.3);

  Float32_t dist = ctl_pid_info.trj_feed.goal_dist;
  Float32_t delta_angle = ctl_pid_info.trj_feed.feed_value * dist;
  Float32_t pos[3] = { 0.0F };
  Float32_t next_pos[3] = { 0.0F };
  pos[0] = ctl_info.curr_pos.x;
  pos[1] = ctl_info.curr_pos.y;
  pos[2] = ctl_info.curr_pos.heading;
  if ((-0.001F < ctl_pid_info.trj_feed.feed_value) &&
      (ctl_pid_info.trj_feed.feed_value < 0.001F)) {
    next_pos[0] = pos[0] + common::com_cos(pos[2]) * dist;
    next_pos[1] = pos[1] + common::com_sin(pos[2]) * dist;
    next_pos[2] = common::NormalizeAngle(pos[2] + delta_angle);
  } else {
    Float32_t r = 1.0F / ctl_pid_info.trj_feed.feed_value;
    Float32_t heading_changed = common::NormalizeAngle(pos[2] + delta_angle);
    next_pos[0] = pos[0] +
        r * (-common::com_sin(pos[2]) + common::com_sin(heading_changed));
    next_pos[1] = pos[1] +
        r * ( common::com_cos(pos[2]) - common::com_cos(heading_changed));
    next_pos[2] = heading_changed;
  }
  glColor3d(1.0, 0.0, 0.0);
  glLineWidth(1.0f);
  DrawEllipse(
        next_pos[0],
        next_pos[1], 0.5, 0.5,
        next_pos[2], 0.0, 10);
  DrawArrow(
        next_pos[0],
        next_pos[1],
        next_pos[2], 2.0, 0.0);
  glLineWidth(3.0f);
  DrawYawRatePath(
        ctl_info.curr_pos.x,
        ctl_info.curr_pos.y,
        ctl_info.curr_pos.heading,
        ctl_pid_info.trj_feed.feed_value,
        ctl_pid_info.trj_feed.goal_dist,
        0.2);

  glColor3d(0.0, 0.0, 1.0);
  glLineWidth(1.0f);
  DrawEllipse(
        ctl_pid_info.trj_feed.goal_point_far.x,
        ctl_pid_info.trj_feed.goal_point_far.y, 0.5, 0.5,
        ctl_pid_info.trj_feed.goal_point_far.h, 0.0, 10);
  DrawArrow(
        ctl_pid_info.trj_feed.goal_point_far.x,
        ctl_pid_info.trj_feed.goal_point_far.y,
        ctl_pid_info.trj_feed.goal_point_far.h, 2.0, 0.0);
  glLineWidth(3.0f);
  DrawYawRatePath(
        ctl_info.curr_pos.x,
        ctl_info.curr_pos.y,
        ctl_info.curr_pos.heading,
        ctl_pid_info.trj_feed.feed_value,
        ctl_pid_info.trj_feed.goal_dist_far,
        0.1);

#if 0
  Int32_t lat_err_level_index = 0;
  Int32_t lat_err_membership_index = 0;
  Float32_t lat_err_membership_grade = 0.0F;
  Int32_t lat_err_v_level_index = 0;
  Int32_t lat_err_v_membership_index = 0;
  Float32_t lat_err_v_membership_grade = 0.0F;

  lat_err_level_index = GetFuzzyLevelIndex(
        s_lat_err_level, module_info_.lateral_control_info.lat_ctl_pid_info.lat_err,
        &lat_err_membership_index, &lat_err_membership_grade);
  lat_err_v_level_index = GetFuzzyLevelIndex(
        s_lat_err_v_level, module_info_.lateral_control_info.lat_ctl_pid_info.lat_err_v,
        &lat_err_v_membership_index, &lat_err_v_membership_grade);
#else
  Int32_t lat_err_level_index = module_info_.lateral_control_info.lat_ctl_pid_info.lat_err_feed.lat_err_lv_idx;
  Int32_t lat_err_v_level_index = module_info_.lateral_control_info.lat_ctl_pid_info.lat_err_feed.lat_err_spd_lv_idx;
#endif

  switch (lat_err_level_index) {
  case (0):
    glColor4d(1.0, 0.0, 0.0, 0.5);
    break;
  case (1):
    glColor4d(1.0, 1.0, 0.0, 0.5);
    break;
  case (2):
    glColor4d(0.0, 0.0, 1.0, 0.5);
    break;
  case (3):
    glColor4d(0.0, 1.0, 0.0, 0.5);
    break;
  case (4):
    glColor4d(0.0, 1.0, 0.0, 0.5);
    break;
  case (5):
    glColor4d(0.0, 0.0, 1.0, 0.5);
    break;
  case (6):
    glColor4d(1.0, 1.0, 0.0, 0.5);
    break;
  case (7):
    glColor4d(1.0, 0.0, 0.0, 0.5);
    break;
  default:
    glColor4d(1.0, 0.0, 0.0, 0.5);
    break;
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(0.0, 0.0, 1.0);
    glVertex3d(0.0, 4*ctl_pid_info.lat_err_feed.lat_err, 1.0);
    glVertex3d(1.0, 4*ctl_pid_info.lat_err_feed.lat_err, 1.0);
    glVertex3d(1.0, 0.0, 1.0);
  }
  glEnd();

  switch (lat_err_v_level_index) {
  case (0):
    glColor4d(1.0, 0.0, 0.0, 0.5);
    break;
  case (1):
    glColor4d(1.0, 1.0, 0.0, 0.5);
    break;
  case (2):
    glColor4d(0.0, 0.0, 1.0, 0.5);
    break;
  case (3):
    glColor4d(0.0, 1.0, 0.0, 0.5);
    break;
  case (4):
    glColor4d(0.0, 1.0, 0.0, 0.5);
    break;
  case (5):
    glColor4d(0.0, 0.0, 1.0, 0.5);
    break;
  case (6):
    glColor4d(1.0, 1.0, 0.0, 0.5);
    break;
  case (7):
    glColor4d(1.0, 0.0, 0.0, 0.5);
    break;
  default:
    glColor4d(1.0, 0.0, 0.0, 0.5);
    break;
  }
  glBegin(GL_QUADS);
  {
    glVertex3d(-1.0, 0.0, 1.0);
    glVertex3d(-1.0, 4*ctl_pid_info.lat_err_feed.lat_err_spd, 1.0);
    glVertex3d(0.0, 4*ctl_pid_info.lat_err_feed.lat_err_spd, 1.0);
    glVertex3d(0.0, 0.0, 1.0);
  }
  glEnd();

  glColor3d(0.0, 0.0, 1.0);
  glLineWidth(3.0);
  double lat_err_feedback =
      common::com_rad2deg(ctl_pid_info.lat_err_feed.feed_value);
  double lat_err_feedback_strength = common::com_abs(lat_err_feedback);
  if (lat_err_feedback_strength > 0.2) {
    if (ctl_pid_info.lat_err_feed.feed_value < 0.0F) {
      DrawArrow(0.0F, 0.0F, -COM_PI_2, lat_err_feedback_strength, 2.0);

      //com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      //    "lat_err_feed %0.1f", lat_err_feedback);
      //renderText(1.0, 0.0, 0.0, string_buffer_, QFont("AnyStyle", 12));
    } else {
      DrawArrow(0.0F, 0.0F, COM_PI_2, lat_err_feedback_strength, 2.0);

      //com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      //    "lat_err_feed %0.1f", lat_err_feedback);
      //renderText(1.0, 0.0, 0.0, string_buffer_, QFont("AnyStyle", 12));
    }
  }

#if 0
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
               "偏差(%0.0f)%s, 速率(%0.0f)%s, %s(%0.1f)",
               ctl_pid_info.lat_err*100, fuzzy_level_string_[lat_err_level_index].c_str(),
               ctl_pid_info.lat_err_v*100, fuzzy_level_string_[lat_err_v_level_index].c_str(),
               lat_err_feedback > 0 ? "左修正" : "右修正", lat_err_feedback);
  renderText(0.0, 0.0, 0.0, string_buffer_, QFont("AnyStyle", 12));
#endif
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

void WidgetMap::DrawVelocityPedal() {
  static std::vector<CirclePoint> steering_shape =
      GetVelocityPedalShape(90.0);

  Int32_t w = width();
  Int32_t h = height();
  Int32_t r = 90;
  Int32_t offset_x = w - 120;
  Int32_t offset_y = h - 160;//270;

  const Chassis_t& chassis_info = module_info_.chassis_info;
  const PlanningResult_t& planning_result = module_info_.planning_result;

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
  /// 速度请求, PH = INT * 0.1 (km/h), Range: [0.0, 200.0 km/h]
  Float64_t angle = phoenix::common::com_deg2rad(180.0+45.0) -
      phoenix::common::com_deg2rad(270.0)
      *((planning_result.tar_v) / max_velocity);
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
  /// 速度请求, PH = INT * 0.1 (km/h), Range: [0.0, 200.0 km/h]
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%03d -- "
      , static_cast<Int32_t>(common::com_round(
                               planning_result.tar_v*3.6)));
  renderText(offset_x-60, h-offset_y-10, string_buffer_, QFont("AnyStyle", 20));
  qglColor(QColor(255, 100, 0));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%03d",
      static_cast<Int32_t>(phoenix::common::com_round(chassis_info.v*3.6F)));
  renderText(offset_x+20, h-offset_y-10, string_buffer_, QFont("AnyStyle", 20));
}

void WidgetMap::DrawAccelerationPedal() {
  Int32_t w = width();
  Int32_t h = height();

  Int32_t offset_x = w - 40;
  Int32_t offset_y = h - 350;//430;

  Int32_t half_pedal_width = 15;
  Int32_t pedal_height = 80;

  const Chassis_t& chassis_info = module_info_.chassis_info;
  const PlanningResult_t& planning_result = module_info_.planning_result;

  /// 加速度请求, PH = INT * 0.01 (m/s^2), Range: [-15.00, 15.00 m/s^2]
  Int32_t target = common::com_round(
      pedal_height * planning_result.tar_a * 0.01 * 0.1);
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
  if (planning_result.tar_a >= 0) {
    qglColor(QColor(0, 100, 255));
  } else {
    qglColor(QColor(255, 0, 0));
  }
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x-half_pedal_width-25, offset_y+target, 0.0);
    glVertex3d(offset_x+half_pedal_width, offset_y+target, 0.0);
  }
  glEnd();
  /// 加速度请求, PH = INT * 0.01 (m/s^2), Range: [-15.00, 15.00 m/s^2]
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1, "%0.2f",
      planning_result.tar_a*0.01);
  renderText(offset_x-half_pedal_width-40, h-offset_y-target-5,
      string_buffer_, QFont("AnyStyle", 12));

  // current
  glLineWidth(1.0);
  if (chassis_info.a >= 0) {
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
  if (chassis_info.a >= 0) {
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

  Int32_t offset_x = w - 62;
  Int32_t offset_y = h - 585;//20;

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
    glVertex3d(offset_x, offset_y+pedal_value, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y+pedal_value, 0.0);
  }
  glEnd();
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%d", module_info_.chassis_info.brake_pedal_value);
  renderText(offset_x+pedal_width-28, h-offset_y-pedal_value+10,
      string_buffer_, QFont("AnyStyle", 10));

  // control value
  qglColor(QColor(255, 0, 0, 150));
  int ctl_value = module_info_.chassis_cmd.brake_value*0.01*pedal_height;
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
      "%0.1f", module_info_.chassis_cmd.brake_value);
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
  qglColor(QColor(0, 255, 0));
  pedal_value = module_info_.chassis_info.acc_pedal_value*0.01*pedal_height;
  glBegin(GL_LINES);
  {
    glVertex3d(offset_x, offset_y+pedal_value, 0.0);
    glVertex3d(offset_x+pedal_width, offset_y+pedal_value, 0.0);
  }
  glEnd();
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%d", module_info_.chassis_info.acc_pedal_value);
  renderText(offset_x+pedal_width-28, h-offset_y-pedal_value+10,
      string_buffer_, QFont("AnyStyle", 10));

  // control value
  qglColor(QColor(0, 255, 0, 150));
  ctl_value = module_info_.chassis_cmd.acc_value*0.01*pedal_height;
  glBegin(GL_QUADS);
  {
    glVertex3d(offset_x, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y, 0.0);
    glVertex3d(offset_x+pedal_width-1, offset_y+ctl_value, 0.0);
    glVertex3d(offset_x, offset_y+ctl_value, 0.0);
  }
  glEnd();
  qglColor(QColor(0, 255, 0));
  com_snprintf(string_buffer_, sizeof(string_buffer_)-1,
      "%0.1f", module_info_.chassis_cmd.acc_value);
  renderText(offset_x+pedal_width-28, h-offset_y-ctl_value-2,
      string_buffer_, QFont("AnyStyle", 10));
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

  //std::cout << "module_info_.module_status_list.module_status_num="
  //          << module_info_.module_status_list.module_status_num
  //          << std::endl;

  Char_t str_buff[256] = { 0 };
  for (Int32_t i = 0;
       i < module_info_.module_status_list.module_status_num; ++i) {
    const ModuleStatus_t& module =
        module_info_.module_status_list.module_status_list[i];

    switch (module.sub_module_id) {
    case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_0): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "CAN0: Timeout");
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAN0: Err(%d)", module.status);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "CAN0: OK,");
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1 || \
     VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
    case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_1): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "CAN1: Timeout");
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAN1: Err(%d)", module.status);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "CAN1: OK,");
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

    case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_2): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "CAN2: Timeout");
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAN2: Err(%d)", module.status);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "CAN2: OK,");
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;
#endif

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1 || \
     VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
    case (SUB_MODULE_ID_CONTROL_RECV_CAN_CHANNEL_3): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "CAN3: Timeout");
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "CAN3: Err(%d)", module.status);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "CAN3: OK,");
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;
#endif

    case (SUB_MODULE_ID_CONTROL_RECV_MSG_IMU): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "IMU: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "IMU: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "IMU: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;

#if ENTER_PLAYBACK_MODE
    case (SUB_MODULE_ID_CONTROL_RECV_MSG_CHASSIS): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1, "Chassis: Timeout,[%dms]",
                     module.param[0]);
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Chassis: Err(%d),[%dms]", module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "Chassis: OK,[%dms]", module.param[0]);
        qglColor(background_normal);
      }
      func_draw_lable(str_buff);
    }
      break;
#endif

    case (SUB_MODULE_ID_CONTROL_RECV_MSG_PLANNING_RESULT): {
      if (module.timeout) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Planning: Timeout,[%dms]", module.param[0]);
        qglColor(background_timeout);
      } else if (MODULE_STATUS_OK != module.status) {
        com_snprintf(str_buff, sizeof(str_buff)-1,
                     "Planning: Err(%d),[%dms]",
                     module.status, module.param[0]);
        qglColor(background_err);
      } else {
        com_snprintf(str_buff, sizeof(str_buff)-1
                     , "Planning: OK,[%dms]", module.param[0]);
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

  qglColor(QColor(0, 100, 255));

#if ENTER_PLAYBACK_MODE
  // Driving Status
  switch (module_info_.chassis_info.driving_mode) {
  case (VEH_DRIVING_MODE_MANUAL):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "M");
    break;
  case (VEH_DRIVING_MODE_ROBOTIC):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "A");
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "I");
    break;
  }
  renderText(10, 40, string_buffer_, QFont("AnyStyle", 30));
#endif

  // 当前档位
#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B1 || \
     VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  switch (module_info_.chassis_info.gear) {
  case (VEH_GEAR_P):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1,
                 "P(%d)", module_info_.chassis_info.gear_number);
    break;
  case (VEH_GEAR_N):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1,
                 "N(%d)", module_info_.chassis_info.gear_number);
    break;
  case (VEH_GEAR_R):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1,
                 "R(%d)", module_info_.chassis_info.gear_number);
    break;
  case (VEH_GEAR_D):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1,
                 "D(%d)", module_info_.chassis_info.gear_number);
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1,
                 "I(%d)", module_info_.chassis_info.gear_number);
  }
  renderText(w-100, 40, string_buffer_, QFont("AnyStyle", 30));
#else
  switch (module_info_.chassis_info.gear) {
  case (VEH_GEAR_P):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "P");
    break;
  case (VEH_GEAR_N):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "N");
    break;
  case (VEH_GEAR_R):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "R");
    break;
  case (VEH_GEAR_D):
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "D");
    break;
  default:
    com_snprintf(string_buffer_, MAX_STRING_BUFF_SIZE-1, "I");
  }
  renderText(w-40, 40, string_buffer_, QFont("AnyStyle", 30));
#endif

  // 当前车速
  std::snprintf(string_buffer_, sizeof(string_buffer_)-1,
                "%0.1fkm/h", module_info_.chassis_info.v*3.6F);
  renderText(w/2-80, 40, string_buffer_, QFont("AnyStyle", 30));

//  // EPB
//  switch (module_info_.chassis_info.epb_status) {
//  case (ad_msg::VEH_EPB_STATUS_OFF):
//    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,"EPB: OFF");
//    break;
//  case (ad_msg::VEH_EPB_STATUS_ON):
//    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,"EPB: ON");
//    break;
//  default:
//    std::snprintf(string_buffer_, sizeof(string_buffer_)-1,"EPB: I");
//    break;
//  }
//  renderText(10, 75, string_buffer_, QFont("AnyStyle", 30));

std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "W: %0.1f", 
              module_info_.chassis_info.gross_weight);
renderText(10, h-180, string_buffer_, QFont("AnyStyle", 12));

#if (VEHICLE_PLATFORM == VEHICLE_PLATFORM_DF_D17_B2)
  Int32_t veh_status_y = h - 130;
  Int32_t veh_status_y_interval = 20;

  // BCM状态   0x0:Not Ready, 0x1:Ready, 0x2:Engaged, 0x3:fault
  switch (module_info_.special_chassis_info.df_d17.BCM_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "BCM:            Not Ready");
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "BCM:            Ready");
    break;
  case (2):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "BCM:            Engaged");
    break;
  case (3):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "BCM:            Fault");
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "BCM:            Invalid");
    break;
  }
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));

  // VECU状态  0x0:Not Ready, 0x1:Ready, 0x2:Engaged, 0x3:fault
  switch (module_info_.special_chassis_info.df_d17.VECU_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "VECU:          Not Ready");
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "VECU:          Ready");
    break;
  case (2):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "VECU:          Engaged");
    break;
  case (3):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "VECU:          Fault");
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "VECU:          Invalid");
    break;
  }
  veh_status_y += veh_status_y_interval;
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));

  // 主转向状态   0:Not Ready / 1:Ready / 2:Reserve / 3:ADU engaged / 4:ADU engaged Degrade / 5:Error
  Uint16_t AHPSFailureCode = module_info_.special_chassis_info.df_d17.AHPSFailureCode;
  switch (module_info_.special_chassis_info.df_d17.EPS_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Not Ready (0x%04X)", AHPSFailureCode);
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Ready (0x%04X)", AHPSFailureCode);
    break;
  case (2):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Reserve (0x%04X)", AHPSFailureCode);
    break;
  case (3):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Engaged (0x%04X)", AHPSFailureCode);
    break;
  case (4):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Degrade (0x%04X)", AHPSFailureCode);
    break;
  case (5): {
    qglColor(QColor(255, 0, 0));
    switch (module_info_.special_chassis_info.df_d17.SteeringErrorEvent1PS) {
    case (1):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 1 - ADU PS故障. (0x%04X)", AHPSFailureCode);
      break;
    case (2):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 2 - AD Mode Enable PS丢失. (0x%04X)", AHPSFailureCode);
      break;
    case (3):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 3 - AD Mode Engage PS丢失. (0x%04X)", AHPSFailureCode);
      break;
    case (4):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 4 - SW Angel Request PS丢失. (0x%04X)", AHPSFailureCode);
      break;
    case (5):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 5 - SW Angel Request PS超出范围. (0x%04X)", AHPSFailureCode);
      break;
    case (6):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 6 - SW Angel Request PS斜率超出范围. (0x%04X)", AHPSFailureCode);
      break;
    case (7):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 7 - SW Angel Request PS信号无效. (0x%04X)", AHPSFailureCode);
      break;
    case (8):
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, 8 - SW Angel Request Valid PS丢失. (0x%04X)", AHPSFailureCode);
      break;
    default:
      std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向:        Error, %d - unknown reason. (0x%04X)", 
                    module_info_.special_chassis_info.df_d17.SteeringErrorEvent1PS, AHPSFailureCode);
      break;
    }
  }
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主转向: Invalid (0x%04X)", AHPSFailureCode);
    break;
  }
  veh_status_y += veh_status_y_interval;
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));

  // 冗余转向状态1  0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  switch (module_info_.special_chassis_info.df_d17.CEPS1_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Not Ready");
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Ready");
    break;
  case (2):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Standby");
    break;
  case (3):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Engaged");
    break;
  case (4):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Reserve");
    break;
  case (5):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Error");
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向1: Invalid");
    break;
  }
  veh_status_y += veh_status_y_interval;
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));

  // 冗余转向状态2  0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  switch (module_info_.special_chassis_info.df_d17.CEPS2_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Not Ready");
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Ready");
    break;
  case (2):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Standby");
    break;
  case (3):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Engaged");
    break;
  case (4):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Reserve");
    break;
  case (5):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Error");
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余转向2: Invalid");
    break;
  }
  veh_status_y += veh_status_y_interval;
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));

  // 主刹车状态   0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  switch (module_info_.special_chassis_info.df_d17.EBS_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Not Ready");
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Ready");
    break;
  case (2):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Standby");
    break;
  case (3):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Engaged");
    break;
  case (4):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Reserve");
    break;
  case (5):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Error");
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "主刹车:        Invalid");
    break;
  }
  veh_status_y += veh_status_y_interval;
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));

  // 冗余刹车状态  0:No Ready / 1:Ready / 2:Standby / 3:ADU engaged / 4:reserve / 5:Error
  switch (module_info_.special_chassis_info.df_d17.CEBS_state) {
  case (0):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Not Ready");
    break;
  case (1):
    qglColor(QColor(50, 200, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Ready");
    break;
  case (2):
    qglColor(QColor(0, 100, 255));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Standby");
    break;
  case (3):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Engaged");
    break;
  case (4):
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Reserve");
    break;
  case (5):
    qglColor(QColor(255, 0, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Error");
    break;
  default:
    qglColor(QColor(255, 100, 0));
    std::snprintf(string_buffer_, sizeof(string_buffer_)-1, "冗余刹车:   Invalid");
    break;
  }
  veh_status_y += veh_status_y_interval;
  renderText(10, veh_status_y, string_buffer_, QFont("AnyStyle", 12));
#endif
}


} // hmi
} // phoenix

