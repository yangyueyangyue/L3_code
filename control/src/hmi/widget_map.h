/******************************************************************************
 ** 图形显示窗口
 ******************************************************************************
 *
 *  图形显示窗口
 *
 *  @file       widget_map.h
 *
 *  @author     pengc
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018 Kotei Co., Ltd.
 ******************************************************************************/
#ifndef PHOENIX_WIDGET_MAP_H_
#define PHOENIX_WIDGET_MAP_H_

#include <vector>
#include "QtGui"
#include "QtOpenGL/QGLWidget"

#include "GL/gl.h"
#include "GL/glu.h"
#include "GL/glut.h"

#include "math/math_utils.h"
#include "math/matrix.h"
#include "geometry/geometry_utils.h"
#include "geometry/quaternion.h"
#include "geometry/aabbox2d.h"
#include "geometry/obbox2d.h"
#include "geometry/sphere2d.h"
#include "geometry/vec2d.h"
#include "geometry/line_segment2d.h"
#include "geometry/geometry_utils.h"
#include "vehicle_model_c.h"
#include "communication_c/shared_data_c.h"


namespace phoenix {
namespace hmi {


class WidgetMap : public QGLWidget {
  Q_OBJECT

 public:
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit WidgetMap(QWidget* parent = 0);
  ~WidgetMap();

  void Update();

 protected:
  void wheelEvent(QWheelEvent *event);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void resizeGL(Int32_t w, Int32_t h);
  void initializeGL();
  void paintGL();

 private:
  void LoadIcon();
  void ChangeView();
  void DrawViewSwitchButton();

  void SetDisplayOriginCoor(Float64_t x, Float64_t y, Float64_t z) {
    view_param_.origin_coor.x = x;
    view_param_.origin_coor.y = y;
    view_param_.origin_coor.z = z;
  }

  inline void DrawVertex(Float64_t x, Float64_t y, Float64_t z) {
    glVertex3d(x-view_param_.origin_coor.x,
               y-view_param_.origin_coor.y,
               z-view_param_.origin_coor.z);
  }

  void DrawCoordinateAxis(Float64_t x, Float64_t y, Float64_t z, Float64_t len);
  void DrawGrid(const phoenix::common::AABBox2d& range, Float64_t interval);
  void DrawArrow(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t len, Float64_t height = 0.0);
  void DrawAABB_2D(const phoenix::common::AABBox2d& aabb, Float64_t height = 0.0);
  void DrawOBB_2D(const phoenix::common::OBBox2d& obb, Float64_t height = 0.0);
  void DrawBarrier(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t width = 6.0, Float64_t height = 2.0);
  void DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
      Float64_t heading = 0.0, Float64_t z = 0.0, Int32_t count = 20);
  void DrawYawRatePath(
      Float64_t start_x, Float64_t start_y, Float64_t start_h,
      Float64_t curvature, Float64_t length, Float64_t z = 0.0);
  void DrawVehicle();

  void UpdateModuleInfo();
  void DrawMap();
  void DrawTargetPath();
  void DrawLateralCtlInfo();

  void DrawVelocityPedal();
  void DrawAccelerationPedal();
  void DrawLongitudinalCtlValuePedal();
  void DrawModuleStatusInfo();
  void DrawChassisInfo();

 private:
  struct {
    common::Quaternion<Float64_t> qu_rot;
    common::Matrix<Float64_t, 3, 3> mat_rot;
    common::Vector3d vec_move;

    struct {
      Int32_t x = 0;
      Int32_t y = 0;
    } prev_mouse_pos;

    struct {
      Int32_t x = 0;
      Int32_t y = 0;
    } first_click_mouse_pos;

    struct {
      Float64_t x = 0;
      Float64_t y = 0;
      Float64_t z = 0;
    } origin_coor;

    Float64_t scaling = 1.0;
  } view_param_;

  struct ModuleInfo {
    ModuleStatusList_t module_status_list;

    Imu_t imu;
    Chassis_t chassis_info;
    SpecialChassisInfo_t special_chassis_info;
    ChassisCtlCmd_t chassis_cmd;
    PlanningResult_t planning_result;
    LateralControlInfo_t lateral_control_info;

    ModuleInfo() {
      Phoenix_AdMsg_ClearImu(&imu);
      Phoenix_AdMsg_ClearChassis(&chassis_info);
      Phoenix_AdMsg_ClearSpecialChassisInfo(&special_chassis_info);
      Phoenix_AdMsg_ClearChassisCtlCmd(&chassis_cmd);
      Phoenix_AdMsg_ClearPlanningResult(&planning_result);
      common::com_memset(&lateral_control_info, 0, sizeof(lateral_control_info));
    }
  } module_info_;

  enum {
    VIEW_MODE_1,
    VIEW_MODE_2
  };
  Int32_t vew_mode_ = VIEW_MODE_1;
  common::AABBox2d view_switch_button_;
  GLuint texture_id_of_icon_aerial_view_;
  GLuint texture_id_of_icon_3d_view_;

  std::vector<std::string> fuzzy_level_string_;
  // 字符串缓存，用来在窗口显示字符信息
  enum { MAX_STRING_BUFF_SIZE = 1024*2 };
  Char_t string_buffer_[MAX_STRING_BUFF_SIZE];
};


} // hmi
} // phoenix


#endif  // PHOENIX_WIDGET_MAP_H_
