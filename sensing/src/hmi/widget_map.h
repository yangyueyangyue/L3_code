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
#ifndef PHOENIX_HMI_WIDGET_MAP_H_
#define PHOENIX_HMI_WIDGET_MAP_H_

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

#include "communication/shared_data.h"

#include <iostream>
#include <fstream>
#include <ostream>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <rosgraph_msgs/Clock.h>
// #include <locale.h>
// #include <stdio.h>

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

  // 画点
  inline void DrawVertex(Float64_t x, Float64_t y, Float64_t z) {
    glVertex3d(x-view_param_.origin_coor.x,
               y-view_param_.origin_coor.y,
               z-view_param_.origin_coor.z);
  }

  // 绘制坐标轴
  void DrawCoordinateAxis(Float64_t x, Float64_t y, Float64_t z, Float64_t len);
  // 绘制网格
  void DrawGrid(const phoenix::common::AABBox2d& range, Float64_t interval);
  // 绘制箭头
  void DrawArrow(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t len, Float64_t height = 0.0);
  // 绘制十字架
  void DrawCross(Float64_t x, Float64_t y, Float64_t len);
  // 绘制AABB框
  void DrawAABB_2D(const phoenix::common::AABBox2d& aabb, Float64_t height = 0.0);
  // 绘制OBB框
  void DrawOBB_2D(const phoenix::common::OBBox2d& obb, Float64_t height = 0.0);
  // 绘制障碍
  void DrawBarrier(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t width = 6.0, Float64_t height = 2.0);
  // 绘制椭圆
  void DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
      Float64_t heading = 0.0, Float64_t z = 0.0, Int32_t count = 20);
  void DrawEllipse_Camera(Float64_t x, Float64_t y, Float64_t a, Float64_t b, \
                          Float64_t l_end = 0.0, Float64_t r_end = 0.0, \
                          Float64_t heading = 0.0, Float64_t z = 0.0, Int32_t count = 20);
  void DrawVehicle();

  void UpdateModuleInfo();
  void DrawMap();

  void DrawLaneMarkCameraList();
  void DrawLaneCurbCameraList();
  void DrawObstacleCameraList();
  void DrawObstacleEsrFrontList();
  void DrawTrafficConeList();
  void DrawObstacleMaxieyeCameraList();
  void DrawObstacleLidarListFront();
  void DrawObstacleAnngicRadarFrontLeftList();
  void DrawObstacleAnngicRadarFrontRightList();
  void DrawObstacleAnngicRadarRearLeftList();
  void DrawObstacleAnngicRadarRearRightList();
  void DrawVisualControlLaneMarkCameraList();
  void DrawObstacleVisualControlFrontList();
  void DrawObstacleVisualControlFrontLeftList();
  void DrawObstacleVisualControlFrontRightList();
  void DrawObstacleVisualControlRearLeftList();
  void DrawObstacleVisualControlRearRightList();
  void DrawObstacleVisualControlRearList();
  void DrawObstacleFusionList();
  void DrawADHMIObstacleFusionList();

  void DrawChassisInfo();
  void DrawStatusInfo();

    // 将时间戳转换为时间
  void TimeStampToLocalTime();

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
    ad_msg::ModuleStatusList module_status_list;

    ad_msg::Gnss gnss_info_;
    ad_msg::Imu imu_info_;
    ad_msg::Chassis chassis_info;
    ad_msg::LaneMarkCameraList lane_mark_camera_list;
    ad_msg::LaneMarkCameraList lane_curb_camera_list;
    ad_msg::ObstacleCameraList obstacle_camera_list;
    ad_msg::ObstacleRadarList obstacle_esr_front_list;
    ad_msg::ObstacleCameraList obstacle_maxieye_camera_list;
    ad_msg::ObstacleLidarList obstacle_lidar_list_front;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_front_left_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_front_right_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_left_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_right_list;
    ad_msg::LaneMarkCameraList visual_control_lane_mark_camera_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_left_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_right_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_left_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_right_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_list;
    ad_msg::ObstacleList obstacle_fusion_list;
    ad_msg::ObstacleList adhmi_obstacle_fusion_list;
    rosgraph_msgs::Clock adhmi_clock;

    ModuleInfo() {
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

  // 字符串缓存，用来在窗口显示字符信息
  enum { MAX_STRING_BUFF_SIZE = 1024*2 };
  Char_t string_buffer_[MAX_STRING_BUFF_SIZE];

  Int32_t width_value = width();
  Int32_t height_value = height();
  Int32_t line_number = 0;
  Int32_t tab_width = 450;

  // rosbag内部时间
  Char_t local_time[1024];
};


} // hmi
} // phoenix


#endif  // PHOENIX_HMI_WIDGET_MAP_H_
