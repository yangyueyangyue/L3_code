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

#define ENABLE_DISPLAY_AROUND_RADAR_TEXT  (1)

#define ENABLE_DISPLAY_HANDLED_AROUND_RADAR_TEXT  (1)

#define ENABLE_DISPLAY_MAIN_RADAR_TEXT  (0)

#define ENABLE_DISPLAY_FUSION_TEXT   (1)

#define ENABLE_DISPLAY_VISUAL_CONTROL_TEXT (1)

#define ENABLE_DISPLAY_CAMERA_TEXT  (0)

#define ENABLE_DISPLAY_AROUND_RADAR   (1)  

#define ENABLE_DISPLAY_PREPROCESS_AROUND_RADAR   (1)  

#define ENABLE_DISPLAY_CHASSIS_INFO (1)

#define ENABLE_DISPLAY_VISUAL_CONTROL (1)

#define ENABLE_DISPLAY_PREPROCESS_JM (1)





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
  void DrawCross(Float64_t x, Float64_t y, Float64_t len);
  void DrawCross10(Float64_t x, Float64_t y, Float64_t len);
  void DrawAABB_2D(const phoenix::common::AABBox2d& aabb, Float64_t height = 0.0);
  void DrawOBB_2D(const phoenix::common::OBBox2d& obb, Float64_t height = 0.0);
  void DrawBarrier(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t width = 6.0, Float64_t height = 2.0);
  void DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
      Float64_t heading = 0.0, Float64_t z = 0.0, Int32_t count = 20);
  void DrawVehicle();

  void UpdateModuleInfo();
  void DrawMap();

  void DrawLaneMarkCameraList();
  void DrawLaneCurbCameraList();
  void DrawObstacleCameraList();
  void DrawObstacleEsrFrontList();
  void DrawObstacleMaxieyeCameraList();
  void DrawObstacleMaxieyeCameraAfterPreprocessList();
  void DrawObstacleLidarListFront();
  
  void DrawObstacleAnngicRadarFrontLeftList();
  void DrawObstacleAnngicRadarFrontRightList();
  void DrawObstacleAnngicRadarRearLeftList();
  void DrawObstacleAnngicRadarRearRightList();

  void DrawObstacleRadarLeftFwList();
  void DrawObstacleRadarRightFwList();
  void DrawObstacleRadarLeftBwList();
  void DrawObstacleRadarRightBwList();
  void DrawObstacleAroundRadarList(const ad_msg::ObstacleRadarList& obstacle_list , const QColor& color);

  void DrawObstacleVisualControlFrontList();
  void DrawObstacleVisualControlFrontLeftList();
  void DrawObstacleVisualControlFrontRightList();
  void DrawObstacleVisualControlRearLeftList();
  void DrawObstacleVisualControlRearRightList();
  void DrawObstacleVisualControlRearList();
  void DrawObstacleFusionList();

  void DrawObstacleJMFrontList();
  void DrawObstacleJMFrontLeftList();
  void DrawObstacleJMFrontRightList();
  void DrawObstacleJMRearLeftList();
  void DrawObstacleJMRearRightList();

  void DrawObstacleJMCameraList(const ad_msg::ObstacleCameraList& obstacle_list , const QColor& color);

  void DrawChassisInfo();
  void DrawStatusInfo();

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
    ad_msg::ObstacleCameraList obstacle_maxieye_camera_after_preprocess_list;
    ad_msg::ObstacleLidarList obstacle_lidar_list_front;

    ad_msg::ObstacleRadarList obstacle_anngic_radar_front_left_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_front_right_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_left_list;
    ad_msg::ObstacleRadarList obstacle_anngic_radar_rear_right_list;

    ad_msg::ObstacleRadarList obstacle_radar_left_fw_filter_list;
    ad_msg::ObstacleRadarList obstacle_radar_right_fw_filter_list;
    ad_msg::ObstacleRadarList obstacle_radar_left_bw_filter_list;
    ad_msg::ObstacleRadarList obstacle_radar_right_bw_filter_list;

    ad_msg::ObstacleCameraList obstacle_visual_control_front_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_left_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_front_right_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_left_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_right_list;
    ad_msg::ObstacleCameraList obstacle_visual_control_rear_list;

    ad_msg::ObstacleCameraList obstacle_jm_front_list;
    ad_msg::ObstacleCameraList obstacle_jm_front_left_list;
    ad_msg::ObstacleCameraList obstacle_jm_front_right_list;
    ad_msg::ObstacleCameraList obstacle_jm_rear_left_list;
    ad_msg::ObstacleCameraList obstacle_jm_rear_right_list;
    ad_msg::ObstacleCameraList obstacle_jm_rear_list;

    ad_msg::ObstacleList obstacle_fusion_list;

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
};


} // hmi
} // phoenix


#endif  // PHOENIX_HMI_WIDGET_MAP_H_
