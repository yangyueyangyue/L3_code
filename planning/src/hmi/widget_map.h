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
  void ConstructPolarPlotPoints();
  void DrawPolarPlot();
  void DrawArrow(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t len, Float64_t height = 0.0);
  void DrawAABB_2D(const phoenix::common::AABBox2d& aabb, Float64_t height = 0.0);
  void DrawOBB_2D(const phoenix::common::OBBox2d& obb, Float64_t height = 0.0);
  void DrawBarrier(Float64_t x, Float64_t y, Float64_t heading,
      Float64_t width = 6.0, Float64_t height = 2.0);
  void DrawEllipse(Float64_t x, Float64_t y, Float64_t a, Float64_t b,
      Float64_t heading = 0.0, Float64_t z = 0.0, Int32_t count = 20);
  void DrawIcon(GLuint texture_id, const common::AABBox2d& area);
  void DrawVehicle();

  void UpdateModuleInfo();
  void DrawMap();

  void DrawRelativePosList();
  void DrawLaneMarkCameraList();
  void DrawLaneInfoCameraList();
  void DrawObstacleCameraList();
  void DrawObstacleRadarFrontList();
  void DrawObstacleRadarRearList();
  void DrawObstacleRadarFrontLeftList();
  void DrawObstacleRadarFrontRightList();
  void DrawObstacleRadarRearLeftList();
  void DrawObstacleRadarRearRightList();
  void DrawSrr2DetectionsFrontLeftList();
  void DrawSrr2DetectionsFrontRightList();
  void DrawSrr2DetectionsRearLeftList();
  void DrawSrr2DetectionsRearRightList();
  void DrawObstacleLidarList0();
  void DrawObstacleLidarList1();
  void DrawObstacleTrackedInfoList();
  void DrawObstacleList();

  void DrawDrivingMap();
  void DrawRoadArea();
  void DrawRoadLaneLine();
  void DrawMapTrafficLights();
  void DrawReferenceLines();
  void DrawObstaclesOfDrivingMap();

  void DrawTrajectoryPlanningInfo();
  void DrawTargetTrajectory();
  void DrawVelocityPlanningInfo();

  void DrawSpeedRestrictionMark();
  void DrawTrafficLight();
  void DrawVelocityPedal();
  void DrawAccelerationPedal();
  void DrawModuleStatusInfo();
  void DrawChassisInfo();
  void DrawSteeringWheelAnglePedal();
  void DrawLongitudinalCtlValuePedal();

  void DrawSceneStory();

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

    ad_msg::Gnss gnss;
    ad_msg::Gnss filtered_gnss_info;
    ad_msg::Imu imu;
    ad_msg::Chassis chassis_info;
    ad_msg::ChassisCtlCmd chassis_ctl_cmd_info;
    ad_msg::RelativePosList relative_pos_list;
    ad_msg::RelativePosList gnss_track_list;
    ad_msg::RelativePosList gnss_filtered_track_list;
    ad_msg::RelativePosList utm_track_list;
    ad_msg::RelativePosList utm_filtered_track_list;
    ad_msg::RelativePosList odom_track_list;
    ad_msg::RelativePosList odom_filtered_track_list;
    ad_msg::LaneMarkCameraList lane_mark_camera_list;
    ad_msg::LaneInfoCameraList lane_info_camera_list;
    ad_msg::ObstacleCameraList obstacle_camera_list;
    ad_msg::ObstacleRadarList obstacle_radar_front_list;
    ad_msg::ObstacleRadarList obstacle_radar_rear_list;
    ad_msg::ObstacleRadarList obstacle_radar_front_left_list;
    ad_msg::ObstacleRadarList obstacle_radar_front_right_list;
    ad_msg::ObstacleRadarList obstacle_radar_rear_left_list;
    ad_msg::ObstacleRadarList obstacle_radar_rear_right_list;
    ad_msg::ObstacleRadarList srr2_detections_front_left_list;
    ad_msg::ObstacleRadarList srr2_detections_front_right_list;
    ad_msg::ObstacleRadarList srr2_detections_rear_left_list;
    ad_msg::ObstacleRadarList srr2_detections_rear_right_list;
    ad_msg::ObstacleLidarList obstacle_lidar_list_0;
    ad_msg::ObstacleLidarList obstacle_lidar_list_1;
    ad_msg::ObstacleTrackedInfoList obstacle_tracked_info_list;
    ad_msg::ObstacleList obstacle_list;
    ad_msg::TrafficSignalList traffic_signal_list;
    ad_msg::TrafficLightList traffic_light_list;

    driv_map::DrivingMapInfo driving_map_info;
    planning::TrajectoryPlanningInfo trajectory_planning_info;

    planning::ActionPlanningResult action_planning_result;
    planning::TrajectoryPlanningResult trajectory_planning_result;
    planning::VelocityPlanningResult velocity_planning_result;


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
  GLuint texture_id_of_icon_curve_;
  GLuint texture_id_of_icon_ramp_;
  GLuint texture_id_of_icon_passing_ramp_;
  GLuint texture_id_of_icon_tunnel_;

  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
  left_boundary_of_target_trajectory_;
  common::StaticVector<common::Vec2d, common::Path::kMaxPathPointNum>
  right_boundary_of_target_trajectory_;

  std::vector<std::vector<common::Vec2d> > polar_plot_points;

  std::vector<std::string> fuzzy_level_string_;

  // 字符串缓存，用来在窗口显示字符信息
  enum { MAX_STRING_BUFF_SIZE = 1024*2 };
  Char_t string_buffer_[MAX_STRING_BUFF_SIZE];
};


} // hmi
} // phoenix


#endif  // PHOENIX_HMI_WIDGET_MAP_H_
