#ifndef PERCEPTION_SENSORSFUSION_MULTI_CONSTRUCTURE_CLASS_REPO_H_
#define PERCEPTION_SENSORSFUSION_MULTI_CONSTRUCTURE_CLASS_REPO_H_

#include "ad_msg.h"
#include "geometry/geometry_utils.h"
#include "container/ring_buffer.h"
#include "container/doubly_linked_list.h"
#include "curve/path.h"
#include "sensors_fusion_macros.h"
#include "sensors_fusion_param_macros.h"
#include "driving_map_wrapper.h"

namespace phoenix{
namespace perception{
namespace sensorsfusion{

/**
 * @struct RelativePos
 * @brief 单个目标相对位置信息(当前车身坐标系下)
 */
struct ObjPosition{
    Float32_t x_;                //水平向前，单位：米
    Float32_t y_;                //水平向左，单位：米
    Float32_t range_;            //障碍物距离，单位：米
    Float32_t angle_;            //通过反正切函数求角度(y/x)，单位：角度 (-PI/2 ~ PI/2)
    Float32_t heading_;          //航向，单位：（-PI～PI弧度）

    void Clear();

    ObjPosition();

};


/**
 * @struct ObjInfosBase
 * @brief 单个障碍物信息基类
 */
struct ObjInfosBase{

    ObjInfosBase();
    void Clear();

    Int32_t raw_obj_id_;          //从传感器端来源的障碍物ID
    Int32_t raw_obj_idx_;         //在障碍物列表中的标号
    Int32_t sensor_type_;         //传感器类型来源
    Int32_t obj_type_;            //障碍物类型

    struct ObjPosition pos_;      //单个障碍物位置信息

    Float32_t width_;             //障碍物宽。单位：米
    Float32_t height_;            //障碍物高。单位：米
    Float32_t length_;            //障碍物长。单位：米

    Float32_t v_x_;               //障碍物前向速度。单位：m/s
    Float32_t v_y_;               //障碍物左向速度。单位：m/s

    Float32_t a_x_;               //障碍物前向加速度。单位：m/s2
    Float32_t a_y_;               //障碍物左向加速度。单位：m/s2

    common::AABBox2d aabb_;       //障碍物物理矩形框大小
};


/**
 * @struct ObjIndex
 * @brief 障碍物属性（障碍物ID，传感器来源）
 */
struct ObjIndex{
    Int32_t obj_idx_;                //跟踪的障碍物ID序号从0开始（包含0）
    Int32_t sensor_idx_;             //跟踪的传感器ID序号从0开始（包含0）

    ObjIndex();

    ObjIndex(const Int32_t obj_idx, const Int32_t sen);

    void Clear();
    bool IsValid() const;
};


/**
 * @struct ObjID
 * @brief 障碍物属性（障碍物ID，生命周期）
 */
struct ObjID{
  Int32_t id_;
  Int32_t age_;

  void Clear() {
    id_ = -1;
    age_ = 0;
  }

  ObjID(){
      Clear();
  }
};


/**
 * @struct ObjFromCamera
 * @brief 从相机获取的单个障碍物
 */
struct ObjFromCamera : public ObjInfosBase{
    ad_msg::ObstacleCamera raw_data_;
    ObjIndex tracked_obj_index_;
    int sensor_id; 
    void Clear();

    ObjFromCamera();

};

/**
 * @struct ObjsFromCamera
 * @brief 从单个相机而来的多个障碍物
 */
struct ObjsFromCameraList{
    enum { MAX_OBJS_NUM = 64 };

    ad_msg::MsgHead msg_head_;
    Int32_t sensor_id_;

    common::StaticVector<ObjFromCamera, ObjsFromCameraList::MAX_OBJS_NUM> objects_;

    void Clear();

    ObjsFromCameraList();

};

/**
 * @struct ObjFromRadar
 * @brief 从雷达获取单个障碍物
 */
struct ObjFromRadar : public ObjInfosBase{
    ad_msg::ObstacleRadar raw_data_;
    ObjIndex tracked_obj_index_;
    void Clear();

    ObjFromRadar();
};

/**
 * @struct ObjsFromRadar
 * @brief 从雷达获取多个障碍物
 */
struct ObjsFromRadarList{
    enum{ MAX_OBJS_NUM = 64 };

    ad_msg::MsgHead msg_head_;
    Int32_t sensor_id_;

    common::StaticVector<ObjFromRadar, ObjsFromRadarList::MAX_OBJS_NUM> objects_;

    void Clear();

    ObjsFromRadarList();
};


/**
 * @struct ObjFromLidar
 * @brief 从激光雷达获取单个障碍物
 */
struct ObjFromLidar : public ObjInfosBase{
    ad_msg::ObstacleLidar raw_data_;
    ObjIndex tracked_obj_index_;

    void Clear();

    ObjFromLidar();
};

/**
 * @struct ObjsFromLidar
 * @brief 从激光雷达获取多个障碍物
 */
struct ObjsFromLidarList{
    enum{ MAX_OBJS_NUM = 64 };

    ad_msg::MsgHead msg_head_;
    Int32_t sensor_id_;

    common::StaticVector<ObjFromLidar, ObjsFromLidarList::MAX_OBJS_NUM> objects_;

    void Clear();

    ObjsFromLidarList();
};


/**
 * @struct ObjsFilterDataSource
 * @brief 障碍物Filter源数据，用于Filter障碍物
 */
struct ObjsFilterDataSource {
  Int64_t timestamp_;
//  const ad_msg::RelativePosList* rel_pos_list_;

//  const ad_msg::ObstacleCameraList* objs_list_main_forward_cam_;
//  const ad_msg::ObstacleRadarList* objs_list_main_forward_radar_;
//  const ad_msg::ObstacleLidarList* objs_list_main_forward_lidar_;

//  const ad_msg::ObstacleRadarList* objs_list_left_side_radar_;
//  const ad_msg::ObstacleRadarList* objs_list_right_side_radar_;
//  const ad_msg::ObstacleRadarList* objs_list_left_backward_radar_;
//  const ad_msg::ObstacleRadarList* objs_list_right_backward_radar_;

//  const ad_msg::ObstacleCameraList* objs_list_center_forward_cam_;
//  const ad_msg::ObstacleCameraList* objs_list_left_side_cam_;
//  const ad_msg::ObstacleCameraList* objs_list_right_side_cam_;
//  const ad_msg::ObstacleCameraList* objs_list_left_backward_cam_;
//  const ad_msg::ObstacleCameraList* objs_list_right_backward_cam_;

//  const driv_map::DrivingMapWrapper* driving_map_;

  ad_msg::LaneMarkCameraList* curb_list_cam_;

  ad_msg::RelativePosList* rel_pos_list_;

  ad_msg::ObstacleCameraList* objs_list_main_forward_cam_;
  ad_msg::ObstacleCameraList* objs_list_main_forward_cam_80_;
  ad_msg::ObstacleCameraList* objs_list_main_forward_cam_150_;
  ad_msg::ObstacleRadarList* objs_list_main_forward_radar_;
  ad_msg::ObstacleLidarList* objs_list_main_forward_lidar_;

  ad_msg::ObstacleRadarList* objs_list_left_side_radar_;
  ad_msg::ObstacleRadarList* objs_list_right_side_radar_;
  ad_msg::ObstacleRadarList* objs_list_left_backward_radar_;
  ad_msg::ObstacleRadarList* objs_list_right_backward_radar_;

  ad_msg::ObstacleCameraList* objs_list_center_forward_cam_;
  ad_msg::ObstacleCameraList* objs_list_left_side_cam_;
  ad_msg::ObstacleCameraList* objs_list_right_side_cam_;
  ad_msg::ObstacleCameraList* objs_list_left_backward_cam_;
  ad_msg::ObstacleCameraList* objs_list_right_backward_cam_;

  driv_map::DrivingMapWrapper* driving_map_;

  /**
   * @brief 构造函数
   */
  ObjsFilterDataSource();
 };

/**
 * @struct EgoVehicleParams
 * @brief
 */
struct EgoVehicleParams{
  // 车长，单位：米
  Float32_t vehicle_length_;
  // 车宽，单位：米
  Float32_t vehicle_width_;
  // 车高，单位：米
  Float32_t vehicle_height_;
  // 车辆的定位点到 front of vehicle 的距离，单位：米
  Float32_t dist_of_localization_to_front_;
  // 车辆的定位点到 rear of vehicle 的距离，单位：米
  Float32_t dist_of_localization_to_rear_;
  // 车辆的定位点到中心点的距离，单位：米
  Float32_t dist_of_localization_to_center_;

  void Clear();

  EgoVehicleParams& operator =(const EgoVehicleParams& vehicle_params);

  EgoVehicleParams();
};


/**
 * @struct RelativePos
 * @brief 相对位置(当前车身坐标系下)
 */
struct RelativePos {
  /// 相对时间 ms
  Int32_t relative_time_;

  /// 相对位置 x坐标，单位（米）
  Float32_t x_;
  /// 相对位置 y坐标，单位（米）
  Float32_t y_;
  /// 相对的航向角（-PI～PI弧度）
  Float32_t heading_;
  /// 航向角的角速度，单位（弧度/秒）
  Float32_t yaw_rate_;
  /// 速度，单位（米/秒）
  Float32_t v_;

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 构造函数
   */
  RelativePos() {
    Clear();
  }
};

/**
 * @struct RelativePosList
 * @brief 相对位置列表
 */
struct RelativePosList {
  /// 最大位置点的数量
  enum { MAX_RELATIVE_POS_NUM = 40 };

  /// 消息头
  ad_msg::MsgHead msg_head_;
  /// 相对位置点的数量
  Int32_t relative_pos_num_;
  /// 相对位置点的列表
  RelativePos relative_pos_[MAX_RELATIVE_POS_NUM];

  /**
   * @brief 清除内部数据
   */
  void Clear();

  /**
   * @brief 构造函数
   */
  RelativePosList() {
    Clear();
  }
};

/**
 * @struct TrjPointInfos
 * @brief 定义跟踪轨迹点信息
 */
struct TrjPointInfos {
    Int32_t seq_num_;
    Float32_t relative_time_ms_;
    Float32_t x_;
    Float32_t y_;
    Float32_t vx_;
    Float32_t vy_;

    TrjPointInfos();

    void Clear();
};


/**
 * @struct ObjTrjAvgStatus
 * @brief 障碍物跟踪轨迹平滑
 */
struct ObjTrjAvgStatus{
  Float32_t x_;
  Float32_t y_;
  Float32_t v_x_;
  Float32_t v_y_;
  Float32_t a_x_;
  Float32_t a_y_;

  ObjTrjAvgStatus(){
      common::com_memset(this, 0.0F, sizeof(ObjTrjAvgStatus));
  }
};


/**
 * @struct ObjsAssociation
 * @brief
 */
struct ObjsAssociation{
    bool mismatch_;
    ObjIndex tar_obj_index_;
    Int32_t cost_;
    Float32_t dist_to_tar_obj_;

    void Clear() {
        mismatch_ = false;
        tar_obj_index_.Clear();
        cost_ = 0;
        dist_to_tar_obj_ = 0.0F;
    }

    ObjsAssociation(){
        Clear();
    }
};

typedef common::DoublyLinkedList<ObjsAssociation, MAX_OBJS_ASSOCIATION_NUM> ObjAssoListType;
typedef common::DataPool<ObjAssoListType::node_type, MAX_OBJS_ASSOCIATION_NUM> ObjAssoPoolType;


/**
* @struct SensorsRef
* @brief 跟不同类型传感器有关联的数据。最多为 SensorsRef::MAX_OBJ_ID_NUM条轨迹
*/
struct SensorsRef {
//Note：这两个宏的数量需相同
enum { MAX_OBJ_ID_NUM = 4 };
//定义跟踪相关信息
enum { MAX_POS_POINT_NUM = 4 };

// 有效标记
bool valid_;

// 传感器类型
Int32_t sensor_type_;

// 障碍物类型
Int32_t obj_type_;

// 障碍物宽度
Float32_t width_;

// 障碍物长度
Float32_t length_;

// 障碍物高度
Float32_t height_;

// 障碍物x向速度分量
Float32_t v_x_;

// 障碍物y向速度分量
Float32_t v_y_;

// 障碍物x向加速度分量
Float32_t a_x_;

// 障碍物y向加速度分量
Float32_t a_y_;

// 最近一次跟踪(匹配)上的标记
Int32_t age_;

Int32_t last_age_;

// 障碍物的包围盒
common::AABBox2d aabb_;

//  // 传感器关联数量
//  Int32_t asso_sensors_num_;

// 障碍物ID
struct ObjID obj_id_[MAX_OBJ_ID_NUM];

struct ObjPosition pos_[MAX_POS_POINT_NUM];

void Clear();


Int32_t FindObjId(const Int32_t id) const;

Int32_t AddPartnerObjId(const Int32_t id, const Int32_t duration);

Int32_t FetchSensorObjId() const;

SensorsRef(){
  Clear();
}

};


/**
* @struct SensorsFusionObj
* @brief
*/
struct SensorsFusionObj{
  // 有效标记
  bool valid_;

  // 障碍物类型
  Int32_t obj_type_;

  // 障碍物宽度
  Float32_t width_;

  // 障碍物高度
  Float32_t height_;

  // 障碍物长度
  Float32_t length_;

  // 障碍物位置
  struct ObjPosition pos_;

  // 障碍物的包围盒
  common::AABBox2d aabb_;

  // 障碍物x向速度分量
  Float32_t v_x_;

  // 障碍物y向速度分量
  Float32_t v_y_;

  // 障碍物x向加速度分量
  Float32_t a_x_;

  // 障碍物y向加速度分量
  Float32_t a_y_;

  SensorsFusionObj() {
    Clear();
  }

  void Clear();
};


/**
 * @struct ObjsAssociation
 * @brief 用于关联其它障碍物(匹配用)
 */
struct ObjsAssociationList{
  ObjAssoListType objs_ass_list_[LIST_ID_MAX];
};


/**
 * @struct MainRefLineRelation
 * @brief 跟主参考线的关联关系
 */
struct MainRefLineRelation{
  bool valid_;
  common::PathPoint proj_on_major_ref_line_;

  void Clear() {
    valid_ = false;
    proj_on_major_ref_line_.Clear();
  }

  MainRefLineRelation(){
      Clear();
  }
};


/**
 * @struct MatchingInfos
 * @brief
 */
struct MatchingInfos{
    bool always_match_;
    ObjIndex candidate_idx_;

    void Clear(){
        always_match_ = false;
        candidate_idx_.Clear();
    }

    MatchingInfos(){
        Clear();
    }
};


/**
 * @struct MatchingCondition
 * @brief
 */
struct MatchingCondition {
  bool valid_;
  Float32_t max_x_diff_;
  Float32_t max_y_diff_;
  Float32_t max_vx_diff_;
  Float32_t max_vy_diff_;

  MatchingCondition();

  void Clear();
};


/**
 * @struct MatchingParams
 * @brief
 */
struct MatchingParams {
  Float32_t searching_radius_;
  bool masks_[SENSOR_ID_MAX];

  Int32_t matched_sensors_nums_;
  Int32_t min_matched_times_;
  Int32_t max_dismatch_times_;
  Float32_t max_vx_diff_;
  Float32_t max_vy_diff_;

  struct MatchingCondition condition_;

  bool using_est_pos_;
  bool enable_add_mul_obj_id_;
  bool enable_sub_optimal_match_;
  bool match_again_by_id_if_unmatched_;
  bool match_with_velocity_;


  Float32_t max_x_diff_for_matched_again_by_id;
  Float32_t max_y_diff_for_matched_again_by_id;
  Float32_t max_vx_diff_for_matched_again_by_id;
  Float32_t max_vy_diff_for_matched_again_by_id;


  MatchingParams() {
    Clear();
  }

  void Clear();
};


/**
 * @struct ObjsTracked
 * @brief
 */
struct ObjsTracked {
    // 删除标志
    bool erase_flag_;

    // 列表标记（跟踪上的obj， 新加入的雷达obj，新加入的相机obj，新加入的激光雷达obj）
    Int32_t list_id_;

    // 原始传感器数据index
    Int32_t raw_obj_idx_;

    // 跟踪状态
    Int32_t track_status_;

    // 生命值，若小于等于0，则将从列表删除
    Int32_t life_;

    // 保存上一次的生命值 
    Int32_t last_life_;

    // 生命值
    Int32_t life_ref_count_;

    // 跟踪（匹配）上的次数
    Int32_t matched_;

    // 最近一次跟踪（匹配）上的标记
    /*
    age_实际作用就是用来保存上一次匹配上时duration_的值。
    */
    Int32_t age_;

    // 障碍在列表中存在的时间（次数）
    Int32_t duration_;

    // 上一次匹配时的感知类型
    Int32_t last_perception_type;

    // 融合障碍物id
    Uint32_t fusion_id;

    // 感知类型
    Int32_t perception_type;

    // 置信度
    Int32_t confidence_;

    // 障碍物的航向角
    Float32_t heading_;

    // 平滑后的位置
    common::Matrix<Float32_t, 6, 1> pos_status_;

    // 平滑后的位置的方差矩阵
    common::Matrix<Float32_t, 6, 6> pos_covariance_;

    // 跟踪的轨迹.目前只跟踪两条轨迹
    common::RingBuffer<TrjPointInfos, MAX_TRACKED_OBJ_TRJ_POINT_NUM> traced_trj_[2];

    // 跟不同类型传感器有关联的数据
    struct SensorsRef sensor_ref_[SENSOR_ID_MAX];

    struct SensorsFusionObj fused_info_;

    // 障碍物跟踪轨迹平滑
    struct ObjTrjAvgStatus avg_status_;

    // 用于关联其它障碍物(匹配用)
    struct ObjsAssociationList objs_ass_;

    // 跟主参考线的关联关系
    struct MainRefLineRelation ref_line_;

    // 构造函数
    ObjsTracked();

    // 清除内部所有信息
    void Clear(ObjAssoPoolType& obj_ass_pool);

    // 清除内部信息(关联信息除外)
    void Clear();

    // 清除关联信息
    void ClearAssoList(ObjAssoPoolType& obj_ass_pool);

    // 清除跟踪的轨迹
    void ClearTrackedTrj();

    // 获取障碍物外形尺寸
    void GetObjOverallDimension(
       const Int32_t sen_idx, Float32_t* length, Float32_t* width, Float32_t* height) const;

    bool IsOhterSensorValid(const Int32_t sen_id) const;

    bool IsRadarValid() const;

    bool IsCameraValid() const;

    bool IsLidarValid() const;

    bool IsForesightObj() const;

    bool IsAroundObj() const;

    bool IsRightSensorObj() const;

    bool IsLeftSensorObj() const;
    
    void CalcFusedSensorObj(SensorsFusionObj* fused_obj);

    void GenerateJointModeFusionID();

};


/**
 * @struct ObjTrackedList
 * @brief
 */
struct ObjTrackedList{
  ad_msg::MsgHead msg_head_;
  common::StaticVector<ObjsTracked, MAX_TRACKED_OBJ_NUM> objects_;

  ObjTrackedList() {}

  void Clear(ObjAssoPoolType& obj_asso_pool) {
      msg_head_.Clear();

      Int32_t obj_num = objects_.Size();
      for (Int32_t i = 0; i < obj_num; ++i) {
          objects_[i].Clear(obj_asso_pool);
      }

      objects_.Clear();
  }
};


/**
 * @class CalcSquareDistToObjTracked
 * @brief 计算OBJ到OBJ的平方距离
 */
class CalcSquareDistToObjTracked {
public:
  /*
   * @brief 构造函数
   * @param[in]  point   待查询点
   * @param[in]  lane_lines  车道线
   */
  CalcSquareDistToObjTracked(
      const common::AABBox2d& aabb,
      const common::StaticVector<ObjsTracked, MAX_TRACKED_OBJ_NUM>& objects)
    : obj_aabbox_(aabb), tracked_objects_(objects) {
    // nothing to do
  }
  /*
   * @brief 重载的'()'运算符
   * @param[in]  aabb            待查询的目标
   * @param[in]  tree_obj_index  待查询的目标在k-d树中对应的索引
   * @return 点到目标的平方距离。
   */
  Float32_t operator ()(
      const ObjIndex& index, Int32_t tree_obj_index) const {
    return (common::Square(common::DistAABBToAABB_2D(
                             tracked_objects_[index.obj_idx_].sensor_ref_[
                           index.sensor_idx_].aabb_, obj_aabbox_)));
  }

private:
  // 待查询Bounding box of target obj
  const common::AABBox2d& obj_aabbox_;
  const common::StaticVector<ObjsTracked, MAX_TRACKED_OBJ_NUM>&
      tracked_objects_;
};


/**
 * @struct FuncCmpObjAssoCost
 * @brief
 */
class FuncCmpObjAssoCost {
public:
  bool operator()(
      const ObjsAssociation& data1, const ObjsAssociation& data2) const {
    bool ret = false;
    if (data1.cost_ < data2.cost_) {
      ret = true;
    } else if (data1.cost_ == data2.cost_) {
      ret = (data1.dist_to_tar_obj_ < data2.dist_to_tar_obj_) ? true : false;
    } else {
      ret = false;
    }
    return (ret);
  }
};


/**
 * @struct FuncCmpObjAssoObjIndex
 * @brief
 */
class FuncCmpObjAssoObjIndex {
public:
  FuncCmpObjAssoObjIndex(const Int32_t obj_idx) {
    obj_index_ = obj_idx;
  }

  bool operator()(const ObjsAssociation& data) const {
    return (data.tar_obj_index_.obj_idx_ == obj_index_);
  }

private:
  Int32_t obj_index_;
};


/**
 * @struct FuncCmpObjAssObjMismatchFlag
 * @brief
 */
class FuncCmpObjAssObjMismatchFlag {
public:
  FuncCmpObjAssObjMismatchFlag(bool flag) {
    mismatch_flag_ = flag;
  }

  bool operator()(const ObjsAssociation& data) const {
    return (data.mismatch_ == mismatch_flag_);
  }

private:
  bool mismatch_flag_;
};


///**
// * @struct FindRelPosFromListByTimestamp
// * @brief
// */
//bool FindRelPosFromListByTimestamp(
//      const Int64_t timestamp, const ad_msg::RelativePosList& pos_list,
//      ad_msg::RelativePos* pos);

} //sensors fusion
} //perception
} //phoenix


#endif
