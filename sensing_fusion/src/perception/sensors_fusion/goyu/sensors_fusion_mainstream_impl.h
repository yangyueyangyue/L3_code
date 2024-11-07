#ifndef PERCEPTION_SENSORS_FUSION_GOYU_OBJS_FILTER_IMPL_H_
#define PERCEPTION_SENSORS_FUSION_GOYU_OBJS_FILTER_IMPL_H_

#include "sensors_fusion/multi_constructure_classes_repo.h"
#include "sensors_fusion/sensors_fusion_mainstream_interface.h"
#include "tracker.h"

namespace phoenix{
namespace perception{
namespace sensorsfusion{
namespace goyu{


/**
 * @class ObjFilterImpl
 * @brief 过滤感知模块不稳定的障碍物，
 *        对相机及毫米波的障碍物数据进行融合及跟踪
 */
class SensorsFusionMainstreamImpl : public ISensorsFusionMainstream{
public:
   SensorsFusionMainstreamImpl();
   ~SensorsFusionMainstreamImpl();

   /**
    * @brief 实现障碍物融合算法流程
    */
   virtual Int32_t ProcessSensorsFusionPipeline(const ObjsFilterDataSource& recv_sensors_data) override;

   /**
    * @brief 清除内部数据
    */
   virtual void Clear() override;  


   /**
    * @brief 获取过滤及融合后的障碍物列表
    * @param[out] 过滤及融合后的障碍物列表
    * @return 障碍物列表数量
    */
   virtual Int32_t GetObstaclesLists(ad_msg::ObstacleList* obj_list) override;

   /**
    * @brief 获取跟踪列表中的障碍物数据（调试用）
    * @param[out] 跟踪列表中的障碍物数据
    */
   virtual void GetObstacleTrackedInfoList(
       ad_msg::ObstacleTrackedInfoList* obj_list) override;

   /**
    * @brief 获取跟踪列表中的障碍物数据（调试用）
    * @param[out] 跟踪列表中的障碍物数据
    */
   virtual Int32_t CheckSourceDataAndTrackedInfoList(const ObjsFilterDataSource& recv_sensors_data,
       const ad_msg::ObstacleList& obj_list) override;


protected:
   /**
    * @brief 障碍物列表关联匹配
    */
   virtual void ObstaclesListMatches() override;

   /**
    * @brief 障碍物结果预测
    */
   virtual void ObstaclesPrediction() override;

   /**
    * @brief 障碍物结果跟踪
    */
   virtual void ObstaclesTracking() override;

private:
   /**
    * @brief 导入相机障碍物的结果
    * @param[in] sensor_id 传感器ID
    * @param[in] data_source 传感器数据来源
    * @param[out] time_elapsed_ms 相对之前，经过的时间
    * @return true, 添加成功； false，添加失败
    */
   bool AddObjsListCamera(
       const Int32_t sensor_id, const ObjsFilterDataSource& data_source,
       Int64_t* time_elapsed);

   /**
    * @brief 导入相机障碍物的结果
    * @param[in] sensor_id 传感器ID
    * @param[in] data_source 传感器数据来源
    * @param[out] time_elapsed_ms 相对之前，经过的时间
    * @return true, 添加成功； false，添加失败
    */
   bool AddObjsListMainCamera_80(
       const Int32_t sensor_id, const ObjsFilterDataSource& data_source,
       Int64_t* time_elapsed);

   /**
    * @brief 导入相机障碍物的结果
    * @param[in] sensor_id 传感器ID
    * @param[in] data_source 传感器数据来源
    * @param[out] time_elapsed_ms 相对之前，经过的时间
    * @return true, 添加成功； false，添加失败
    */
   bool AddObjsListMainCamera_150(
       const Int32_t sensor_id, const ObjsFilterDataSource& data_source,
       Int64_t* time_elapsed);

   /**
    * @brief 导入雷达障碍物的结果
    * @param[in] sensor_id 传感器ID
    * @param[in] data_source 传感器数据来源
    * @param[out] time_elapsed_ms 相对之前，经过的时间
    * @return true, 添加成功； false，添加失败
    */
   bool AddObjsListRadar(
       const Int32_t sensor_id, const ObjsFilterDataSource& data_source,
       Int64_t* time_elapsed);

   /**
    * @brief 导入激光雷达障碍物的结果
    * @param[in] sensor_id 传感器ID
    * @param[in] data_source 传感器数据来源
    * @param[out] time_elapsed_ms 相对之前，经过的时间
    * @return true, 添加成功； false，添加失败
    */
   bool AddObjsListLidar(
       const Int32_t sensor_id, const ObjsFilterDataSource& data_source,
       Int64_t* time_elapsed);

   /**
    * @brief 过滤毫米波中不稳定的障碍物
    * @param[in] sensor_id 传感器ID
    * @param[in] time_elapsed_ms 相对之前，经过的时间
    */
   void FilterByRadar(const Int32_t sensor_id, const Float32_t time_elapsed_ms);

   /**
    * @brief 将相机识别的障碍物进行过滤
    * @param[in] sensor_id 传感器ID
    * @param[in] time_elapsed_ms 相对之前，经过的时间
    */
   void FilterByCamera(const Int32_t sensor_id, const Float32_t time_elapsed_ms);

   /**
    * @brief 将相机识别的障碍物进行过滤
    * @param[in] sensor_id 传感器ID
    * @param[in] time_elapsed_ms 相对之前，经过的时间
    */
   void FilterByMainCamera_80(const Int32_t sensor_id, const Float32_t time_elapsed_ms);

   /**
    * @brief 将相机识别的障碍物进行过滤
    * @param[in] sensor_id 传感器ID
    * @param[in] time_elapsed_ms 相对之前，经过的时间
    */
   void FilterByMainCamera_150(const Int32_t sensor_id, const Float32_t time_elapsed_ms);

   /**
    * @brief 将Lidar识别的障碍物进行过滤
    * @param[in] sensor_id 传感器ID
    * @param[in] time_elapsed_ms 相对之前，经过的时间
    */
   void FilterByLidar(const Int32_t sensor_id, const Float32_t time_elapsed_ms);

   /**
    * @brief 过滤感知模块不稳定的障碍物，\n
    *        对相机及毫米波的障碍物数据进行融合及跟踪
    * @param[in] timestamp 当前的时间戳
    * @param[in] data_source 需要进行过滤及融合的毫米波雷达及相机识别的障碍物信息等
    * @return true - 成功，false - 识别
    */
   virtual Int32_t Update(const ObjsFilterDataSource& data_source) override;


private:
   EgoVehicleParams vehicle_params_;

   ad_msg::RelativePosList rel_pos_list_;
   ad_msg::RelativePos current_rel_pos_;

   //TODO：将此类更改为智能指针
   ObjsFromCameraList objs_from_main_forward_cam_;
   ObjsFromCameraList objs_from_main_forward_cam_80_;
   ObjsFromCameraList objs_from_main_forward_cam_150_;
      ObjsFromCameraList objs_from_center_forward_cam_;
         ObjsFromCameraList objs_from_left_side_cam_;
            ObjsFromCameraList objs_from_left_backward_cam_;
               ObjsFromCameraList objs_from_right_side_cam_;
                  ObjsFromCameraList objs_from_right_backward_cam_;

   ObjsFromRadarList objs_from_main_forward_radar_;
      ObjsFromRadarList objs_from_left_side_radar_;
         ObjsFromRadarList objs_from_left_backward_radar_;
            ObjsFromRadarList objs_from_right_side_radar_;
               ObjsFromRadarList objs_from_right_backward_radar_;

   ObjsFromLidarList objs_from_main_forward_lidar_;

   ObjsTracker obj_tracker_;    //TODO: 将其改为指针

};

} // goyu
} // sensors fusion
} // perception
} // phoenix


#endif
