#ifndef PERCEPTION_SENSORS_FUSION_TRACKER_WRAPPER_H_
#define PERCEPTION_SENSORS_FUSION_TRACKER_WRAPPER_H_

#include "geometry/aabbox2d.h"
#include "geometry/geometry_utils.h"
#include "curve/path.h"
#include "curve/cubic_polynomial_curve1d.h"

#include "sensors_fusion/multi_constructure_classes_repo.h"

// #define ENABLE_TransformVert_2D     (1)

#define ENABLE_POSITION_MAKE_UP     (1)   //使能传感器数据的补偿操作。

#define ENABLE_OUTPUT_ACCELERATION     (1)   //加速度输出使能。

#define REMOVE_ACCEL_CALC_OF_SMOOTH_FUN      (1)   //去掉平滑滤波中关于加速度计算以及参与计算的部分

#define DEBUG_AROUND_RADAR_ID  1

#define REMOVE_SMALL_WIDTH_OBJ  0


#define FIX_BUG_FUSION_OBJ_RECOIL   1  //解决融合障碍物莫名后退的问题        


#define USE_JOINT_ID_FOR_FUSION_ID_MODE       (1)   //是否使用传感器ID拼接组合成融合ID




/**
 * @brief 用来控制前向毫米波雷达的有效宽度的比例，经过实际测试，发现前向毫米波雷达的给出的障碍物宽度过宽，所以设置一个比例
 * 来进行调节
 */
#define FRONT_RADAR_USABLE_WIDTH_SCALE     (0.0)
#define FRONT_RADAR_USABLE_WIDTH_OFFSET    (0.5)



namespace phoenix{
namespace perception{
namespace sensorsfusion{

/**
 * @class ObjsTracker
 * @brief 障碍物跟踪后的结果
 */
class ObjsTracker {
public:

    /**
     * @brief 清空信息
     *
     */
    void Clear();

    /**
     * @brief 设置其它模块输出的位置信息列表到本地
     * param[in] pos_list - 目标障碍物的位置信息列表
     *
     */
    void SetRelativePosList(const ad_msg::RelativePosList& pos_list);

    /**
     * @brief 设置其它模块输出的位置信息到本地
     * param[in] pos_list - 目标障碍物的位置信息
     *
     */
    void SetCurrentRelPos(const ad_msg::RelativePos& pos);

    /**
     * @brief 根据sensor_id添加障碍物的信息到列表中
     * param[in] sensor_id - 传感器ID信息
     * param[in] src_obj - 目标障碍物的位置信息
     * param[out]
     *
     */
    ObjIndex AddNewObj(const Int32_t sensor_id, const ObjInfosBase& src_obj);

    /**
     * @brief 将新增加的障碍物跟踪列表进行时间及空间同步(对齐到时间流中)。清理KD-Tree
     * param[in] timestamp - 当前时间戳
     * return true - 成功； false - 失败
     *
     */
    bool PreProcessForAddingNewObj(const Int64_t timestamp);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void PostProcessForAddingNewObj();

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void MatchToTrackedObj(
        const MatchingParams& param,
        const ObjIndex& src_obj_idx,
        const ObjInfosBase& src_obj);

    /**
     * @brief 为跟踪列表中的每个obj找到最匹配的OBJ的信息
     */
    void FindBestMatchedObj(
        const Int32_t sen_id, const MatchingParams& param);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void UpdateTrackList(const ObjsFilterDataSource& data_source);


    void ModifyAroundRadarObjConfidence();


    void CheckObjLife();

    /**
     * @brief 获取跟踪列表中的障碍物数据（调试用）
     * @param[out] 跟踪列表中的障碍物数据
     */
    void GetObstacleTrackedInfoList(
        ad_msg::ObstacleTrackedInfoList* obj_list) const;

    /**
     * @brief 获取过滤及融合后的障碍物列表
     * @param[out] 过滤及融合后的障碍物列表
     */
    void GetObstaclesLists(ad_msg::ObstacleList* obj_list) const;

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    Int32_t getTrackedObjectsNums() const;


    /**
     * @brief 获取车身参数
     * 
     */
    void GetVehicleParams(struct EgoVehicleParams& vehicle_params);

    ObjsTracker() = default;
    ObjsTracker(const struct EgoVehicleParams& vehicle_params);

    ~ObjsTracker() = default;

private:

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    Int32_t GetListIdBySensorId(const Int32_t sen_id) const;

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    Int32_t GetSensorIdByListId(const Int32_t list_id) const;

    /**
     * @brief 移除障碍物跟踪列表中过期的障碍物
     */
    void DeleteErasedObjFromTrackList();

    /**
     * @brief 更新跟踪列表里面的障碍物信息
     */
    void UpdatePrevObjTrackedList(
       const Float32_t time_elapsed_ms,
        const common::Matrix<Float32_t, 3, 3>& mat_conv);

    /**
     * @brief 添加伙伴关系
     */
    void AddPartnerRelationship(
        const MatchingParams& param,
        const ObjIndex& src_obj_idx,
        const ObjInfosBase& src_obj,
        const common::StaticVector<MatchingInfos, MAX_PARTNER_NUM>& matched_obj_idxs);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void SortObjAssoList(const Int32_t list_id);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void SetBestMatchedObj(const Int32_t list_id);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void UpdateMatchedObj(
        const Int32_t sen_id, const Int32_t list_id, const MatchingParams& param);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void UpdateUnmatchedObj(
        const Int32_t sen_id, const Int32_t list_id, const MatchingParams& param);


    /**
     * @brief 当通过ID匹配上之后，对sen_id类型的传感器数据partner_obj_sen，判断其是否与tracked_obj中非sen_id的传感器数据物理上匹配，
     * 
     * 
     * @param tracked_obj 
     * @param tracked_obj_sen 
     * @param partner_obj_sen 
     * @param sen_id 
     * @return true 
     * @return false 
     */
    bool IsValidMatchedForObjPartnerAndObjsTracked(ObjsTracked& tracked_obj, SensorsRef& partner_obj_sen, const MatchingParams &param, const Int32_t sen_id);


    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void MatchObjAgainByIdWhenUnmatched(
        const Int32_t sen_id, const MatchingParams& param,
        const common::StaticVector<Int32_t, 64>& unmatched_objs);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void EstimateObjPos(
        const Int32_t duration,
        const SensorsRef& tracked_obj,
        const SensorsRef& sensor_obj,
        Float32_t* x_est, Float32_t* y_est,
        Float32_t* vx_est, Float32_t* vy_est);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void EraseExpiredObj();

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void MergeTrackedObjs();

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void UpdateObjConfidence();

    /**
     * @brief   更新融合障碍物Id
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void UpdataObjFusionId();

    /**
     * @brief   根据前视一体机路沿过滤障碍物
     * param[in]    源数据
     * param[out]
     */
    void CurbFiltObj(const ObjsFilterDataSource& data_source);

    /**
     * @brief   将车道线/路沿从笛卡尔坐标系转换为Trenet坐标系
     * param[in]    车道线/路沿数据
     * param[in]    Trenet坐标系下的路径
     * param[out]   
     */
    bool ConstructPath(ad_msg::LaneMarkCamera& lane_curb, common::Path& path);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void GroupObjs(const ObjsFilterDataSource& data_source); // TODO:缺高精度地图

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void FindRoadBoundaryWidth(
        const common::StaticVector<driv_map::RoadBoundary,
        common::Path::kMaxPathPointNum>& road_boundary,
       const Float32_t s_ref, Float32_t* left_width, Float32_t* right_width);

    /**
     * @brief
     * param[in]
     * param[in]
     * param[out]
     *
     */
    void SmoothTrackedObjs();

    /**
     * @brief 针对左侧雷达和环视相机的融合结果进行平滑
     * 
     */
    void SmoothAroundLeftTrackedObjs();

    /**
     * @brief 针对右侧雷达和环视相机的融合结果进行平滑
     * 
     */
    void SmoothAroundRightTrackedObjs();

    /**
     * @brief 打印障碍物跟踪列表中的某个障碍物数据（调试用）
     * @param[in] obj_index - 障碍物在列表中的索引
     * @param[in] obj - 障碍物数据
     */
    void ShowTrackedObject(const Int32_t obj_index, const ObjsTracked& obj) const;

    /**
     * @brief 打印障碍物跟踪列表中的障碍物数据（调试用）
     */
    void ShowTrackedObjectsList() const;

private:
    struct EgoVehicleParams vehicle_params_;

    ad_msg::RelativePosList rel_pos_list_;
    ad_msg::RelativePos current_rel_pos_;
    ObjTrackedList tracked_objects_;

    // 用于加快最近OBJ查询等功能的k-d树
    common::AABBoxKDTree2d<ObjIndex,
        MAX_TRACKED_OBJ_NUM, 16> tracked_objects_kdtree_;
    // used for k-d tree
    common::StaticVector<common::AABBoxKDTreeNodeAssociation, 64>
        kdtree_nodes_stack_;

    ObjAssoPoolType obj_ass_pool_;

};



} //sensors fusion
} //perception
} //phoenix

#endif

