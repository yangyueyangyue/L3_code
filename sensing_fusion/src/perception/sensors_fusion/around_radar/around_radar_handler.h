#ifndef AROUND_RADAR_MANAGER_H_
#define AROUND_RADAR_MANAGER_H_

#include <vector>
#include <map>
#include "radar_tracker.h"
#include "sensors_fusion/sensors_fusion_macros.h"
#include "framework/communication/shared_data.h"

/*
是否采将雷达数据输入到融合模块
*/
#define ENABLE_AROUND_RADAR_INPUT_FUSION  (0)

/*
是否启用预处理
*/
#define ENABLE_AROUND_RADAR_PRE_PROCESS  (0)

/**
 * @brief 让侧雷达跳过融合处理，直接合并到融合结果中，但是要配合
 * ENABLE_AROUND_RADAR_INPUT_FUSION 一起使用
 */
#define ENABLE_AROUND_RADAR_SKIP_FUSION  (0) 

/*
显示每一次预处理消耗的时间
*/
#define ENABLE_SHOW_RADAR_PRE_COST_TIME  (1)

/*
是否启用卡尔曼滤波进行数据预处理
*/
#define ENABLE_AROUND_RADAR_PRE_KF_FILTER (1)


/*
是否开启坐标偏移，由于侧雷达坐标系有可能与其他传感器坐标系不一致，所以有必要开启坐标偏移
*/
#define ENABLE_AROUND_RADAR_COORDINATE_BIAS     (1)






#define OBSTACLE_LEFT_FW_RADAR_X_BIAS_VALUE   (-DISTANCE_BETWEEN_MAINCAM_AND_FRONTWHEEL)//(1.25 - 7.5)
#define OBSTACLE_RIGHT_FW_RADAR_X_BIAS_VALUE   (-DISTANCE_BETWEEN_MAINCAM_AND_FRONTWHEEL)//(1.25 - 7.0)
#define OBSTACLE_LEFT_BW_RADAR_X_BIAS_VALUE   (-DISTANCE_BETWEEN_MAINCAM_AND_FRONTWHEEL)//(-4.35 - (-0.8))//(0.9 - 7.0)
#define OBSTACLE_RIGHT_BW_RADAR_X_BIAS_VALUE   (-DISTANCE_BETWEEN_MAINCAM_AND_FRONTWHEEL)//(0.9 - 6.3)//(0.9 - 7.0)

#define OBSTACLE_LEFT_FW_RADAR_Y_BIAS_VALUE   (0.0)
#define OBSTACLE_RIGHT_FW_RADAR_Y_BIAS_VALUE   (0.0)
#define OBSTACLE_LEFT_BW_RADAR_Y_BIAS_VALUE   (0.0)
#define OBSTACLE_RIGHT_BW_RADAR_Y_BIAS_VALUE   (0.0)



#define   MAX_TRACKERLIST_SIZE    1000
#define   MAX_ERR_RADAR_LIST_SIZE    20



namespace phoenix{
namespace perception{
namespace radar{

/*
*/
using namespace std;

class AroundRadarHandler{
public:
AroundRadarHandler();
~AroundRadarHandler();
void SetRelativePosList(ad_msg::RelativePosList* rel_pos_list);

void UpdateLeftFwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_);
void UpdateRightFwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_);
void UpdateLeftBwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_);
void UpdateRightBwRadarList(ad_msg::ObstacleRadarList& radar_list, ad_msg::Chassis& chassis_);

void UpdateRadarList(ad_msg::ObstacleRadarList& radar_list, AroundRadarType radar_type, std::vector<ErrorRadarData>& err_radar_list, ad_msg::Chassis& chassis_);

/*
* 针对挂角问题做的特殊处理
*/
void UpdateErrRadarDataList(ad_msg::ObstacleRadar & obstacle, std::vector<size_t> & invalid_ind_vec, std::vector<ErrorRadarData>& err_radar_list, ad_msg::RelativePos & rel_pos, int obj_idx);


void CombineAroundRadarToFusedListDirectly(ad_msg::ObstacleRadarList& left_fw_radar_list,
                                        ad_msg::ObstacleRadarList& right_fw_radar_list,
                                        ad_msg::ObstacleRadarList& left_bw_radar_list,
                                        ad_msg::ObstacleRadarList& right_bw_radar_list,
                                        ad_msg::ObstacleList & objs_sensors_fusion_res
                                        );

void Clock(Int64_t timestamp);

void CorrectCoordinate(ad_msg::ObstacleRadar & obstacle, AroundRadarType radar_type);

private:

void CollectValidObjects(ad_msg::ObstacleRadarList& radar_list, std::vector<RadarTracker> & tracker_list);

bool IsRadarDataValid(ad_msg::ObstacleRadar & obstacle, AroundRadarType radar_type, ad_msg::Chassis& chassis_);

void AssociateAndFilter(ad_msg::ObstacleRadar & obstacle, std::map<size_t, Int32_t> & idx2id_map ,std::map<Int32_t, size_t> & id2idx_map , 
          std::vector<RadarTracker> & tracker_list, AroundRadarType radar_type, Int64_t timestamp);

void HandlePeriodicTask(std::map<size_t, Int32_t> & idx2id_map ,std::map<Int32_t, size_t> & id2idx_map , 
          std::vector<RadarTracker> & tracker_list, AroundRadarType radar_type, Int64_t timestamp);

void CombineAroundRadarListToFusedList(ad_msg::ObstacleRadarList& around_radar_list,
                                       ad_msg::ObstacleList & objs_sensors_fusion_res);

void CheckRadarDataStatus(ad_msg::ObstacleRadar & obstacle, AroundRadarType around_radar_type);

#if 0
std::vector<std::shared_ptr<RadarTracker> > left_fw_radar_list_ ;
std::vector<std::shared_ptr<RadarTracker> > right_fw_radar_list_ ;
std::vector<std::shared_ptr<RadarTracker> > left_bw_radar_list_ ;
std::vector<std::shared_ptr<RadarTracker> > right_bw_radar_list_ ;
#endif

std::vector<RadarTracker> left_fw_radar_list_ ;
std::vector<RadarTracker> right_fw_radar_list_ ;
std::vector<RadarTracker> left_bw_radar_list_ ;
std::vector<RadarTracker> right_bw_radar_list_ ;

std::map<size_t, Int32_t> left_fw_idx2id_;
std::map<size_t, Int32_t> right_fw_idx2id_;
std::map<size_t, Int32_t> left_bw_idx2id_;
std::map<size_t, Int32_t> right_bw_idx2id_;

std::map<Int32_t, size_t> left_fw_id2idx_;
std::map<Int32_t, size_t> right_fw_id2idx_;
std::map<Int32_t, size_t> left_bw_id2idx_;
std::map<Int32_t, size_t> right_bw_id2idx_;

ad_msg::RelativePosList* rel_pos_list_;


std::vector<ErrorRadarData> left_fw_err_radar_list_ ;
std::vector<ErrorRadarData> right_fw_err_radar_list_ ;
std::vector<ErrorRadarData> left_bw_err_radar_list_ ;
std::vector<ErrorRadarData> right_bw_err_radar_list_ ;
int  tmp_count_;

framework::PerceptionModuleStatus perception_module_status_;


};




}// end of namespace radar
}// end of namespace perception
}// end of namespace phoenix

#endif




