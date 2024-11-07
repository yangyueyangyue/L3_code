#ifndef VISUAL_CONTROL_MANAGER_H_
#define VISUAL_CONTROL_MANAGER_H_

#include <vector>
#include <map>
#include "msg_common.h"
#include "msg_obstacle.h"
#include "utils/log.h"
#include "framework/communication/shared_data.h"
#include "math/math_utils.h"

#define  VEHICLE_WIDTH   (2.5)  



/*
是否采将环视数据输入到融合模块
*/
#define ENABLE_VISUAL_CONTROL_INPUT_FUSION  (0)

/*
是否将前环视相机数据输入到融合模块
*/
#define ENABLE_CENTER_FORWARD_VISUAL_CONTROL_INPUT_FUSION  (1)

/*
是否启用预处理
*/
#define ENABLE_VISUAL_CONTROL_PRE_PROCESS  (1)

/**
 * @brief 让环视跳过融合处理，直接合并到融合结果中，但是要配合
 * ENABLE_AROUND_RADAR_INPUT_FUSION 一起使用
 */
#define ENABLE_VISUAL_CONTROL_SKIP_FUSION  (0) 

/*
显示每一次预处理消耗的时间
*/
#define ENABLE_SHOW_VISUAL_PRE_COST_TIME  (0)

/*
是否启用卡尔曼滤波进行数据预处理
*/
#define ENABLE_VISUAL_CONTROL_PRE_KF_FILTER (0)

#define   MAX_TRACKERLIST_SIZE    1000

#define VISUAL_CONTROL_PRE_LOG_LEVEL  7

/**
 * @brief 设置环视的纵向和横向速度为0
 * 
 */
#define SET_VISUAL_CONTROL_X_V_TO_ZERO     (1)

#define SET_VISUAL_CONTROL_Y_V_TO_ZERO     (1)



namespace phoenix{
namespace perception{
namespace visual{





/*根据视觉处理器不同的FOV进行过滤*/
//前向环视相机
#define CENTER_FW_CAMERA_CV_X_MIN 4.0
#define CENTER_FW_CAMERA_CV_X_MAX 50.0
#define CENTER_FW_CAMERA_CV_Y_MIN -14.0
#define CENTER_FW_CAMERA_CV_Y_MAX 14.0

//左前向环视相机
#define LEFT_SIDE_CAMERA_CV_X_MIN -35.0
#define LEFT_SIDE_CAMERA_CV_X_MAX 15.0
#define LEFT_SIDE_CAMERA_CV_Y_MIN 0.0 //(VEHICLE_WIDTH/2)
#define LEFT_SIDE_CAMERA_CV_Y_MAX 15.0

//右前向环视相机
#define RIGHT_SIDE_CAMERA_CV_X_MIN -35.0
#define RIGHT_SIDE_CAMERA_CV_X_MAX 15.0
#define RIGHT_SIDE_CAMERA_CV_Y_MIN -15.0
#define RIGHT_SIDE_CAMERA_CV_Y_MAX 0.0 //-(VEHICLE_WIDTH/2)

//左后向环视相机
#define LEFT_BW_CAMERA_CV_X_MIN -120.0
#define LEFT_BW_CAMERA_CV_X_MAX 0.0
#define LEFT_BW_CAMERA_CV_Y_MIN -4.0
#define LEFT_BW_CAMERA_CV_Y_MAX 20.0

//右后向环视相机
#define RIGHT_BW_CAMERA_CV_X_MIN -120.0
#define RIGHT_BW_CAMERA_CV_X_MAX 0.0
#define RIGHT_BW_CAMERA_CV_Y_MIN -20.0
#define RIGHT_BW_CAMERA_CV_Y_MAX 4.0

enum VisualControlType{
CENTER_FW_CAMERA_TYPE=0,
LEFT_SIDE_CAMERA_TYPE,
RIGHT_SIDE_CAMERA_TYPE,
LEFT_BW_CAMERA_TYPE,
RIGHT_BW_CAMERA_TYPE,
};


/*
*/
using namespace std;

class VisualControlHandler{
public:
VisualControlHandler();
~VisualControlHandler();
void UpdateCenterFwCameraList(ad_msg::ObstacleCameraList& camera_list);
void UpdateLeftSideCameraList(ad_msg::ObstacleCameraList& camera_list);
void UpdateRightSideCameraList(ad_msg::ObstacleCameraList& camera_list);
void UpdateLeftBwCameraList(ad_msg::ObstacleCameraList& camera_list);
void UpdateRightBwCameraList(ad_msg::ObstacleCameraList& camera_list);

void CheckCameraDataStatus(ad_msg::ObstacleCamera & obstacle, VisualControlType visual_control_type);

private:
void UpdateCameraList(ad_msg::ObstacleCameraList& camera_list,VisualControlType visual_control_type);
bool IsCameraDataValid(ad_msg::ObstacleCamera & obstacle, VisualControlType visual_control_type);

framework::PerceptionModuleStatus perception_module_status_;

};


}// end of namespace visual
}// end of namespace perception
}// end of namespace phoenix

#endif