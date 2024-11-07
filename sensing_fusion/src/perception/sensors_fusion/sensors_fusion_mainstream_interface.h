#ifndef PERCEPTION_SENSORS_FUSION_SENSORS_FUSION_MAINSTREAM_INTERFACE_H_
#define PERCEPTION_SENSORS_FUSION_SENSORS_FUSION_MAINSTREAM_INTERFACE_H_

#include "utils/log.h"
#include "ad_msg.h"
#include "multi_constructure_classes_repo.h"

namespace phoenix{
namespace perception{
namespace sensorsfusion
{
/**
 * @struct ISensorsFusionMainstream
 * @brief 业务抽象接口。表示多传感器融合业务主流程。将通过此来抽象Goyu与DFCV方法
 */
class ISensorsFusionMainstream{
public:
    ISensorsFusionMainstream();
    virtual ~ISensorsFusionMainstream();

    /**
     * @brief 初始化多传感器融合参数
     * @return true - 初始化成功； false - 初始化失败
     */
    virtual bool Initial();

    /**
     * @brief 感知融合主流程
     */
    virtual Int32_t ProcessSensorsFusionPipeline(const ObjsFilterDataSource& recv_sensors_data);


    /**
     * @brief 获取过滤及融合后的障碍物列表
     * @param[out] 过滤及融合后的障碍物列表
     * @return 障碍物列表数量
     */
    virtual int32_t GetObstaclesLists(ad_msg::ObstacleList*);

    /**
     * @brief 获取跟踪列表中的障碍物数据（调试用）
     * @param[out] obj_list - 跟踪列表中的障碍物数据
     */
    virtual void GetObstacleTrackedInfoList(
        ad_msg::ObstacleTrackedInfoList* );

    virtual Int32_t CheckSourceDataAndTrackedInfoList(const ObjsFilterDataSource& recv_sensors_data,
    const ad_msg::ObstacleList& obj_list);

    /**
     * @brief 结束并回收相关资源
     * @return true 终止成功；false 终止失败
     */
    virtual bool Terminate();

    /**
     * @brief 清除内部数据
     */
    virtual void Clear() = 0;


protected:

    /**
     * @brief 传感器数据来源
     * @return 当前引入传感器数量（按位置进行区分）
     */
    virtual Int32_t CollectSensorsInfos() const;

    /**
     * @brief 传感器数据预处理(含时间对其、运动补偿、单传感器滤波、异常点点提取与过滤等)
     */
    virtual void SensorsInfosPreprocess();

    /**
     * @brief 障碍物列表关联匹配
     */
    virtual void ObstaclesListMatches() = 0;

    /**
     * @brief 障碍物结果预测
     */
    virtual void ObstaclesPrediction() = 0;

    /**
     * @brief 障碍物结果跟踪
     */
    virtual void ObstaclesTracking() = 0;

    /**
     * @brief 获取车身信息(自车车速、方向盘转角等信息)
     */
    virtual void GetChassisInfos();

    /**
     * @brief 获取高精度地图信息(各车道线信息、)
     */
    virtual void GetHAMInfos();

    /**
     * @brief 更新过滤感知模块不稳定的障碍物，\n
     *        对相机及毫米波的障碍物数据进行融合及跟踪
     * @return true - 成功，false - 识别
     */
    virtual Int32_t Update(const ObjsFilterDataSource& data_source) = 0;

private:
   ISensorsFusionMainstream(const ISensorsFusionMainstream&) = delete;
   ISensorsFusionMainstream& operator=(const ISensorsFusionMainstream&) = delete;


};

} // namespace sensors fusion

} // namespace perception

} // namespace phoenix

#endif 
