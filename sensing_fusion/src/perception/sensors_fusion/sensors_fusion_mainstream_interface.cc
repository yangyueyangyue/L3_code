#include "utils/log.h"
#include "sensors_fusion_mainstream_interface.h"
#include <iostream>

namespace phoenix {
namespace perception{
namespace sensorsfusion{

ISensorsFusionMainstream::ISensorsFusionMainstream()
{
    LOG_WARN << "[WorkPipeline on Base Class]Constructure function.";
 }

ISensorsFusionMainstream::~ISensorsFusionMainstream()
{
    LOG_WARN << "[WorkPipeline on Base Class]Deconstructure function.";
}

bool sensorsfusion::ISensorsFusionMainstream::Initial()
{
    LOG_WARN << "[WorkPipeline on Base Class]Initial the process.";
    return false;
}

Int32_t sensorsfusion::ISensorsFusionMainstream::ProcessSensorsFusionPipeline(const ObjsFilterDataSource& recv_sensors_data)
{
    LOG_INFO(2) << "[WorkPipeline]The main processing begin.";

    //各自实现具体流程: 将融合主流程加入到这里
    if (!Initial())
        return 0;

    if (CollectSensorsInfos() < 1)
        return 0;

    GetChassisInfos();

    GetHAMInfos();

    SensorsInfosPreprocess();

    ObstaclesListMatches();

    ObstaclesTracking();

    ObstaclesPrediction();

    LOG_INFO(2) << "[WorkPipeline]The main processing end.";
    return 0;
}

bool sensorsfusion::ISensorsFusionMainstream::Terminate()
{
    LOG_WARN << "[WorkPipeline on Base Class]Terminate the process.";
    return false;
}

Int32_t sensorsfusion::ISensorsFusionMainstream::CollectSensorsInfos() const
{
    LOG_WARN << "[WorkPipeline on Base Class]Collect the God blessed sensors' infos.";
    return 0;
}


void sensorsfusion::ISensorsFusionMainstream::SensorsInfosPreprocess()
{
    LOG_WARN << "[WorkPipeline on Base Class]The sensors results preprocess.";
}

void sensorsfusion::ISensorsFusionMainstream::GetChassisInfos()
{
    LOG_WARN << "[WorkPipeline on Base Class]Get the informations of Chassis.";
}

void sensorsfusion::ISensorsFusionMainstream::GetHAMInfos()
{
    LOG_WARN << "[WorkPipeline on Base Class]Get the informations of HAM.";
}

void ISensorsFusionMainstream::GetObstacleTrackedInfoList(ad_msg::ObstacleTrackedInfoList *)
{
    LOG_WARN << "[WorkPipeline on Base Class]Get the obstacles tracked information list.";
}


Int32_t ISensorsFusionMainstream::CheckSourceDataAndTrackedInfoList(const ObjsFilterDataSource& recv_sensors_data,
    const ad_msg::ObstacleList& obj_list)
{
    LOG_WARN << "[WorkPipeline on Base Class]Check the obstacles tracked information list.";
    return -1;
}


Int32_t sensorsfusion::ISensorsFusionMainstream::GetObstaclesLists(ad_msg::ObstacleList*)
{
    LOG_WARN << "[WorkPipeline on Base Class]Get the obstacles list info.";
    return -1;
}



} // namespace sensors fusion
} // namespace perception
} //phoenix


