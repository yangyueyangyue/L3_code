//
#include "task_recv_sensors_infos_and_process.h"

#include "utils/com_utils.h"
#include "utils/log.h"
#include "vehicle_model_wrapper.h"
#include "communication/shared_data.h"
#include "pc/util.h"

#include "work/work_sensors_fusion.h"

namespace phoenix {
namespace perception {
namespace sensorsfusion{

TaskRecvSensorsInfosAndProcessor::TaskRecvSensorsInfosAndProcessor(framework::Task *manager) :
    Task(framework::TaskId::TASK_ID_RECV_SENSORS_FUSION_DATA, "Sensors fusion processor", manager),
    thread_running_flag_sensors_fusion_(false)
{

}

TaskRecvSensorsInfosAndProcessor::~TaskRecvSensorsInfosAndProcessor()
{
    Stop();
}

bool TaskRecvSensorsInfosAndProcessor::Start()
{
    if (!thread_running_flag_sensors_fusion_) {
        LOG_INFO(3) << "Start sensors fusion processor.";
        thread_running_flag_sensors_fusion_ = true;
        thread_sensors_fusion_ = boost::thread(boost::bind(&TaskRecvSensorsInfosAndProcessor::ThreadSensorsFusionProcessor, this));
        LOG_INFO(3) << "Start sensors fusion thread....[OK]";
    }
    return true;
}

bool TaskRecvSensorsInfosAndProcessor::Stop()
{
    //Stop sensors fusion thread
    if (thread_running_flag_sensors_fusion_) {
        LOG_INFO(3) << "Stop sensors fusion thread...";

        thread_running_flag_sensors_fusion_ = false;

        bool ret = thread_sensors_fusion_.timed_join(boost::posix_time::seconds(2));

        if (false == ret) {
            LOG_ERR << "Failed to wait thread \"Sensors fusion processor\" exiting. ";
            LOG_INFO(3) << "Stop sensors fusion thread...[NG]";
        } else {
            LOG_INFO(3) << "Stop sensors fusion thread...[OK]";
        }
    }
    return true;
}

void TaskRecvSensorsInfosAndProcessor::ThreadSensorsFusionProcessor()
{
    LOG_INFO(3) << "Sensors fusion thread ... [Started]]";

    framework::Dormancy dormancy(100);
    while (thread_running_flag_sensors_fusion_) {
        common::Stopwatch performance_timer;

        Int32_t status = work_sensors_fusion_.DoWork();

        Int32_t time_elapsed = performance_timer.Elapsed();
        
        phoenix::ad_msg::ObstacleList obstacle_fusion;
        phoenix::framework::SharedData* shared_data = phoenix::framework::SharedData::instance();
        shared_data->GetObstaclesFusionList(&obstacle_fusion);
        //printf("num:%d\n", obstacle_fusion.obstacle_num);
        framework::MessageRecvFusionObstacleList sensors_fusion_res(&obstacle_fusion);
        Notify(sensors_fusion_res);

#if ENABLE_PUBLISH_PERCEPTION_DEBUG
        // Add Perception debug BY wzf for Ros
        framework::MessagePerceptionDebug perception_debug(
          &(work_sensors_fusion_.GetPerceptionDebug()), status, time_elapsed);
        Notify(perception_debug);
#endif

        dormancy.Sleep();
    }
}

bool TaskRecvSensorsInfosAndProcessor::HandleMessage(const framework::Message &msg, framework::Task *sender)
{
    bool ret = true;

    switch (msg.id()) {
    case framework::MSG_ID_RECV_SENSORS_FUSION_DATA:
    {

    }
        break;
    default:
        ret = false;
        break;
    }

    return ret;
}




}  // namespace
}  // namespace perception
}  // namespace phoenix
