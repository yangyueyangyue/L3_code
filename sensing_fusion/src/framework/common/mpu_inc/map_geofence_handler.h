#ifndef MAP_GEOFENCE_HANDLER_H_ 
#define MAP_GEOFENCE_HANDLER_H_

#include "MpuState.pb.h"

#include "ExpWrapperComm.h"


namespace exp_map_t
{

class MapGeofenceHandler{
public:
    MapGeofenceHandler(){}
    ~MapGeofenceHandler(){}
    static void ConvertMpuStateToMapGeofenceInfo(const MpuState::MonitorMpuState & mpu_state, stMapGeofenceInfo & geofence_info){
        geofence_info.m_uTimeStamp = mpu_state.timestamp();
        Reason reason = Reason::NONE;
        geofence_info.m_currentInReasons.clear();
        if(mpu_state.single() == MpuState::EMPUState::MPUOK && mpu_state.trajvalue().geofencestate() == MpuState::GeofenceType::GeofenceTypeInTunnel){
            geofence_info.m_bInGeofence = false;
            /**
             * @brief 这里设置为1，只是作为一个初始化的值，融合模块只是判断是在隧道中。其他的并不关心。
             * 
             */
            reason = Reason::TUNNEL;
            geofence_info.m_currentInReasons.insert(pair<Reason, uint32_t>(reason, 1)); 
        }
    }

};


















}// end of namespace exp_map_t






#endif
