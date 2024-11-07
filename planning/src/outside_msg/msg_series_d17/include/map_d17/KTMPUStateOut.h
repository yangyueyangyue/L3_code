#ifndef __KT_MPUSTATE_OUT_H__
#define __KT_MPUSTATE_OUT_H__

#include "SystemBase.h"
#include "SystemMacro.h"
#include <mutex>

namespace mpu_t
{
    enum class EMPUState : uint32
    {
        MPUERR = 0, // 失败
        MPUOK = 1,  // 成功
    };

    const uint32 LimitSpeedDefault = 0xFF;      // 限速默认值
    const uint32 FussDefault = 0;               // 融合定位置信度默认值
    const float32 SlopeDefault = 0.0f;          // 坡度默认值
    const float32 CurvatureDefault = 100000.0f; // 曲率默认值

    // 20231229:根据文档修改如下
    // 0x0：无 0x1：实时模式 0x2：事件模式 0x3：远程实时模式 0x4：远程事件模式
    // 旧定义值 0x0:关闭模式 0x1：事件模式   0x2：实时模式   0x3:强制事件模式    0x4：强制实时模式  0xFF: 无效值
    enum class EMPUEventMode : uint32
    {
        Close = 0x00,
        RealTime = 0x01,
        Event = 0x02,
        FroceRealTime = 0x03,
        FroceEvent = 0x04,
        Default = 0xFF, // 无效值
    };

    enum class MonitorGeofenceType : uint16
    {
        // 20231214 降级规则: 正常:0, 2, 3, 4, 6, 0xFF, 5     降级A：7    降级D：1，8
        MonitorGeofenceTypeRamp150 = 0,         // 即将进入匝道前150m     Reason:LaneConnectRamp offset < 150m
        MonitorGeofenceTypeInRamp = 1,          // 完全进入匝道           Reason:Ramp
        MonitorGeofenceTypeTunnel200 = 2,       // 即将进入隧道前200m     Reason:
        MonitorGeofenceTypeInTunnel = 3,        // 隧道中                Reason:Tunnel
        MonitorGeofenceTypeOutTunnel = 4,       // 出隧道                 Reason:
        MonitorGeofenceTypeExceCurvature = 5,   // 曲率过大（弯道半径<250m）Reason:CurvatureOverrun
        MonitorGeofenceTypeIn = 6,              // 在地理围栏范围内
        MonitorGeofenceTypeLaneEndMore100M = 7, // 车道结束>100m
        MonitorGeofenceTypeLaneEndLess100M = 8, // 车道结束<=100m
        MonitorGeofenceTypeDefault = 0xFF,      // 默认值
    };

    enum EMpuGeofenceBit // MpuGeofenceBits对应位的说明
    {
        EMGBFlg = 0,         // 后续值是否有效
        EMGBInGeofence,      // 是否在地理围栏内
        EMGBRamp150,         // 即将进入匝道前150m     Reason:LaneConnectRamp offset < 150m
        EMGBInRamp,          // 完全进入匝道           Reason:Ramp
        EMGBTunnel200,       // 即将进入隧道前200m     Reason:
        EMGBInTunnel,        // 隧道中                Reason:Tunnel
        EMGBOutTunnel,       // 出隧道                 Reason:
        EMGBExceCurvature,   // 曲率过大（弯道半径<250m）Reason:CurvatureOverrun
        EMGBLaneEndMore100M, // 车道结束>100m
        EMGBLaneEndLess100M, // 车道结束<=100m
        EMGBMaxHead = 16,    // 最大前缀

        // 后续为原geofence_t::Reason值 + EMGBMaxHead
        // RAMP = 1,                   // 匝道
        // LANECONNECTIONRAMP = 2,     // 连接到匝道的车道
        // TRAFFICLINEMISSING = 3,     // 车道线缺失
        // LANENARROWORWIDE = 4,       // 车道过窄/过宽
        // LANEENDING = 5,             // 车道结束
        // EMERGENCYLANE = 6,          // 紧急车道
        // NOSEPARATEAREA = 7,         // 非隔离区域
        // TRAFFICLIGHT = 8,           // 交通灯
        // HIGHWAYEND = 9,             // 高速结束
        // NOTOPEN = 10,               // 未开放道路
        // CURVATUREOVERRUN = 11,      // 曲率过大
        // SLOPEOVERRUN = 12,          // 坡度过大
        // DIVIDEROAD = 13,            // 分叉道路
        // TUNNEL = 14,                // 隧道
        // NoPathMatchArea = 16,       ///< 配准失败区域
        // RoadDivergence = 31,        ///< 分流点
        // RoadConfluence = 32,        ///< 汇流点
        // Bridge = 33,                ///< 桥梁
        // RampArea = 34,              ///< 下匝道口(无论规划是否下匝道)
        // PlanPathRampArea = 35,      ///< 规划路径上的下匝道口
        // PlanPathHighWayEnd = 36,    ///< 规划路径上高速公路终点
        // UpRampArea = 37,            ///< 上匝道口(无论规划是否上匝道)
        // PlanPathUpRampArea = 38,    ///< 规划路径上的上匝道口
        // TollBooth = 39,             ///< 收费站
        // LeftEmergencyParking = 40,  ///< 车道左侧有紧急停车带
        // RightEmergencyParking = 41, ///< 车道右侧有紧急停车带
        // LeftDivergenceNode = 42,    ///< 车道左侧有分流口
        // RightDivergenceNode = 43,   ///< 车道右侧有分流口
        // LeftConfluenceNode = 44,    ///< 车道左侧有合流口
        // RightConfluenceNode = 45,   ///< 车道右侧有合流口
        EMGBMaxBit = 64, // 最大bit位，地理围栏原因+EMGBMaxHead只能小于该值
    };

    enum class MonitorPlanningType : uint32
    {
        MonitorPlanningTypeOut = 0,         // 是否在规划路径上: 0 不在
        MonitorPlanningTypeIn = 1,          // 是否在规划路径上: 1 在
        MonitorPlanningTypeDefault = 0x0FF, // 默认值
    };

    enum class MonitorHighWayType : uint32
    {
        MonitorHighWayTypeOut = 0,        // 是否在高速上: 0 不在
        MonitorHighWayTypeIn = 1,         // 是否在高速上: 1 在
        MonitorHighWayTypeDefault = 0xFF, // 是否在高速上:  默认值
    };

    typedef struct MonitorTraj
    {
        MonitorPlanningType planningState = MonitorPlanningType::MonitorPlanningTypeDefault;
        MonitorGeofenceType geofenceState = MonitorGeofenceType::MonitorGeofenceTypeDefault;
    } stMpuTraj;

    typedef struct MonitorState
    {
        bool AntennaConnectStatus = false;
        bool MpuMCURunStatus = false;
        bool MpuSOCRunStatus = false;
        bool F9KDemarcateStatus = false;
        bool MpuEngineRunStatus = false;
        bool IsHasMapData = false;
        bool VehicleCANData = false;
        bool FrontCameraLineStatus = false;
    } stMonitorState;

    typedef struct MonitorSingle
    {
        bool F9KGnssWeakenStatus = false;
        bool F9kRTKStatus = false;
        bool F9kImuInitStatus = false;
        bool FrontCameraConfidence = false;
        bool FusiionConfidenceStatus = false;
    } stMonitorSingle;

    typedef struct MonitorMpuVersionIn
    {
        std::string MpuVersion;     // Mpu版本: APP + 地图引擎 + SOC
        std::string MapDataVersion; // 地图数据版本
        std::string MCUVersion;     // MCU版本：MCU + 硬件版本
    } stMonitorMpuVersionIn;

    enum class EMPUEvent // 表示事件对应的bit
    {
        StateErr = 0,          // Mpu状态异常MpuState==0
        TrajErr = 1,           // 规划异常 Traj==0
        SingleErr = 2,         // 信号异常 Single==0
        CameraLose = 3,        // 相机信号丢失	m_Cameraflag == false
        VSpeedLose = 4,        // 轮速信号丢失	m_Speedflag == false
        VGearLose = 5,         // 档位信号丢失	m_Gearflag == false
        VSteerLose = 6,        // 方向盘信号丢失	m_Steerflag == false
        MapEngineLose = 7,     // 地图引擎丢失	m_MapEngineflag == false
        MapDataErr = 8,        // 地图数据	m_MapEngineVersionflag == false
        MCUErr = 9,            // MCU状态	m_MCUflag == false
        SOCErr = 10,           // SOC状态	m_SOCflag == false
        F9KErr = 11,           // F9K信号丢失
        RTKErr = 12,           // RTK信号丢失
        SDData = 13,           // SD数据下发
        PositionTimeOut = 14,  // 定位信号不输出
        CommuDelay = 15,       // MPU通讯异常 向adcu侧发送决策图层数据延迟
        RemindJunction = 16,   // 分合流口	31,32,34,35,37,38
        RemindTunnel = 17,     // 在隧道中
        RemindRamp = 18,       // 在匝道中
        RemindGeoFence = 19,   // 其他地理围栏提醒
        ExceCrossSlope = 20,   // 横坡过大	CrossSlope > 10%
        ExceSlope = 21,        // 纵坡过大	Slope > 10%
        ExceCurvate = 22,      // 曲率过大	MonitorGeofenceTypeExceCurvature == geofenceState
        AbnormSpeedLimit = 23, // 限速异常	前后帧限速差大于20，限速<60,20秒内限速变化超过3次
    };

    enum class EMaskMpuEvent : uint64
    {
        EMaskMEventStateErr = (uint64)(0x1 << (uint32)EMPUEvent::StateErr),   // Mpu状态异常MpuState==0
        EMaskMEventTrajErr = (uint64)(0x1 << (uint32)EMPUEvent::TrajErr),     // 规划异常 Traj==0
        EMaskMEventSingleErr = (uint64)(0x1 << (uint32)EMPUEvent::SingleErr), // 信号异常 Single==0
        EMaskMEventCameraLose = (uint64)(0x1 << (uint32)EMPUEvent::CameraLose), // 相机信号丢失	m_Cameraflag == false
        EMaskMEventVSpeedLose = (uint64)(0x1 << (uint32)EMPUEvent::VSpeedLose), // 轮速信号丢失	m_Speedflag == false
        EMaskMEventVGearLose = (uint64)(0x1 << (uint32)EMPUEvent::VGearLose), // 档位信号丢失	m_Gearflag == false
        EMaskMEventVSteerLose = (uint64)(0x1 << (uint32)EMPUEvent::VSteerLose), // 方向盘信号丢失	m_Steerflag == false
        EMaskMEventMapEngineLose =
            (uint64)(0x1 << (uint32)EMPUEvent::MapEngineLose), // 地图引擎丢失	m_MapEngineflag == false
        EMaskMEventMapDataErr =
            (uint64)(0x1 << (uint32)EMPUEvent::MapDataErr),             // 地图数据	m_MapEngineVersionflag == false
        EMaskMEventMCUErr = (uint64)(0x1 << (uint32)EMPUEvent::MCUErr), // MCU状态	m_MCUflag == false
        EMaskMEventSOCErr = (uint64)(0x1 << (uint32)EMPUEvent::SOCErr), // SOC状态	m_SOCflag == false
        EMaskMEventF9KErr = (uint64)(0x1 << (uint32)EMPUEvent::F9KErr), // F9K信号丢失
        EMaskMEventRTKErr = (uint64)(0x1 << (uint32)EMPUEvent::RTKErr), // RTK信号丢失
        EMaskMEventSDData = (uint64)(0x1 << (uint32)EMPUEvent::SDData), // SD数据下发
        EMaskMEventPositionTimeOut = (uint64)(0x1 << (uint32)EMPUEvent::PositionTimeOut), // 定位信号不输出
        EMaskMEventCommuDelay =
            (uint64)(0x1 << (uint32)EMPUEvent::CommuDelay), // MPU通讯异常 向adcu侧发送决策图层数据延迟
        EMaskMEventRemindJunction = (uint64)(0x1 << (uint32)EMPUEvent::RemindJunction), // 分合流口	31,32,34,35,37,38
        EMaskMEventRemindTunnel = (uint64)(0x1 << (uint32)EMPUEvent::RemindTunnel),     // 在隧道中
        EMaskMEventRemindRamp = (uint64)(0x1 << (uint32)EMPUEvent::RemindRamp),         // 在匝道中
        EMaskMEventRemindGeoFence = (uint64)(0x1 << (uint32)EMPUEvent::RemindGeoFence), // 其他地理围栏提醒
        EMaskMEventExceCrossSlope = (uint64)(0x1 << (uint32)EMPUEvent::ExceCrossSlope), // 横坡过大	CrossSlope > 10%
        EMaskMEventExceSlope = (uint64)(0x1 << (uint32)EMPUEvent::ExceSlope),           // 纵坡过大	Slope > 10%
        EMaskMEventExceCurvate =
            (uint64)(0x1 << (uint32)
                         EMPUEvent::ExceCurvate), // 曲率过大	MonitorGeofenceTypeExceCurvature == geofenceState
        EMaskMEventAbnormSpeedLimit =
            (uint64)(0x1 << (uint32)
                         EMPUEvent::AbnormSpeedLimit), // 限速异常	前后帧限速差大于20，限速<60,20秒内限速变化超过3次
    };

    typedef struct MPUEvent
    {
        bool StateErr = false;        // Mpu状态异常MpuState==0
        bool TrajErr = false;         // 规划异常 Traj==0
        bool SingleErr = false;       // 信号异常 Single==0
        bool CameraLose = false;      // 相机信号丢失	m_Cameraflag == false
        bool VSpeedLose = false;      // 轮速信号丢失	m_Speedflag == false
        bool VGearLose = false;       // 档位信号丢失	m_Gearflag == false
        bool VSteerLose = false;      // 方向盘信号丢失	m_Steerflag == false
        bool MapEngineLose = false;   // 地图引擎丢失	m_MapEngineflag == false
        bool MapDataErr = false;      // 地图数据	m_MapEngineVersionflag == false
        bool MCUErr = false;          // MCU状态	m_MCUflag == false
        bool SOCErr = false;          // SOC状态	m_SOCflag == false
        bool F9KErr = false;          // F9K信号丢失
        bool RTKErr = false;          // RTK信号丢失
        bool SDData = false;          // SD数据下发
        bool PositionTimeOut = false; // 定位信号不输出
        bool CommuDelay = false;      // MPU通讯异常 向adcu侧发送决策图层数据延迟
        bool RemindJunction = false;  // 分合流口	31,32,34,35,37,38
        bool RemindTunnel = false;    // 在隧道中
        bool RemindRamp = false;      // 在匝道中
        bool RemindGeoFence = false;  // 其他地理围栏提醒
        bool ExceCrossSlope = false;  // 横坡过大	CrossSlope > 10%
        bool ExceSlope = false;       // 纵坡过大	Slope > 10%
        bool ExceCurvate = false;     // 曲率过大	MonitorGeofenceTypeExceCurvature == geofenceState
        bool AbnormSpeedLimit = false; // 限速异常	前后帧限速差大于20，限速<60,20秒内限速变化超过3次
        void Default()
        {
            StateErr = false;        // Mpu状态异常MpuState==0
            TrajErr = false;         // 规划异常 Traj==0
            SingleErr = false;       // 信号异常 Single==0
            CameraLose = false;      // 相机信号丢失	m_Cameraflag == false
            VSpeedLose = false;      // 轮速信号丢失	m_Speedflag == false
            VGearLose = false;       // 档位信号丢失	m_Gearflag == false
            VSteerLose = false;      // 方向盘信号丢失	m_Steerflag == false
            MapEngineLose = false;   // 地图引擎丢失	m_MapEngineflag == false
            MapDataErr = false;      // 地图数据	m_MapEngineVersionflag == false
            MCUErr = false;          // MCU状态	m_MCUflag == false
            SOCErr = false;          // SOC状态	m_SOCflag == false
            F9KErr = false;          // F9K信号丢失
            RTKErr = false;          // RTK信号丢失
            SDData = false;          // SD数据下发
            PositionTimeOut = false; // 定位信号不输出
            CommuDelay = false;      // MPU通讯异常 向adcu侧发送决策图层数据延迟
            RemindJunction = false;  // 分合流口	31,32,34,35,37,38
            RemindTunnel = false;    // 在隧道中
            RemindRamp = false;      // 在匝道中
            RemindGeoFence = false;  // 其他地理围栏提醒
            ExceCrossSlope = false;  // 横坡过大	CrossSlope > 10%
            ExceSlope = false;       // 纵坡过大	Slope > 10%
            ExceCurvate = false;     // 曲率过大	MonitorGeofenceTypeExceCurvature == geofenceState
            AbnormSpeedLimit = false; // 限速异常	前后帧限速差大于20，限速<60,20秒内限速变化超过3次
        }
    } stMPUEvent;

    // typedef struct MonitorLaneEnd
    // {
    //     bool valid = false; // 状态是否有效
    //     uint32 dis = 0;     // 距离
    // } stMonitorLandEnd;

    typedef struct MonitorMpuStateIn
    {
        uint64 TimeStamp = 0;
        EMPUState State = EMPUState::MPUERR;  // MPU状态
        EMPUState Single = EMPUState::MPUERR; // MPU信号
        EMPUState Traj = EMPUState::MPUERR;   // Mpu Traj State
        uint32 FussValue = FussDefault;       // 融合置信度(0,1,2,3)
        MonitorTraj TrajValue;                //
        MonitorState StateValue;
        MonitorSingle SingleValue;
        MonitorHighWayType HighWayValue = MonitorHighWayType::MonitorHighWayTypeDefault;
        uint32 LimitSpeedMax = LimitSpeedDefault; // 当前位置当前车道的最大限速
        uint32 LimitSpeedMin = LimitSpeedDefault; // 当前位置当前车道的最小限速
        float32 Slope = SlopeDefault;             // 纵向坡度   后期考虑实时性的数据单独提出来发送
        float32 CrossSlope = SlopeDefault;        // 横向坡度   后期考虑实时性的数据单独提出来发送
        MonitorMpuVersionIn MpuVer;               // Mpu版本信息
        float32 Curvature = CurvatureDefault;     // 曲率
        uint32 MpuEventMode = (uint32)EMPUEventMode::Default; // Mpu事件模式
        bool MpuEventFlg = false;                             // Mpu事件是否有效
        uint64 MpuEvent = 0;                                  // Mpu事件
        uint64 MpuGeofenceBits = 0;                           // 地理围栏

        // std::vector<uint32> MpuDiagnose;                   // 诊断故障码
    } stMonitorMpuStateIn;

    typedef struct MonitorMpuVersion
    {
        uint8_t MpuVersion[32];     // Mpu版本: APP + 地图引擎 + SOC
        uint8_t MapDataVersion[32]; // 地图数据版本
        uint8_t MCUVersion[32];     // MCU版本：MCU + 硬件版本
    } stMonitorMpuVersion;

    typedef struct MonitorMpuState
    {
        uint64 TimeStamp = 0;
        EMPUState State = EMPUState::MPUERR;  // MPU状态
        EMPUState Single = EMPUState::MPUERR; // MPU信号
        EMPUState Traj = EMPUState::MPUERR;   // Mpu Traj State
        uint32 FussValue = FussDefault;       // 融合置信度(0,1,2,3)
        MonitorTraj TrajValue;                //
        MonitorState StateValue;
        MonitorSingle SingleValue;
        MonitorHighWayType HighWayValue = MonitorHighWayType::MonitorHighWayTypeDefault;
        uint32 LimitSpeedMax = LimitSpeedDefault; // 当前位置当前车道的最大限速
        uint32 LimitSpeedMin = LimitSpeedDefault; // 当前位置当前车道的最小限速
        float32 Slope = SlopeDefault;             // 纵向坡度   后期考虑实时性的数据单独提出来发送
        float32 CrossSlope = SlopeDefault;        // 横向坡度   后期考虑实时性的数据单独提出来发送
        MonitorMpuVersion MpuVer;                 // Mpu版本信息
        float32 Curvature = CurvatureDefault;     // 曲率   后期考虑实时性的数据单独提出来发送
        uint32 MpuEventMode = (uint32)EMPUEventMode::Default; // Mpu事件模式
        bool MpuEventFlg = false;                             // Mpu事件是否有效
        uint64 MpuEvent = 0;                                  // Mpu事件
        uint64 MpuGeofenceBits = 0;                           // 地理围栏

        // std::vector<uint32> MpuDiagnose;                   // 诊断故障码
        uint32 MpuLaneLineValid; // 车道线是否有效：0，无效 1，有效:
    } stMonitorMpuState;

    typedef struct MpuExpInfo
    {
        uint64 TimeStamp = 0;
        uint8 MapEnblaeMode = 0;
        uint8 GeofenceAreaFlag = 0;
    } stExpInfo;
    typedef struct MpuState
    {
        uint64 TimeStamp;
        EMPUState PositionState;
        EMPUState ExpState;
        MonitorMpuState MonitorState;
        MpuExpInfo ExpInfo;
    };

    extern bool GetMpuState(struct MpuState &state);
} // namespace mpu_t

#endif
