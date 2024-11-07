#ifndef EXP_WRAPPER_COMM_H
#define EXP_WRAPPER_COMM_H

#include <stdint.h>
#include "KTDecisionProfileCommon.h"

namespace exp_map_t
{
    enum class ExpMsgType : uint8_t
    {
        exp_type_null = 0u,
        exp_type_loc = 1u,
        exp_type_profile = 2u,
    };

    enum class ExpPositionLevel : uint8_t
    {
        exp_position_level_null = 0u,
        exp_position_level_1 = 1u,
        exp_position_level_2 = 2u,
        exp_position_level_3 = 3u,
        exp_position_level_4 = 4u,
        exp_position_level_5 = 5u,
    };
    /*********************************************************************************/
    /**************************************决策预留接口*********************************/
    /*********************************************************************************/
    using EPathId       = PathId;
    using EOffset       = Offset;
    using ELaneIndex    = LaneIndex;
    using EDistance     = Distance;
    using ELaneId       = LineId;
    using ELineId       = LineId;
    using ELaneType     = LaneType;
    typedef struct sExpMapLaneItem
    {
        /* data */
        EPathId s_LanePathId32;
        EOffset s_LaneOffsetS32;
        EOffset s_LaneOffsetE32;

        ELaneIndex s_LaneIndex16;
        ELaneIndex s_LaneNumber16;

        EDistance s_LaneLength;

        ELaneId s_current_lane_id64; //add current lane id
        ELaneId s_left_lane_id64;
        ELaneId s_right_lane_id64;

        std::vector<ELaneId> s_vLaneNextId64;
        std::vector<ELaneId> s_vLanePreId64;

        hdi::decision_map::LineGeometry s_LaneCenterGeometry;
        std::map<ELineId, hdi::decision_map::LineGeometry> s_mLaneLeftGeometry;
        std::map<ELineId, hdi::decision_map::LineGeometry> s_mLaneRightGeometry;

        std::vector<ELaneType> s_vLaneType;
        ELineId s_LaneLineCenterId64;
        std::vector<ELineId> s_vLaneLineLeftId64;
        std::vector<ELineId> s_vLaneLineRightId64;

        void Copy(decision_map_t::sDecisionMapLaneItem & item)
        {
            s_LanePathId32 = item.s_LanePathId32;
            s_LaneOffsetS32= item.s_LaneOffsetS32;
            s_LaneOffsetE32= item.s_LaneOffsetE32;
            s_LaneIndex16  = item.s_LaneIndex16;
            s_LaneNumber16 = item.s_LaneNumber16;
            s_LaneLength = item.s_LaneLength;
            s_current_lane_id64 = item.s_current_lane_id64;
            s_left_lane_id64 = item.s_left_lane_id64;
            s_right_lane_id64= item.s_right_lane_id64;
            s_vLaneNextId64.clear();
            s_vLaneNextId64.assign(s_vLaneNextId64.begin(), s_vLaneNextId64.end());
            s_vLanePreId64.clear();
            s_LaneCenterGeometry.m_lineId = item.s_LaneCenterGeometry.m_lineId;
            s_LaneCenterGeometry.m_curveType = item.s_LaneCenterGeometry.m_curveType;

            s_LaneCenterGeometry.m_points = item.s_LaneCenterGeometry.m_points;
            s_mLaneLeftGeometry = item.s_mLaneLeftGeometry;
            s_mLaneRightGeometry= item.s_mLaneRightGeometry;

            s_vLaneType = item.s_vLaneType;
            s_LaneLineCenterId64 = item.s_LaneLineCenterId64;
            s_vLaneLineLeftId64 = item.s_vLaneLineLeftId64;
            s_vLaneLineRightId64 = item.s_vLaneLineRightId64;
        }
    }stExpMapLaneItem;
    
    /*********************************************************************************/
    /**************************************决策预留接口*********************************/
    /*********************************************************************************/
    typedef struct sSensorInfo
    {
        uint64 itimetown;

        float64 droll;
        float64 dpitch;
        float64 dyaw;

        float64 drollacc;
        float64 dpitchacc;
        float64 dyawacc;
    }stSensorInfo;

    typedef struct sMapExpLocation
    {
        ExpPositionLevel                    location_level;
        decision_map_t::sDecisionPosition   location_info;
        stSensorInfo                        sensor_info;
    } stMapExpLocation;

    typedef struct sMapExpProfileInfo
    {
        decision_map_t::sDatumState profile_datum_info;
        std::vector<decision_map_t::sDecisionMapLaneItem> lane_item_info;
    } stMapExpProfileInfo;

    typedef struct sMapLocationCoordinateTrans
    {
        decision_map_t::sRelativeCoordinate relative_coordinate;
    } stMapLocationCoordinateTrans;

    //*************************地理围栏数据************************//
    enum class Reason : uint8_t
    {
        NONE = 0,               //无效
        RAMP = 1,               //匝道
        LANECONNECTIONRAMP = 2, //连接到匝道的车道
        TRAFFICLINEMISSING = 3, //车道线缺失
        LANENARROWORWIDE = 4,   //车道过窄/过宽
        LANEENDING = 5,         //车道结束
        EMERGENCYLANE = 6,      //紧急车道
        NOSEPARATEAREA = 7,     //非隔离区域
        TRAFFICLIGHT = 8,       //交通灯
        HIGHWAYEND = 9,         //高速结束
        NOTOPEN = 10,           //未开放道路
        CURVATUREOVERRUN = 11,  //曲率过大
        SLOPEOVERRUN = 12,      //坡度过大
        DIVIDEROAD = 13,        //分叉道路
        TUNNEL = 14,            //隧道
    };

    typedef struct Area
    {
        Reason m_reason         = Reason::NONE; // 不在地理围栏的原因
        uint32_t m_offset       = 0;            // 非地理围栏区域起始点在路径上的Offset
        uint32_t m_endOffset    = 0;            // 非地理围栏区域终止点在路径上的Offset
    }stArea;

    typedef struct sMapGeofenceInfo
    {
        uint64_t                    m_uTimeStamp;
        bool                        m_bInGeofence = false;      //车辆当前是否在地理围栏内
        std::vector<Area>           m_reasonArea;
        std::map<Reason, uint32_t>  m_currentInReasons;         //车辆当前不在地理围栏的原因及其驶出距离
        void Clear()
        {
            m_uTimeStamp = 0;
            m_bInGeofence = false;
            std::vector<Area>().swap(m_reasonArea);
            m_reasonArea.clear();
            std::map<Reason, uint32_t>().swap(m_currentInReasons);
            m_currentInReasons.clear();
        }
    } stMapGeofenceInfo;
    //*************************地理围栏数据************************//
}

#endif
