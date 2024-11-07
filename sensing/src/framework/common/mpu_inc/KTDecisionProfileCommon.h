#ifndef KT_DECISION_PROFILE_COMMON_H
#define KT_DECISION_PROFILE_COMMON_H

#include "hdi_decision_map_constants.hpp"
#include "hdi_decision_map_structs.hpp"
#include "SystemBase.h"
#include <string>
#include <map>

using namespace hdi::decision_map;

namespace decision_map_t
{

    enum class eDecisionMapType : uint8
    {
        E_DECISION_MAP_TYPE_DEFAULT,
        E_DECISION_MAP_TYPE_POSITION,
        E_DECISION_MAP_TYPE_PROFILE,
        E_DECISION_MAP_TYPE_PROFILE_CONTROL,
        E_DECISION_MAP_TYPE_PATH_CONTROL,
        E_DECISION_MAP_TYPE_GLOBAL,
    };

    typedef struct tagDatumState
    {
        uint64 m_profiletimestamp;
        uint64 m_timestamp;     ///< 当前基准点对应的信号时间戳
        uint64 m_lastTimestamp; ///< 上一个基准点对应的信号时间戳
        Coordinate m_point;     ///< @brief     基准点的位置
                                ///< @details   相对于上一个基准点的偏移
        void clear()
        {
            m_profiletimestamp = 0;
            m_timestamp = 0;
            m_lastTimestamp = 0;
            m_point.m_x = 0.0;
            m_point.m_y = 0.0;
            m_point.m_z = 0.0;
        }
    } sDatumState;

    // typedef struct tagsDecisionMessage
    // {
    //     e_DecisionMapType eMessageType;
    //     std::string sMessageString;
    // } sDecisionMessage;

    typedef struct tagsLinearItem
    {
        LineId m_lineId = INVALID_LINE_ID;
        LinearObjectType m_type = LinearObjectType::NA; ///< 线型物体的类型
        LineMarking m_marking = LineMarking::NA;        ///< @brief     线型物体的标记类型
                                                        ///< @details   仅类型为边线时有效
                                                        ///<            仅输出首个值，建议使用 @ref ProfileType:LineMarkingType 获取完整列表
        LineColor m_color = LineColor::Unknown;         ///< @brief     线型物体的颜色
                                                        ///< @details   仅类型为边线时有效
                                                        ///<            仅输出首个值，建议使用 @ref ProfileType:LineMarkingColour 获取完整列表
        RoadSide m_roadSide = RoadSide::NA;
        std::vector<Coordinate> m_points;
    } LinearItem;

    typedef struct tagsLaneConnectPair
    {
        PathId initialLanePathId;
        LaneIndex initialLaneIndex;
        Offset initialLaneOffset;
        PathId newLanePathId;
        LaneIndex newLaneIndex;
        Offset newLaneOffset;
    } LaneConnectPair;

    typedef struct tagsDecisionPositionMessage
    {
        Timestamp m_timestamp = INVALID_TIMESTAMP;          ///< 从定位系统获取到 GPS 坐标坐标的时间戳，单位：毫秒
        Timestamp m_positionAge = UINT64_MAX;               ///< 获取到 GPS 位置到向车辆总线发送位置消息的时间差
        Timestamp m_datumTimestamp = 0;                     ///< 当前基准点的时间戳
        ODDStatusType m_oddStatus = ODDStatusType::Unknown; ///< 地理围栏的状态
        std::vector<MatchedPosition> m_positions;           ///< 车辆位置的匹配结果列表
    } sDecisionPositionMessage;

    typedef struct tagsAbsolutePositionMessage
    {
        Timestamp m_timestamp = 0;      ///< 定位信号时间戳
        Timestamp m_datumTimestamp = 0; ///< 当前基准点的时间戳
        Timestamp m_lastTimestamp = 0;  ///< 上一个信号的时间戳
        float m_heading = 0.0;          ///< @brief     车辆航向角
                                        ///< @details   正北为 0，顺时增加，单位：角度，范围：@f$[0, 360)@f$
        VehicleSpeed m_speed = 0.0;     ///< 速度
        Coordinate m_vehiclePosition;
        Coordinate m_relativeVehicle; ///< 相对上一个位置的偏移
        ODDStatusType m_oddStatus;
    } sAbsolutePositionMessage;

    typedef struct tagsDecisionPosition
    {
        sDecisionPositionMessage s_PostionMessage;
        sAbsolutePositionMessage s_AbsolutePosition;
    } sDecisionPosition;

    typedef struct tagsDecisionMapLinearAttribute
    {
        LineControlPoints<PhysicalCrossType, PhysicalCrossType::Unknown> s_LaneCenterLinePhysicalCross;
        std::map<LineId, LineControlPoints<PhysicalCrossType, PhysicalCrossType::Unknown>> s_mLaneLeftLinePhysicalCross;
        std::map<LineId, LineControlPoints<PhysicalCrossType, PhysicalCrossType::Unknown>> s_mLaneRightLinePhysicalCross;

        LineControlPoints<LineMarking, LineMarking::Unknown> s_LaneCenterLineMarking;
        std::map<LineId, LineControlPoints<LineMarking, LineMarking::Unknown>> s_mLaneLeftLineMarking;
        std::map<LineId, LineControlPoints<LineMarking, LineMarking::Unknown>> s_mLaneRightLineMarking;

        LineControlPoints<LineSource, LineSource::Unknown> s_LaneCenterLineSource;
        std::map<LineId, LineControlPoints<LineSource, LineSource::Unknown>> s_mLaneLeftLineSource;
        std::map<LineId, LineControlPoints<LineSource, LineSource::Unknown>> s_mLaneRightLineSource;

        LineControlPoints<LineColor, LineColor::Unknown> s_LaneCenterLineColor;
        std::map<LineId, LineControlPoints<LineColor, LineColor::Unknown>> s_mLaneLeftLineColor;
        std::map<LineId, LineControlPoints<LineColor, LineColor::Unknown>> s_mLaneRightLineColor;

        LineControlPoints<LineLogic, LineLogic::NA> s_LaneCenterLineLogic;
        std::map<LineId, LineControlPoints<LineLogic, LineLogic::NA>> s_mLaneLeftLineLogic;
        std::map<LineId, LineControlPoints<LineLogic, LineLogic::NA>> s_mLaneRightLineLogic;

        LineControlPoints<uint32_t, 0> s_LaneCenterLineWidth;
        std::map<LineId, LineControlPoints<uint32_t, 0>> s_mLaneLeftLineWidth;
        std::map<LineId, LineControlPoints<uint32_t, 0>> s_mLaneRightLineWidth;

        /// huic1116
        template <typename T, T default_>
        void clear_item_map(std::map<LineId, LineControlPoints<T, default_>> &item__)
        {
            typename std::map<LineId, LineControlPoints<T, default_>>::iterator itor_left_begin = item__.begin();
            typename std::map<LineId, LineControlPoints<T, default_>>::iterator itor_left_end = item__.end();
            while (itor_left_begin != itor_left_end)
            {
                std::vector<LineControlPoint<T, default_>>().swap(itor_left_begin->second.m_controlPoints);
                itor_left_begin->second.m_controlPoints.clear();
                itor_left_begin++;
            }
            std::map<LineId, LineControlPoints<T, default_>>().swap(item__);
            item__.clear();
        }
        template <typename T, T default_>
        void clear_item(LineControlPoints<T, default_> &line_control_points)
        {
            std::vector<LineControlPoint<T, default_>>().swap(line_control_points.m_controlPoints);
		line_control_points.m_controlPoints.clear();
        }
        void clear() // delete by huic
        {
            clear_item(s_LaneCenterLinePhysicalCross);
            clear_item_map(s_mLaneLeftLinePhysicalCross);
            clear_item_map(s_mLaneRightLinePhysicalCross);

            clear_item(s_LaneCenterLineMarking);
            clear_item_map(s_mLaneLeftLineMarking);
            clear_item_map(s_mLaneRightLineMarking);

            clear_item(s_LaneCenterLineSource);
            clear_item_map(s_mLaneLeftLineSource);
            clear_item_map(s_mLaneRightLineSource);

            clear_item(s_LaneCenterLineColor);
            clear_item_map(s_mLaneLeftLineColor);
            clear_item_map(s_mLaneRightLineColor);

            clear_item(s_LaneCenterLineLogic);
            clear_item_map(s_mLaneLeftLineLogic);
            clear_item_map(s_mLaneRightLineLogic);

            clear_item(s_LaneCenterLineWidth);
            clear_item_map(s_mLaneLeftLineWidth);
            clear_item_map(s_mLaneRightLineWidth);
        }
        /// huic1116
    } sDecisionMapLinearAttribute;

    enum class VehicleType : uint8
    {
        None           = 0,   ///< 不限制车种
        PersonalCar    = 1,   ///< 小客车/小汽车
        Taxi           = 2,   ///< 出租车
        Bus            = 3,   ///< 公交车
        MotorBus       = 4,   ///< 大客车
        Buggy          = 5,   ///< 小货车/小卡车
        BigTruck       = 6,   ///< 大货车/大卡车
        DangerousGoods = 7,   ///< 危险物品车
        Emergency      = 8,   ///< 急救车
        Trailer        = 9,   ///< 拖/挂车
        Tractor        = 10,  ///< 三轮车
        Motorcycle     = 11,  ///< 摩托车
        ElecBicycle    = 12,  ///< 电动自行车
        Bicycle        = 14 , ///< 自行车/人力车
        Tram           = 15,  ///< 有轨车
        TinyCar        = 16,  ///< 微型车
    };
    typedef struct tagsLaneSpeedValue
    {
        VehicleType vehicle_type;
        uint32      vehicle_limit_speed;
    }stLaneSpeedValue;

    using cal_route = bool;
    typedef struct tagsDecisionMapLaneAttribute
    {
        std::vector<OffsetFloatEntry> s_vLaneWidth;
        std::vector<OffsetFloatEntry> s_vLaneSlope;
        std::vector<OffsetFloatEntry> s_vLaneCrossSlope;
        std::vector<OffsetFloatEntry> s_vLaneCurvature;
        cal_route lane_cal_route;
        std::vector<stLaneSpeedValue>   s_vLaneLimitSpeedMax; //车道最大限速
        std::vector<stLaneSpeedValue>   s_vLaneLimitSpeedMin; //车道最小限速
        /// huic1116
        void clear() // delete by huic
        {
            lane_cal_route = false;
            std::vector<OffsetFloatEntry>().swap(s_vLaneWidth);
            s_vLaneWidth.clear();
            std::vector<OffsetFloatEntry>().swap(s_vLaneSlope);
            s_vLaneSlope.clear();
            std::vector<OffsetFloatEntry>().swap(s_vLaneCrossSlope);
            s_vLaneCrossSlope.clear();
            std::vector<OffsetFloatEntry>().swap(s_vLaneCurvature);
            s_vLaneCurvature.clear();
        }
        /// huic1116
    } sDecisionMapLaneAttribute;

    typedef struct tagsDecisionMapLaneItem
    {
        /// huic1116
        tagsDecisionMapLaneItem() = default;
        ~tagsDecisionMapLaneItem()
        {
            clear();
        }
        /// huic1116
        Timestamp s_LaneItemTimestamp64;
        // sDatumState s_ProfileDatumState;
        PathId s_LanePathId32;
        Offset s_LaneOffsetS32;
        Offset s_LaneOffsetE32;

        LaneIndex s_LaneIndex16;
        LaneIndex s_LaneNumber16;

        Distance s_LaneLength;

        LaneTransition s_LaneTransition;
        LaneConnectType s_LaneConnectType;

        LineId s_current_lane_id64; //add current lane id
        LineId s_left_lane_id64;
        LineId s_right_lane_id64;

        std::vector<LineId> s_vLaneNextId64;
        std::vector<LineId> s_vLanePreId64;

        LinearObject s_LaneCenterObject;
        std::map<LineId, LinearObject> s_mLaneLeftObject;
        std::map<LineId, LinearObject> s_mLaneRightObject;

        LineGeometry s_LaneCenterGeometry;
        std::map<LineId, LineGeometry> s_mLaneLeftGeometry;
        std::map<LineId, LineGeometry> s_mLaneRightGeometry;

        std::vector<LaneType> s_vLaneType;
        LineId s_LaneLineCenterId64;
        std::vector<LineId> s_vLaneLineLeftId64;
        std::vector<LineId> s_vLaneLineRightId64;

        sDecisionMapLinearAttribute s_LinearAttribute;
        sDecisionMapLaneAttribute s_LaneAttribute;
        
        /// huic1116
        void clear() // delete by huic
        {
            std::vector<LineId>().swap(s_vLaneNextId64);
            s_vLaneNextId64.clear();

            std::vector<LineId>().swap(s_vLanePreId64);
            s_vLanePreId64.clear();
            std::map<LineId, LinearObject>().swap(s_mLaneLeftObject);
            s_mLaneLeftGeometry.clear();
            std::map<LineId, LinearObject>().swap(s_mLaneRightObject);
            s_mLaneRightGeometry.clear();

            std::vector<Coordinate>().swap(s_LaneCenterGeometry.m_points);
            s_LaneCenterGeometry.m_points.clear();

            std::map<LineId, LineGeometry>::iterator itor_left_begin = s_mLaneLeftGeometry.begin();
            std::map<LineId, LineGeometry>::iterator itor_left_end = s_mLaneLeftGeometry.end();
            while (itor_left_begin != itor_left_end)
            {
                std::vector<Coordinate>().swap(itor_left_begin->second.m_points);
                itor_left_begin->second.m_points.clear();
                itor_left_begin++;
            }
            std::map<LineId, LineGeometry>().swap(s_mLaneLeftGeometry);
            s_mLaneLeftGeometry.clear();

            std::map<LineId, LineGeometry>::iterator itor_right_begin = s_mLaneRightGeometry.begin();
            std::map<LineId, LineGeometry>::iterator itor_right_end = s_mLaneRightGeometry.end();
            while (itor_right_begin != itor_right_end)
            {
                std::vector<Coordinate>().swap(itor_right_begin->second.m_points);
                itor_right_begin->second.m_points.clear();
                itor_right_begin++;
            }
            std::map<LineId, LineGeometry>().swap(s_mLaneRightGeometry);
            s_mLaneRightGeometry.clear();

            std::vector<LaneType>().swap(s_vLaneType);
            s_vLaneType.clear();

            std::vector<LineId>().swap(s_vLaneLineLeftId64);
            s_vLaneLineLeftId64.clear();

            std::vector<LineId>().swap(s_vLaneLineRightId64);
            s_vLaneLineRightId64.clear();

            s_LinearAttribute.clear();
            s_LaneAttribute.clear();
        }
        /// huic1116
    } sDecisionMapLaneItem;

    typedef struct tagsDecisionMapContent
    {
        sDatumState s_ProfileDatumState;
        std::vector<decision_map_t::sDecisionMapLaneItem> v_MapLaneItem;
        /// huic1116
        void clear() // delete by huic
        {
            memset(&s_ProfileDatumState, 0, sizeof(decision_map_t::sDatumState));
            std::vector<decision_map_t::sDecisionMapLaneItem>::iterator itor_begin = v_MapLaneItem.begin();
            std::vector<decision_map_t::sDecisionMapLaneItem>::iterator itor_end = v_MapLaneItem.end();

            while (itor_begin != itor_end)
            {
                itor_begin->clear();
                itor_begin++;
            }

            std::vector<decision_map_t::sDecisionMapLaneItem>().swap(v_MapLaneItem);
            v_MapLaneItem.clear();
        }
        /// huic1116
    } sDecisionMapContent;

    typedef struct tagsRelativeCoordinate
    {
        Timestamp m_fromTimestamp = 0; ///< 当前基准点的时间戳
        Timestamp m_toTimestamp = 0;   ///< 上一个基准点时间戳

        Coordinate m_relativeCoordinate; ///< 相对上一个位置的偏移
    } sRelativeCoordinate;

    typedef struct tagsPathMatchInfo
    {
        uint32 m_sdRouteID;
        PathMatchStatus m_MatchStatus;
        std::vector<MatchFailResult> m_vMatchResult;
    }sPathMatchInfo;
    
    typedef struct tagsCustomizationMessage
    {
        Timestamp m_timestamp;
        CustomizedType m_customtype;
        ChangeMode m_changemode;
        sPathMatchInfo m_PathMatchInfo;
    }sCustomMessage;
}
#endif
