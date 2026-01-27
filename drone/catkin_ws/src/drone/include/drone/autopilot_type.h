#pragma once

#include <string>
#include <ros/ros.h>
#include <mavros_msgs/ParamGet.h>

namespace drone
{

enum class AutopilotType
{
    UNKNOWN = 0,
    PX4,
    ARDUPILOT
};

inline bool mavrosParamGet(ros::NodeHandle& nh,
                           const std::string& id,
                           mavros_msgs::ParamValue& out)
{
    auto client = nh.serviceClient<mavros_msgs::ParamGet>("/mavros/param/get");
    mavros_msgs::ParamGet srv;
    srv.request.param_id = id;

    if (!client.call(srv)) return false;
    if (!srv.response.success) return false;

    out = srv.response.value;
    return true;
}

inline AutopilotType detectAutopilot(ros::NodeHandle& nh)
{
    mavros_msgs::ParamValue v;

    // PX4
    if (mavrosParamGet(nh, "SYS_MC_EST_GROUP", v))
        return AutopilotType::PX4;

    // ArduPilot
    if (mavrosParamGet(nh, "AHRS_EKF_TYPE", v))
        return AutopilotType::ARDUPILOT;

    return AutopilotType::UNKNOWN;
}

inline std::string autopilotName(AutopilotType t)
{
    switch (t)
    {
        case AutopilotType::PX4:       return "PX4";
        case AutopilotType::ARDUPILOT: return "ArduPilot";
        default:                      return "Unknown";
    }
}

} // namespace drone
