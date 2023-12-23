/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <ros/ros.h>
#include <ros/param.h>
#include <bringup/robot.h>
#include <nlohmann/json.hpp>

MG400Robot::MG400Robot(ros::NodeHandle& nh, std::string name)
    : ActionServer<FollowJointTrajectoryAction>(nh, std::move(name), false)
    , goal_{}
    , control_nh_(nh)
    , trajectory_duration_(1.0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
}

MG400Robot::~MG400Robot()
{
    ROS_INFO("~MG400Robot");
}

void MG400Robot::init()
{
    std::string ip = control_nh_.param<std::string>("robot_ip_address", "192.168.1.6");

    trajectory_duration_ = control_nh_.param("trajectory_duration", 0.3);
    ROS_INFO("trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/EnableRobot", &MG400Robot::enableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DisableRobot", &MG400Robot::disableRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ClearError", &MG400Robot::clearError, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ResetRobot", &MG400Robot::resetRobot, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedFactor", &MG400Robot::speedFactor, this));
    // server_tbl_.push_back(
    //     control_nh_.advertiseService("/bringup/srv/DigitalOutputs", &MG400Robot::digitalOutputs, this));   //DO
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetErrorID", &MG400Robot::getErrorID, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/User", &MG400Robot::user, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Tool", &MG400Robot::tool, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RobotMode", &MG400Robot::robotMode, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PayLoad", &MG400Robot::payload, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DO", &MG400Robot::DO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DOExecute", &MG400Robot::DOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDO", &MG400Robot::toolDO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDOExecute", &MG400Robot::toolDOExecute, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AO", &MG400Robot::AO, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AOExecute", &MG400Robot::AOExecute, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AccJ", &MG400Robot::accJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/AccL", &MG400Robot::accL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedJ", &MG400Robot::speedJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SpeedL", &MG400Robot::speedL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Arch", &MG400Robot::arch, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/CP", &MG400Robot::cp, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/LimZ", &MG400Robot::limZ, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/SetArmOrientation", &MG400Robot::setArmOrientation, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetPayload", &MG400Robot::setPayload, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/PositiveSolution", &MG400Robot::positiveSolution, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/InverseSolution", &MG400Robot::inverseSolution, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PowerOn", &MG400Robot::powerOn, this));
    // //四轴无此指令
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RunScript", &MG400Robot::runScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopScript", &MG400Robot::stopScript, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/PauseScript", &MG400Robot::pauseScript, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/ContinueScript", &MG400Robot::continueScript, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetSafeSkin", &MG400Robot::setSafeSkin, this));
    // //四轴无此指令
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/SetObstacleAvoid", &MG400Robot::setObstacleAvoid, this));
    server_tbl_.push_back(
        control_nh_.advertiseService("/bringup/srv/SetCollisionLevel", &MG400Robot::setCollisionLevel, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetAngle", &MG400Robot::getAngle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetPose", &MG400Robot::getPose, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/EmergencyStop", &MG400Robot::emergencyStop, this));

    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJ", &MG400Robot::movJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovL", &MG400Robot::movL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/JointMovJ", &MG400Robot::jointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Jump", &MG400Robot::jump, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovJ", &MG400Robot::relMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovL", &MG400Robot::relMovL, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovLIO", &MG400Robot::movLIO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJIO", &MG400Robot::movJIO, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Arc", &MG400Robot::arc, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Circle", &MG400Robot::circle, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovJUser", &MG400Robot::relMovJUser, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelMovLUser", &MG400Robot::relMovLUser, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/RelJointMovJ", &MG400Robot::relJointMovJ, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MovJExt", &MG400Robot::movJExt, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ServoJ", &MG400Robot::servoJ, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ServoP", &MG400Robot::servoP, this));  //
    // 四轴无此指令
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Sync", &MG400Robot::sync, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartTrace", &MG400Robot::startTrace, this));
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartPath", &MG400Robot::startPath, this));  //
    // 四轴无此指令
    // server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartFCTrace", &MG400Robot::startFCTrace,
    // this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/MoveJog", &MG400Robot::moveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopmoveJog", &MG400Robot::stopmoveJog, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SyncAll", &MG400Robot::syncAll, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Wait", &MG400Robot::wait, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Continue", &MG400Robot::Continue, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/Pause", &MG400Robot::pause, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ModbusCreate", &MG400Robot::modbusCreate, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ModbusClose", &MG400Robot::modbusClose, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetInBits", &MG400Robot::getInBits, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetInRegs", &MG400Robot::getInRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetHoldRegs", &MG400Robot::getHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetHoldRegs", &MG400Robot::setHoldRegs, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/GetCoils", &MG400Robot::getCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/SetCoils", &MG400Robot::setCoils, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/ToolDI", &MG400Robot::toolDI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DI", &MG400Robot::DI, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/DOGroup", &MG400Robot::DOGroup, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/BrakeControl", &MG400Robot::brakeControl, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StartDrag", &MG400Robot::startDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/StopDrag", &MG400Robot::stopDrag, this));
    server_tbl_.push_back(control_nh_.advertiseService("/bringup/srv/LoadSwitch", &MG400Robot::loadSwitch, this));
    // registerGoalCallback(boost::bind(&MG400Robot::goalHandle, this, _1));
    // registerCancelCallback(boost::bind(&MG400Robot::cancelHandle, this, _1));

    pubFeedInfo = control_nh_.advertise<std_msgs::String>("/bringup/msg/FeedInfo", 1000);

    threadPubFeedBackInfo = std::thread(&MG400Robot::pubFeedBackInfo, this);
    threadPubFeedBackInfo.detach();
    start();
}

void MG400Robot::pubFeedBackInfo()
{
    struct RealTimeData realTimeData;

    // 设置发布频率为10Hz
    ros::Rate rate(100);
    while (ros::ok())
    {
        realTimeData = commander_->getRealTimeData();
        nlohmann::json root;
        root["EnableStatus"] = realTimeData.EnableStatus;
        root["ErrorStatus"] = realTimeData.ErrorStatus;
        root["RunQueuedCmd"] = realTimeData.isRunQueuedCmd;
        std::vector<double> toolvectoractual;
        // memcpy(toolvectoractual.data(), realTimeData.tool_vector_actual, sizeof(realTimeData.tool_vector_actual));
        for (int i = 0; i < 6; i++)
        {
            toolvectoractual.push_back(realTimeData.tool_vector_actual[i]);
        }
        root["ToolVectorActual"] = toolvectoractual;
        std::string toolVectorActualStr = root.dump();

        std_msgs::String msgFeedInfo;
        msgFeedInfo.data = toolVectorActualStr;
        pubFeedInfo.publish(msgFeedInfo);
        rate.sleep();
    }
}

// void MG400Robot::feedbackHandle(const ros::TimerEvent& tm,
//                                 ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     control_msgs::FollowJointTrajectoryFeedback feedback;

//     double current_joints[6];
//     getJointState(current_joints);

//     for (uint32_t i = 0; i < 6; i++)
//     {
//         feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
//         feedback.actual.positions.push_back(current_joints[i]);
//         feedback.desired.positions.push_back(goal_[i]);
//     }

//     handle.publishFeedback(feedback);
// }

// void MG400Robot::moveHandle(const ros::TimerEvent& tm,
//                             ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

//     if (index_ < trajectory->trajectory.points.size())
//     {
//         auto point = trajectory->trajectory.points[index_].positions;
//         double tmp[6];
//         for (uint32_t i = 0; i < 4; i++)
//         {
//             tmp[i] = point[i] * 180.0 / 3.1415926;
//         }

//         bringup::ServoJ srv;
//         srv.request.j1 = tmp[0];
//         srv.request.j2 = tmp[1];
//         srv.request.j3 = tmp[2];
//         srv.request.j4 = tmp[3];

//         servoJ(srv.request, srv.response);
//         index_++;
//     }
//     else
//     {
// #define OFFSET_VAL 0.01
//         double current_joints[6];
//         getJointState(current_joints);
//         if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
//             (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
//             (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
//             (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL))

//         {
//             timer_.stop();
//             movj_timer_.stop();
//             handle.setSucceeded();
//         }
//     }
// }

// void MG400Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     index_ = 0;
//     for (uint32_t i = 0; i < 6; i++)
//     {
//         goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
//     }
//     timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&MG400Robot::feedbackHandle, this, _1, handle));
//     movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
//                                           boost::bind(&MG400Robot::moveHandle, this, _1, handle));
//     timer_.start();
//     movj_timer_.start();
//     handle.setAccepted();
// }

// void MG400Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     timer_.stop();
//     movj_timer_.stop();
//     handle.setSucceeded();
// }

void MG400Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool MG400Robot::isEnable() const
{
    return commander_->isEnable();
}

bool MG400Robot::isConnected() const
{
    return commander_->isConnected();
}

void MG400Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool MG400Robot::enableRobot(bringup::EnableRobot::Request& request, bringup::EnableRobot::Response& response)
{
    try
    {
        char cmd[1000];
        std::string str{ "EnableRobot(" };
        for (int i = 0; i < request.args.size(); i++)
        {
            if (i == request.args.size() - 1)
            {
                str = str + std::to_string(request.args[i]);
                break;
            }
            str = str + std::to_string(request.args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }

    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool MG400Robot::disableRobot(bringup::DisableRobot::Request& request, bringup::DisableRobot::Response& response)
{
    try
    {
        const char* cmd = "DisableRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool MG400Robot::clearError(bringup::ClearError::Request& request, bringup::ClearError::Response& response)
{
    try
    {
        const char* cmd = "ClearError()";
        commander_->dashboardDoCmd(cmd, response.res);
        response.res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        response.res = -1;
        return false;
    }
}

bool MG400Robot::resetRobot(bringup::ResetRobot::Request& request, bringup::ResetRobot::Response& response)
{
    try
    {
        const char* cmd = "ResetRobot()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::speedFactor(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request.ratio);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool MG400Robot::digitalOutputs(bringup::DigitalOutputs::Request& request, bringup::DigitalOutputs::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Do(%d,%d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool MG400Robot::getErrorID(bringup::GetErrorID::Request& request, bringup::GetErrorID::Response& response)
{
    try
    {
        char cmd[100];
        std::vector<std::string> result;
        sprintf(cmd, "GetErrorID()");
        commander_->dashboardDoCmd(cmd, response.res, result);
        std::string resultStr{ "" };
        for (int i = 0; i < result.size(); i++)
        {
            resultStr = resultStr.append(result[i]);
        }
        result.clear();
        result = regexRecv(resultStr);
        for (int i = 0; i < result.size(); i++)
        {
            ROS_ERROR("ErrorID: %s", result[i].c_str());
            response.errorID.push_back(std::stoi(result[i]));
        }

        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::user(bringup::User::Request& request, bringup::User::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::tool(bringup::Tool::Request& request, bringup::Tool::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::robotMode(bringup::RobotMode::Request& request, bringup::RobotMode::Response& response)
{
    try
    {
        const char* cmd = "RobotMode()";

        std::vector<std::string> result;
        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.empty())
            throw std::logic_error("robotMode : Empty string");

        response.mode = str2Int(result[0].c_str());
        response.res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::payload(bringup::PayLoad::Request& request, bringup::PayLoad::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f, %0.3f)", request.weight, request.inertia);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::DO(bringup::DO::Request& request, bringup::DO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::DOExecute(bringup::DOExecute::Request& request, bringup::DOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DOExecute(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::toolDO(bringup::ToolDO::Request& request, bringup::ToolDO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::toolDOExecute(bringup::ToolDOExecute::Request& request, bringup::ToolDOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool MG400Robot::AO(bringup::AO::Request& request, bringup::AO::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d, %d)", request.index, request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

/*
bool MG400Robot::AOExecute(bringup::AOExecute::Request& request, bringup::AOExecute::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AOExecute(%d, %0.3f)", request.index, static_cast<float>(request.value));
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool MG400Robot::accJ(bringup::AccJ::Request& request, bringup::AccJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::accL(bringup::AccL::Request& request, bringup::AccL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::speedJ(bringup::SpeedJ::Request& request, bringup::SpeedJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::speedL(bringup::SpeedL::Request& request, bringup::SpeedL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::arch(bringup::Arch::Request& request, bringup::Arch::Response& response)
{
    try
    {
        char cmd[100];
        if (request.cpValue.size() == 0)
        {
            sprintf(cmd, "Arch(%d)", request.index);
        }
        else
        {
            sprintf(cmd, "Arch(%d,%s)", request.index, request.cpValue[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::cp(bringup::CP::Request& request, bringup::CP::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request.r);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::limZ(bringup::LimZ::Request& request, bringup::LimZ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request.value);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setArmOrientation(bringup::SetArmOrientation::Request& request,
                                   bringup::SetArmOrientation::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request.LorR, request.UorD, request.ForN, request.Config6);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setPayload(bringup::SetPayload::Request& request, bringup::SetPayload::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetPayload(%f)", request.load);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::positiveSolution(bringup::PositiveSolution::Request& request,
                                  bringup::PositiveSolution::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PositiveSolution(%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request.offset1, request.offset2,
                request.offset3, request.offset4, request.user, request.tool);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::inverseSolution(bringup::InverseSolution::Request& request,
                                 bringup::InverseSolution::Response& response)
{
    try
    {
        char cmd[100];
        if (request.JointNear == "")
        {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request.offset1, request.offset2,
                    request.offset3, request.offset4, request.user, request.tool);
        }
        else
        {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%s)", request.offset1, request.offset2,
                    request.offset3, request.offset4, request.user, request.tool, request.isJointNear,
                    request.JointNear.c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool MG400Robot::powerOn(bringup::PowerOn::Request& request, bringup::PowerOn::Response& response)
{
    try
    {
        const char* cmd = "PowerOn()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool MG400Robot::runScript(bringup::RunScript::Request& request, bringup::RunScript::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request.projectName.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::stopScript(bringup::StopScript::Request& request, bringup::StopScript::Response& response)
{
    try
    {
        const char* cmd = "StopScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::pauseScript(bringup::PauseScript::Request& request, bringup::PauseScript::Response& response)
{
    try
    {
        const char* cmd = "PauseScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::modbusCreate(bringup::ModbusCreate::Request& request, bringup::ModbusCreate::Response& response)
{
    try
    {
        char cmd[300];
        std::vector<std::string> result;
        if (request.is_rtu.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d)", request.ip.c_str(), request.port, request.slave_id);
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d,%d)", request.ip.c_str(), request.port, request.slave_id,
                     request.is_rtu[0]);
        }
        commander_->dashboardDoCmd(cmd, response.res, result);
        if (result.size() != 2)
            throw std::logic_error("Haven't recv any result");

        response.res = str2Int(result[0].c_str());
        response.index = str2Int(result[1].c_str());
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        response.index = -1;
        return false;
    }
}

bool MG400Robot::modbusClose(bringup::ModbusClose::Request& request, bringup::ModbusClose::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ModbusClose(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::getInBits(bringup::GetInBits::Request& request, bringup::GetInBits::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetInBits(%d,%d,%d)", request.index, request.addr, request.count);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::getInRegs(bringup::GetInRegs::Request& request, bringup::GetInRegs::Response& response)
{
    try
    {
        char cmd[100];
        if (request.valType.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d)", request.index, request.addr, request.count);
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valType[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::getHoldRegs(bringup::GetHoldRegs::Request& request, bringup::GetHoldRegs::Response& response)
{
    try
    {
        char cmd[100];
        if (request.valType.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d)", request.index, request.addr, request.count);
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d,%s)", request.index, request.addr, request.addr,
                     request.valType[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setHoldRegs(bringup::SetHoldRegs::Request& request, bringup::SetHoldRegs::Response& response)
{
    try
    {
        char cmd[100];
        if (request.valType.size() == 0)
        {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s)", request.index, request.addr, request.count,
                     request.valTab.c_str());
        }
        else
        {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", request.index, request.addr, request.count,
                     request.valTab.c_str(), request.valType[0].c_str());
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::getCoils(bringup::GetCoils::Request& request, bringup::GetCoils::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetCoils(%d,%d,%d)", request.index, request.addr, request.count);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setCoils(bringup::SetCoils::Request& request, bringup::SetCoils::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "SetCoils(%d,%d,%d,%s)", request.index, request.addr, request.count,
                 request.valTab.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::DI(bringup::DI::Request& request, bringup::DI::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "DI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::toolDI(bringup::ToolDI::Request& request, bringup::ToolDI::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ToolDI(%d)", request.index);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::DOGroup(bringup::DOGroup::Request& request, bringup::DOGroup::Response& response)
{
    try
    {
        char cmd[1000];
        std::string str{ "DOGroup(" };
        for (int i = 0; i < request.args.size(); i++)
        {
            if (i == request.args.size() - 1)
            {
                str = str + std::to_string(request.args[i]);
                break;
            }
            str = str + std::to_string(request.args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::brakeControl(bringup::BrakeControl::Request& request, bringup::BrakeControl::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "BrakeControl(%d,%d)", request.axisID, request.value);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::startDrag(bringup::StartDrag::Request& request, bringup::StartDrag::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StartDrag()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::stopDrag(bringup::StopDrag::Request& request, bringup::StopDrag::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StopDrag()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::loadSwitch(bringup::LoadSwitch::Request& request, bringup::LoadSwitch::Response& response)
{
    try
    {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "LoadSwitch(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

// bool MG400Robot::setHoldRegs(bringup::SetHoldRegs::Request& request, bringup::SetHoldRegs::Response& response)
// {
//     try
//     {
//         char cmd[200];
//         std::vector<std::string> result;
//         snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", request.index, request.addr, request.count,
//                  request.regs.c_str(), request.type.c_str());
//         commander_->dashboardDoCmd(cmd, response.res, result);
//         if (result.empty())
//             throw std::logic_error("Haven't recv any result");

//         response.res = str2Int(result[0].c_str());
//         return true;
//     }
//     catch (const TcpClientException& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
//     catch (const std::exception& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
// }

bool MG400Robot::continueScript(bringup::ContinueScript::Request& request, bringup::ContinueScript::Response& response)
{
    try
    {
        const char* cmd = "ContinueScript()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool MG400Robot::setSafeSkin(bringup::SetSafeSkin::Request& request, bringup::SetSafeSkin::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}*/

bool MG400Robot::setObstacleAvoid(bringup::SetObstacleAvoid::Request& request,
                                  bringup::SetObstacleAvoid::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request.status);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::setCollisionLevel(bringup::SetCollisionLevel::Request& request,
                                   bringup::SetCollisionLevel::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request.level);
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::getAngle(bringup::GetAngle::Request& request, bringup::GetAngle::Response& response)
{
    try
    {
        char cmd[100] = "GetAngle()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::getPose(bringup::GetPose::Request& request, bringup::GetPose::Response& response)
{
    try
    {
        char cmd[100];
        if (request.user.size() == 0)
        {
            strcpy(cmd, "GetPose()");
        }
        else
        {
            sprintf(cmd, "GetPose(%d,%d)", request.user[0], request.tool[0]);
        }

        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::emergencyStop(bringup::EmergencyStop::Request& request, bringup::EmergencyStop::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::sync(bringup::Sync::Request& request, bringup::Sync::Response& response)
{
    try
    {
        char result[50];
        const char* cmd = "Sync()";
        commander_->dashboardDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool MG400Robot::movJ(bringup::MovJ::Request& request, bringup::MovJ::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::movL(bringup::MovL::Request& request, bringup::MovL::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

// bool MG400Robot::servoJ(bringup::ServoJ::Request& request, bringup::ServoJ::Response& response)
// {
//     try
//     {
//         char cmd[100];
//         sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f)", request.j1, request.j2, request.j3, request.j4);
//         commander_->motionDoCmd(cmd, response.res);
//         response.res = 0;
//         return true;
//     }
//     catch (const TcpClientException& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
// }

bool MG400Robot::jump(bringup::Jump::Request& request, bringup::Jump::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::arc(bringup::Arc::Request& request, bringup::Arc::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.x1, request.y1, request.z1,
                request.r1, request.x2, request.y2, request.z2, request.r2);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::circle(bringup::Circle::Request& request, bringup::Circle::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "Circle(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request.count, request.x1,
                request.y1, request.z1, request.r1, request.x2, request.y2, request.z2, request.r2);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::relMovJUser(bringup::RelMovJUser::Request& request, bringup::RelMovJUser::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "RelMovJUser(%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.r,
                request.user);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::relMovLUser(bringup::RelMovLUser::Request& request, bringup::RelMovLUser::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "RelMovLUser(%0.3f,%0.3f,%0.3f,%0.3f,%d", request.x, request.y, request.z, request.r,
                request.user);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::relJointMovJ(bringup::RelJointMovJ::Request& request, bringup::RelJointMovJ::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "RelJointMovJ(%0.3f,%0.3f,%0.3f,%0.3f", request.offset1, request.offset2, request.offset3,
                request.offset4);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::movJExt(bringup::MovJExt::Request& request, bringup::MovJExt::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovJExt(%0.3f", request.offset);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

// bool MG400Robot::servoP(bringup::ServoP::Request& request, bringup::ServoP::Response& response)
// {
//     try
//     {
//         char cmd[100];
//         sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z, request.r);
//         commander_->motionDoCmd(cmd, response.res);
//         return true;
//     }
//     catch (const TcpClientException& err)
//     {
//         ROS_ERROR("%s", err.what());
//         response.res = -1;
//         return false;
//     }
// }

bool MG400Robot::relMovJ(bringup::RelMovJ::Request& request, bringup::RelMovJ::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f)", request.offset1, request.offset2, request.offset3,
                request.offset4);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::relMovL(bringup::RelMovL::Request& request, bringup::RelMovL::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovL(%0.3f,%0.3f,%0.3f)", request.x, request.y, request.z);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::jointMovJ(bringup::JointMovJ::Request& request, bringup::JointMovJ::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "jointMovJ(%0.3f,%0.3f,%0.3f,%0.3f", request.j1, request.j2, request.j3, request.j4);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::movLIO(bringup::MovLIO::Request& request, bringup::MovLIO::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovLIO(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::movJIO(bringup::MovJIO::Request& request, bringup::MovJIO::Response& response)
{
    try
    {
        char cmd[500];
        sprintf(cmd, "MovJIO(%0.3f,%0.3f,%0.3f,%0.3f", request.x, request.y, request.z, request.r);
        std::string str{ "" };
        for (int i = 0; i < request.paramValue.size(); i++)
        {
            str = str + "," + std::string(request.paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

/*
bool MG400Robot::startTrace(bringup::StartTrace::Request& request, bringup::StartTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request.trace_name.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

/*
bool MG400Robot::startPath(bringup::StartPath::Request& request, bringup::StartPath::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request.trace_name.c_str(), request.const_val, request.cart);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

/*
bool MG400Robot::startFCTrace(bringup::StartFCTrace::Request& request, bringup::StartFCTrace::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request.trace_name.c_str());
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
*/

bool MG400Robot::moveJog(bringup::MoveJog::Request& request, bringup::MoveJog::Response& response)
{
    try
    {
        char cmd[100];
        if (request.axisID == "")
        {
            strcpy(cmd, "MoveJog()");
        }
        else
        {
            std::string str = "MoveJog(" + std::string(request.axisID);
            for (int i = 0; i < request.paramValue.size(); i++)
            {
                str = str + "," + std::string(request.paramValue[i]);
            }
            str = str + ")";
            strcpy(cmd, str.c_str());
        }

        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::stopmoveJog(bringup::StopmoveJog::Request& request, bringup::StopmoveJog::Response& response)
{
    try
    {
        char cmd[100] = "moveJog()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::syncAll(bringup::SyncAll::Request& request, bringup::SyncAll::Response& response)
{
    try
    {
        char cmd[100] = "SyncAll()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}
bool MG400Robot::wait(bringup::Wait::Request& request, bringup::Wait::Response& response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Wait(%d)", request.time);
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::Continue(bringup::Continues::Request& request, bringup::Continues::Response& response)
{
    try
    {
        char cmd[100] = "Continue()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

bool MG400Robot::pause(bringup::Pause::Request& request, bringup::Pause::Response& response)
{
    try
    {
        char cmd[100] = "Pause()";
        commander_->motionDoCmd(cmd, response.res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        ROS_ERROR("%s", err.what());
        response.res = -1;
        return false;
    }
}

int MG400Robot::str2Int(const char* val)
{
    char* err;
    int mode = (int)strtol(val, &err, 10);
    if (*err != 0)
        throw std::logic_error(std::string("Invalid value : ") + val);
    return mode;
}

std::vector<std::string> MG400Robot::regexRecv(std::string getRecvInfo)
{
    std::regex pattern("-?\\d+");
    std::smatch matches;
    std::string::const_iterator searchStart(getRecvInfo.cbegin());
    std::vector<std::string> vecErrorId;
    while (std::regex_search(searchStart, getRecvInfo.cend(), matches, pattern))
    {
        for (auto& match : matches)
        {
            vecErrorId.push_back(match.str());
        }
        searchStart = matches.suffix().first;
    }
    return vecErrorId;
};