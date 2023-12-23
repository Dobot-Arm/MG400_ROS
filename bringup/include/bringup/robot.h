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

#include <string>
#include <memory>
#include <algorithm>
#include <regex>
#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <jsoncpp/json/json.h>
#include <bringup/commander.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <bringup/EnableRobot.h>
#include <bringup/DisableRobot.h>
#include <bringup/ClearError.h>
#include <bringup/ResetRobot.h>
#include <bringup/SpeedFactor.h>
// #include <bringup/DigitalOutputs.h>
#include <bringup/User.h>
#include <bringup/Tool.h>
#include <bringup/RobotMode.h>
#include <bringup/PayLoad.h>
#include <bringup/DO.h>
#include <bringup/DOExecute.h>
#include <bringup/ToolDO.h>
#include <bringup/ToolDOExecute.h>
// #include <bringup/AO.h>
// #include <bringup/AOExecute.h>
#include <bringup/AccJ.h>
#include <bringup/AccL.h>
#include <bringup/SpeedJ.h>
#include <bringup/SpeedL.h>
#include <bringup/Arch.h>
#include <bringup/CP.h>
#include <bringup/LimZ.h>
#include <bringup/SetArmOrientation.h>
// #include <bringup/PowerOn.h>
#include <bringup/RunScript.h>
#include <bringup/StopScript.h>
#include <bringup/PauseScript.h>
#include <bringup/ContinueScript.h>
#include <bringup/GetHoldRegs.h>
#include <bringup/SetHoldRegs.h>
// #include <bringup/SetSafeSkin.h>
#include <bringup/SetObstacleAvoid.h>

#include <bringup/SetCollisionLevel.h>
#include <bringup/EmergencyStop.h>

#include <bringup/MovJ.h>
#include <bringup/MovL.h>
#include <bringup/Jump.h>
#include <bringup/Arc.h>
#include <bringup/Sync.h>
#include <bringup/Circle.h>
// #include <bringup/ServoJ.h>
// #include <bringup/StartTrace.h>
#include <bringup/StartPath.h>
// #include <bringup/StartFCTrace.h>
#include <bringup/MoveJog.h>
// #include <bringup/ServoP.h>
#include <bringup/RelMovJ.h>
#include <bringup/RelMovL.h>
#include <bringup/JointMovJ.h>
#include <bringup/RobotStatus.h>
#include <bringup/ModbusCreate.h>
#include <bringup/GetErrorID.h>
#include <bringup/SetPayload.h>
#include <bringup/PositiveSolution.h>
#include <bringup/InverseSolution.h>
#include <bringup/ModbusClose.h>
#include <bringup/GetInBits.h>
#include <bringup/GetInRegs.h>
#include <bringup/GetCoils.h>
#include <bringup/SetCoils.h>
#include <bringup/DI.h>
#include <bringup/ToolDI.h>
#include <bringup/DOGroup.h>
#include <bringup/BrakeControl.h>
#include <bringup/StartDrag.h>
#include <bringup/StopDrag.h>
#include <bringup/LoadSwitch.h>
#include <bringup/GetAngle.h>
#include <bringup/GetPose.h>
#include <bringup/MovLIO.h>
#include <bringup/MovJIO.h>
#include <bringup/RelMovJUser.h>
#include <bringup/RelMovLUser.h>
#include <bringup/MovJExt.h>
#include <bringup/StopmoveJog.h>
#include <bringup/SyncAll.h>
#include <bringup/Wait.h>
#include <bringup/Continues.h>
#include <bringup/Pause.h>
#include <bringup/RelJointMovJ.h>

using namespace actionlib;
using namespace control_msgs;

/**
 * MG400Robot
 */
class MG400Robot : protected ActionServer<FollowJointTrajectoryAction>
{
private:
    double goal_[6];
    uint32_t index_;
    ros::Timer timer_;
    ros::Timer movj_timer_;
    double trajectory_duration_;
    ros::NodeHandle control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<ros::ServiceServer> server_tbl_;
    std::thread threadPubFeedBackInfo;
    ros::Publisher pubFeedInfo;

public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    MG400Robot(ros::NodeHandle& nh, std::string name);

    /**
     * MG400Robot
     */
    ~MG400Robot() override;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    bool enableRobot(bringup::EnableRobot::Request& request, bringup::EnableRobot::Response& response);
    bool disableRobot(bringup::DisableRobot::Request& request, bringup::DisableRobot::Response& response);
    bool clearError(bringup::ClearError::Request& request, bringup::ClearError::Response& response);
    bool resetRobot(bringup::ResetRobot::Request& request, bringup::ResetRobot::Response& response);
    bool speedFactor(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response);
    // bool digitalOutputs(bringup::DigitalOutputs::Request& request, bringup::DigitalOutputs::Response& response); //DO
    bool getErrorID(bringup::GetErrorID::Request& request, bringup::GetErrorID::Response& response);
    bool user(bringup::User::Request& request, bringup::User::Response& response);
    bool tool(bringup::Tool::Request& request, bringup::Tool::Response& response);
    bool robotMode(bringup::RobotMode::Request& request, bringup::RobotMode::Response& response);
    bool payload(bringup::PayLoad::Request& request, bringup::PayLoad::Response& response);
    bool DO(bringup::DO::Request& request, bringup::DO::Response& response);
    bool DOExecute(bringup::DOExecute::Request& request, bringup::DOExecute::Response& response);
    bool toolDO(bringup::ToolDO::Request& request, bringup::ToolDO::Response& response);
    bool toolDOExecute(bringup::ToolDOExecute::Request& request, bringup::ToolDOExecute::Response& response);
    // bool AO(bringup::AO::Request& request, bringup::AO::Response& response);    //四轴无此指令
    // bool AOExecute(bringup::AOExecute::Request& request, bringup::AOExecute::Response& response);   //四轴无此指令
    bool accJ(bringup::AccJ::Request& request, bringup::AccJ::Response& response);
    bool accL(bringup::AccL::Request& request, bringup::AccL::Response& response);
    bool speedJ(bringup::SpeedJ::Request& request, bringup::SpeedJ::Response& response);
    bool speedL(bringup::SpeedL::Request& request, bringup::SpeedL::Response& response);
    bool arch(bringup::Arch::Request& request, bringup::Arch::Response& response);
    bool cp(bringup::CP::Request& request, bringup::CP::Response& response);
    bool limZ(bringup::LimZ::Request& request, bringup::LimZ::Response& response);
    bool setArmOrientation(bringup::SetArmOrientation::Request& request,
                           bringup::SetArmOrientation::Response& response);
    bool setPayload(bringup::SetPayload::Request& request, bringup::SetPayload::Response& response);
    bool positiveSolution(bringup::PositiveSolution::Request& request, bringup::PositiveSolution::Response& response);
    bool inverseSolution(bringup::InverseSolution::Request& request, bringup::InverseSolution::Response& response);
    // bool powerOn(bringup::PowerOn::Request& request, bringup::PowerOn::Response& response);
    bool runScript(bringup::RunScript::Request& request, bringup::RunScript::Response& response);
    bool stopScript(bringup::StopScript::Request& request, bringup::StopScript::Response& response);
    bool pauseScript(bringup::PauseScript::Request& request, bringup::PauseScript::Response& response);
    bool continueScript(bringup::ContinueScript::Request& request, bringup::ContinueScript::Response& response);
    bool modbusCreate(bringup::ModbusCreate::Request& request, bringup::ModbusCreate::Response& response);
    bool modbusClose(bringup::ModbusClose::Request& request, bringup::ModbusClose::Response& response);
    bool getInBits(bringup::GetInBits::Request& request, bringup::GetInBits::Response& response);
    bool getInRegs(bringup::GetInRegs::Request& request, bringup::GetInRegs::Response& response);
    bool getHoldRegs(bringup::GetHoldRegs::Request& request, bringup::GetHoldRegs::Response& response);
    bool setHoldRegs(bringup::SetHoldRegs::Request& request, bringup::SetHoldRegs::Response& response);
    bool getCoils(bringup::GetCoils::Request& request, bringup::GetCoils::Response& response);
    bool setCoils(bringup::SetCoils::Request& request, bringup::SetCoils::Response& response);
    bool DI(bringup::DI::Request& request, bringup::DI::Response& response);
    bool toolDI(bringup::ToolDI::Request& request, bringup::ToolDI::Response& response);
    bool DOGroup(bringup::DOGroup::Request& request, bringup::DOGroup::Response& response);
    bool brakeControl(bringup::BrakeControl::Request& request, bringup::BrakeControl::Response& response);
    bool startDrag(bringup::StartDrag::Request& request, bringup::StartDrag::Response& response);
    bool stopDrag(bringup::StopDrag::Request& request, bringup::StopDrag::Response& response);
    bool loadSwitch(bringup::LoadSwitch::Request& request, bringup::LoadSwitch::Response& response);

    // bool setSafeSkin(bringup::SetSafeSkin::Request& request, bringup::SetSafeSkin::Response& response);
    bool setObstacleAvoid(bringup::SetObstacleAvoid::Request& request, bringup::SetObstacleAvoid::Response& response);
    //    bool getTraceStartPose(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response);
    //    bool getPathStartPose(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response);
    //    bool positiveSolution(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response);
    bool setCollisionLevel(bringup::SetCollisionLevel::Request& request,
                           bringup::SetCollisionLevel::Response& response);
    //    bool handleTrajPoints(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response);
    //    bool getSixForceData(bringup::SpeedFactor::Request& request, bringup::SpeedFactor::Response& response);
    bool getAngle(bringup::GetAngle::Request& request, bringup::GetAngle::Response& response);
    bool getPose(bringup::GetPose::Request& request, bringup::GetPose::Response& response);
    bool emergencyStop(bringup::EmergencyStop::Request& request, bringup::EmergencyStop::Response& response);

    bool movJ(bringup::MovJ::Request& request, bringup::MovJ::Response& response);
    bool movL(bringup::MovL::Request& request, bringup::MovL::Response& response);
    bool jointMovJ(bringup::JointMovJ::Request& request, bringup::JointMovJ::Response& response);
    bool jump(bringup::Jump::Request& request, bringup::Jump::Response& response);
    bool relMovJ(bringup::RelMovJ::Request& request, bringup::RelMovJ::Response& response);
    bool relMovL(bringup::RelMovL::Request& request, bringup::RelMovL::Response& response);
    bool movLIO(bringup::MovLIO::Request& request, bringup::MovLIO::Response& response);
    bool movJIO(bringup::MovJIO::Request& request, bringup::MovJIO::Response& response);
    bool arc(bringup::Arc::Request& request, bringup::Arc::Response& response);
    bool circle(bringup::Circle::Request& request, bringup::Circle::Response& response);
    bool relMovJUser(bringup::RelMovJUser::Request& request, bringup::RelMovJUser::Response& response);
    bool relMovLUser(bringup::RelMovLUser::Request& request, bringup::RelMovLUser::Response& response);
    bool relJointMovJ(bringup::RelJointMovJ::Request& request, bringup::RelJointMovJ::Response& response);
    bool movJExt(bringup::MovJExt::Request& request, bringup::MovJExt::Response& response);
    // bool servoJ(bringup::ServoJ::Request& request, bringup::ServoJ::Response& response);
    // bool servoP(bringup::ServoP::Request& request, bringup::ServoP::Response& response);
    bool sync(bringup::Sync::Request& request, bringup::Sync::Response& response);
    // bool startTrace(bringup::StartTrace::Request& request, bringup::StartTrace::Response& response);
    // bool startPath(bringup::StartPath::Request& request, bringup::StartPath::Response& response);
    // bool startFCTrace(bringup::StartFCTrace::Request& request, bringup::StartFCTrace::Response& response);
    bool moveJog(bringup::MoveJog::Request& request, bringup::MoveJog::Response& response);
    bool stopmoveJog(bringup::StopmoveJog::Request& request, bringup::StopmoveJog::Response& response);
    bool syncAll(bringup::SyncAll::Request& request, bringup::SyncAll::Response& response);
    bool wait(bringup::Wait::Request& request, bringup::Wait::Response& response);
    bool Continue(bringup::Continues::Request& request, bringup::Continues::Response& response);
    bool pause(bringup::Pause::Request& request, bringup::Pause::Response& response);

private:
    static int str2Int(const char* val);
    void pubFeedBackInfo();
    std::vector<std::string> regexRecv(std::string getRecvInfo);
    // void feedbackHandle(const ros::TimerEvent& tm,
    //                     actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    // void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle
    // handle);
    // void goalHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    // void cancelHandle(actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
};
