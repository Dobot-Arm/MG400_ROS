#include "rosDemoMG400.h"
static const char* kAlarmServoJsonFile = "../alarmFile/alarm_servo.json";
static const char* kAlarmControllerJsonFile = "../alarmFile/alarm_controller.json";
// Include other necessary headers

RosDemoMG400::RosDemoMG400(ros::NodeHandle* nh)
{
    m_enableRobot = nh->serviceClient<rosdemo::EnableRobot>("/bringup/srv/EnableRobot");
    m_disableRobot = nh->serviceClient<rosdemo::DisableRobot>("/bringup/srv/DisableRobot");
    m_clearError = nh->serviceClient<rosdemo::ClearError>("/bringup/srv/ClearError");
    m_getErrorID = nh->serviceClient<rosdemo::GetErrorID>("/bringup/srv/GetErrorID");
    m_movl = nh->serviceClient<rosdemo::MovL>("/bringup/srv/MovL");
    m_continue = nh->serviceClient<rosdemo::Continues>("/bringup/srv/Continue");
    m_sync = nh->serviceClient<rosdemo::Sync>("/bringup/srv/Sync");

    subFeedInfo = nh->subscribe("/bringup/msg/FeedInfo", 10, &RosDemoMG400::getFeedBackInfo, this);
    // Add more services if needed

    threadParseRobotError = std::thread(&RosDemoMG400::parseRobotAlarm, this);
    threadParseRobotError.detach();

    threadClearRobotError = std::thread(&RosDemoMG400::clearRobotError, this);
    threadClearRobotError.detach();
}

void RosDemoMG400::getFeedBackInfo(const std_msgs::String::ConstPtr& msg)
{
    std::string feedIndo = msg->data;

    // 从 JSON 字符串中提取数组
    nlohmann::json parsedJson = nlohmann::json::parse(feedIndo);

    std::unique_lock<std::shared_mutex> lockInfo(m_mutex);
    if (parsedJson.count("EnableStatus") && parsedJson["EnableStatus"].is_number())
    {
        feedbackData.EnableStatus = parsedJson["EnableStatus"];
    }
    if (parsedJson.count("ErrorStatus") && parsedJson["ErrorStatus"].is_number())
    {
        feedbackData.ErrorStatus = parsedJson["ErrorStatus"];
    }
    if (parsedJson.count("RunQueuedCmd") && parsedJson["RunQueuedCmd"].is_number())
    {
        feedbackData.RunQueuedCmd = parsedJson["RunQueuedCmd"];
    }

    if (parsedJson.count("ToolVectorActual") && parsedJson["ToolVectorActual"].is_array())
    {
        for (int i = 0; i < 6; i++)
        {
            feedbackData.ToolVectorActual[i] = parsedJson["ToolVectorActual"][i];
        }
    }
}

void RosDemoMG400::clearRobotError()
{
    while (true)
    {
        {
            std::shared_lock<std::shared_mutex> lockInfo(m_mutex);
            if (feedbackData.ErrorStatus && feedbackData.EnableStatus)

            {
                rosdemo::GetErrorID srvGetError;

                // 请求服务
                if (SendService(m_getErrorID, srvGetError))
                {
                    for (int i = 0; i < srvGetError.response.errorID.size(); i++)
                    {
                        bool alarmState{ false };
                        for (const auto& alarmControllerJson : m_JsonDataController)
                        {
                            if (static_cast<int>(srvGetError.response.errorID[i]) ==
                                static_cast<int>(alarmControllerJson["id"]))
                            {
                                ROS_ERROR("Control ErrorID : %d,  %s, %s", srvGetError.response.errorID[i],
                                          static_cast<std::string>(alarmControllerJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmControllerJson["en"]["description"]).c_str());
                                alarmState = true;
                                break;
                            }
                        }

                        if (alarmState)
                        {
                            continue;
                        }

                        for (const auto& alarmServoJson : m_JsonDataServo)
                        {
                            if (static_cast<int>(srvGetError.response.errorID[i]) ==
                                static_cast<int>(alarmServoJson["id"]))
                            {
                                ROS_ERROR("Servo ErrorID : %d,  %s, %s", srvGetError.response.errorID[i],
                                          static_cast<std::string>(alarmServoJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmServoJson["en"]["description"]).c_str());
                                break;
                            }
                        }
                    }
                    rosdemo::ClearError srvClearError;

                    // 请求服务
                    if (!SendService(m_clearError, srvClearError))
                    {
                        ROS_ERROR("clearError  service  fail");
                        continue;
                    }
                    rosdemo::Continues srvContinue;

                    // 请求服务
                    if (!SendService(m_continue, srvContinue))
                    {
                        ROS_ERROR("Continue service  fail");
                        continue;
                    }
                }
                else
                {
                    ROS_ERROR("geterrorid service  fail");
                }
            }
            else
            {
                if (feedbackData.EnableStatus && (!feedbackData.RunQueuedCmd))
                {
                    rosdemo::Continues srvContinue;
                    if (!SendService(m_continue, srvContinue))
                    {
                        ROS_ERROR("clearError  service  fail");
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
}

void RosDemoMG400::movePoint(std::vector<double>& pointA)
{
    if (pointA.size() < 4U)
    {
        ROS_ERROR("MOVL params is  ERROR");
    }
    else
    {
        rosdemo::MovL srvMovL;
        srvMovL.request.x = pointA[0];
        srvMovL.request.y = pointA[1];
        srvMovL.request.z = pointA[2];
        srvMovL.request.r = pointA[3];
        if (!SendService(m_movl, srvMovL))
        {
            ROS_ERROR("MOVL service fail");
        }
    }
}

void RosDemoMG400::finishPoint(std::vector<double>& point)
{
    if (point.size() < 4U)
    {
        ROS_ERROR("MOVL params size is ERROR");
    }

    auto compareSize = [=](double value1, double value2) { return std::abs(value1 - value2) < 1; };
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::shared_lock<std::shared_mutex> lockInfo(m_mutex);
        for (int i = 0; i < 4; i++)
        {
            stateFinish = compareSize(feedbackData.ToolVectorActual[i], point[i]);
            if (!stateFinish)
            {
                break;
            }
        }
        if (stateFinish)
        {
            break;
        }
    }
}

void RosDemoMG400::parseRobotAlarm()
{
    // 获取当前cpp文件所在的目录路径
    std::string currentDirectory = __FILE__;
    size_t lastSlash = currentDirectory.find_last_of("/");
    std::string directory = currentDirectory.substr(0, lastSlash + 1);

    // 拼接目标json文件的路径
    std::string jsonServoFilePath = directory + kAlarmServoJsonFile;
    std::string jsonControllerFilePath = directory + kAlarmControllerJsonFile;

    auto readJsonFile = [](const std::string& filePath, nlohmann::json& jsonData) {
        std::ifstream jsonFile(filePath);
        if (jsonFile.is_open())
        {
            jsonFile >> jsonData;

            // 在这里使用 jsonData 对象访问json数据
            jsonFile.close();    // 关闭文件流，释放资源
        }
        else
        {
            ROS_ERROR("Failed to open json file : %s", filePath.c_str());
        }
    };

    readJsonFile(jsonServoFilePath, m_JsonDataServo);
    readJsonFile(jsonControllerFilePath, m_JsonDataController);
}