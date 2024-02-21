#include "rosDemoMG400.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RosDemoMG400");
    ros::NodeHandle nh;

    RosDemoMG400 serviceHandler(&nh);
    std::vector<double> pointA{ 310, 62, -86, 68 };
    std::vector<double> pointB{ 369, -7, -86, 50 };

    // 创建一个新线程，并将obj和data作为引用传递给匿名函数
    std::thread threadMove([&serviceHandler, &pointA, &pointB]() {
        while (true)
        {
            serviceHandler.movePoint(pointA);
            serviceHandler.finishPoint(pointA);
            serviceHandler.movePoint(pointB);
            serviceHandler.finishPoint(pointB);
        }
    });
    threadMove.detach();
    ros::spin();
    return 0;
}
