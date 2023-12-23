//
// Created by zhangran on 2021/10/20.
//

#include <QApplication>
#include "main_window.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "joint_publisher_gui");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    QApplication app(argc, argv);
    MainWindow main_window(nh);
    main_window.show();

    return QApplication::exec();
}