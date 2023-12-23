/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#pragma once

#include <QWidget>
#include <ros/ros.h>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

/**
 *
 */
class MainWindow : public QWidget
{
    Q_OBJECT

public:
    // -2.82 2.82
    // -0.52 1.57
    // 0 1.57
    // -3.14 3.14

    static constexpr double J1_MIN = -2.82;
    static constexpr double J1_MAX = 2.82;
    static constexpr double J2_MIN = -0.52;
    static constexpr double J2_MAX = 1.57;
    static constexpr double J3_MIN = 0;
    static constexpr double J3_MAX = 1.57;
    static constexpr double J4_MIN = -3.14;
    static constexpr double J4_MAX = 3.14;

private:
    ros::Timer tm_;
    Ui::MainWindow* ui_;
    ros::NodeHandle& nh_;
    ros::Publisher joint_state_pub_;

    double j1_;
    double j2_;
    double j3_;
    double j4_;

public:
    explicit MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~MainWindow() override;

private:
    void randomBtn();
    void centerBtn();
    void j1ValueChange(int value);
    void j2ValueChange(int value);
    void j3ValueChange(int value);
    void j4ValueChange(int value);
    void publishJointStates(const ros::TimerEvent& evt);
};
