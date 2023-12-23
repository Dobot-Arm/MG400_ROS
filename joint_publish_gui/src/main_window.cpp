/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include "main_window.h"
#include "ui_main_window.h"

#include <bringup/convert.h>
#include <sensor_msgs/JointState.h>

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent) : QWidget(parent), nh_(nh), ui_(new Ui::MainWindow)
{
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 100);
    tm_ = nh_.createTimer(ros::Rate(50), &MainWindow::publishJointStates, this);
    tm_.start();
    ui_->setupUi(this);

    ui_->j1_txt->setText("0.000");
    ui_->j2_txt->setText("0.000");
    ui_->j3_txt->setText("0.000");
    ui_->j4_txt->setText("0.000");
    j1_ = 0.000;
    j2_ = 0.000;
    j3_ = 0.000;
    j4_ = 0.000;
    ui_->j1_slider->setValue(std::abs(J1_MIN) / (J1_MAX - J1_MIN) * 1000);
    ui_->j2_slider->setValue(std::abs(J2_MIN) / (J2_MAX - J2_MIN) * 1000);
    ui_->j3_slider->setValue(std::abs(J3_MIN) / (J3_MAX - J3_MIN) * 1000);
    ui_->j4_slider->setValue(std::abs(J4_MIN) / (J4_MAX - J4_MIN) * 1000);
    connect(ui_->j1_slider, &QSlider::valueChanged, this, &MainWindow::j1ValueChange);
    connect(ui_->j2_slider, &QSlider::valueChanged, this, &MainWindow::j2ValueChange);
    connect(ui_->j3_slider, &QSlider::valueChanged, this, &MainWindow::j3ValueChange);
    connect(ui_->j4_slider, &QSlider::valueChanged, this, &MainWindow::j4ValueChange);

    connect(ui_->random_btn, &QPushButton::clicked, this, &MainWindow::randomBtn);
    connect(ui_->center_btn, &QPushButton::clicked, this, &MainWindow::centerBtn);
}

MainWindow::~MainWindow()
{
    delete ui_;
}

void MainWindow::publishJointStates(const ros::TimerEvent& evt)
{
    joint_state_pub_.publish(Convert::toJointState(j1_, j2_, j3_, j4_));
}

void MainWindow::j1ValueChange(int value)
{
    char str[50];
    j1_ = J1_MIN + (J1_MAX - J1_MIN) * value / 1000;
    sprintf(str, "%0.3f", j1_);
    ui_->j1_txt->setText(str);
}

void MainWindow::j2ValueChange(int value)
{
    char str[50];
    j2_ = J2_MIN + (J2_MAX - J2_MIN) * value / 1000;
    sprintf(str, "%0.3f", j2_);
    ui_->j2_txt->setText(str);
}

void MainWindow::j3ValueChange(int value)
{
    char str[50];
    j3_ = J3_MIN + (J3_MAX - J3_MIN) * value / 1000;
    sprintf(str, "%0.3f", j3_);
    ui_->j3_txt->setText(str);
}

void MainWindow::j4ValueChange(int value)
{
    char str[50];
    j4_ = J4_MIN + (J4_MAX - J4_MIN) * value / 1000;
    sprintf(str, "%0.3f", j4_);
    ui_->j4_txt->setText(str);
}

void MainWindow::randomBtn()
{
    int32_t val = random() % 1000;
    j1ValueChange(val);
    ui_->j1_slider->setValue(val);

    val = random() % 1000;
    j2ValueChange(val);
    ui_->j2_slider->setValue(val);

    val = random() % 1000;
    j3ValueChange(val);
    ui_->j3_slider->setValue(val);

    val = random() % 1000;
    j4ValueChange(val);
    ui_->j4_slider->setValue(val);
}

void MainWindow::centerBtn()
{
    j1ValueChange(std::abs(J1_MIN) / (J1_MAX - J1_MIN) * 1000);
    j2ValueChange(std::abs(J2_MIN) / (J2_MAX - J2_MIN) * 1000);
    j3ValueChange(std::abs(J3_MIN) / (J3_MAX - J3_MIN) * 1000);
    j4ValueChange(std::abs(J4_MIN) / (J4_MAX - J4_MIN) * 1000);

    ui_->j1_slider->setValue(std::abs(J1_MIN) / (J1_MAX - J1_MIN) * 1000);
    ui_->j2_slider->setValue(std::abs(J2_MIN) / (J2_MAX - J2_MIN) * 1000);
    ui_->j3_slider->setValue(std::abs(J3_MIN) / (J3_MAX - J3_MIN) * 1000);
    ui_->j4_slider->setValue(std::abs(J4_MIN) / (J4_MAX - J4_MIN) * 1000);
}
