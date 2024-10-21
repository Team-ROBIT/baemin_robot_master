/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/baemin_robot_master/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baemin_robot_master
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
  laser_topic_list.clear();
  laser_topic_list.push_back("/lidar1/scan");
  laser_topic_list.push_back("/lidar2/scan");
  laser_topic_list.push_back("/lidar3/scan");
  laser_topic_list.push_back("/lidar4/scan");
  laser_topic_previous_time.resize(laser_topic_list.size());
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  delete timer10ms;
  delete timer1s;
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "baemin_robot_master");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.

  readParams();

  // Publishers
  comm_pub = n.advertise<std_msgs::Bool>("/comm_stat/robot", 1);

  // Subscribers
  eStop = n.subscribe<std_msgs::Bool>("ESTOP", 1, &QNode::eStopCallback, this);
  comm_sub = n.subscribe<std_msgs::Bool>("/comm_stat/operator", 1, &QNode::commStatusCallback, this);
  BMS_sub = n.subscribe<hunter_msgs::HunterBmsStatus>("/BMS_status", 1, &QNode::BMSStatusCallback, this);
  hunter_status_sub = n.subscribe<hunter_msgs::HunterStatus>("/hunter_status", 1, &QNode::hunterStatusCallback, this);
  imu_sub = n.subscribe<sensor_msgs::Imu>(imu_topic.c_str(), 1, &QNode::imuCallback, this);
  cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, &QNode::cmdCallback, this);

  laser_sub_vec.clear();
  for (size_t i = 0; i < laser_topic_list.size(); i++)
  {
    if (!laser_topic_list[i].empty())
    {
      ROS_INFO("laser_topic_list[%zu] : %s", i, laser_topic_list[i].c_str());
      laser_sub_vec.push_back(
          n.subscribe<sensor_msgs::LaserScan>(laser_topic_list[i], 1, boost::bind(&QNode::laserCallback, this, _1, i)));
    }
    else
    {
      ROS_WARN("Empty laser topic at index %zu", i);
    }
  }

  timer10ms = new QTimer(this);
  timer1s = new QTimer(this);
  connect(timer10ms, &QTimer::timeout, this, &QNode::onTimer10ms);
  connect(timer1s, &QTimer::timeout, this, &QNode::onTimer1s);
  timer10ms->start(10);
  timer1s->start(1000);

  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(33);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::readParams()
{
  std::string data;
  ros::param::get("/baemin_robot_master/imu_topic", data);
  std::cout << "[baemin_robot_master] IMU topic : " << data.c_str() << std::endl;
  imu_topic = data;
}

void QNode::commStatusCallback(const std_msgs::BoolConstPtr& stat)
{
  comm_cnt = 0;
  Q_EMIT sigStatusUpdate(true);
}

void QNode::BMSStatusCallback(const hunter_msgs::HunterBmsStatusConstPtr& stat)
{
  battery_voltage = stat->battery_voltage;
  Q_EMIT sigBatteryUpdate();
}

void QNode::hunterStatusCallback(const hunter_msgs::HunterStatusConstPtr& stat)
{
  for (int i = 0; i < 3; i++)
  {
    rpm[i] = stat->motor_states[i].rpm;
  }
  Q_EMIT sigRPMUpdate();
}

void QNode::imuCallback(const sensor_msgs::ImuConstPtr& imu_)
{
  imu[0] = imu_->linear_acceleration.x;
  imu[1] = imu_->linear_acceleration.y;
  imu[2] = imu_->linear_acceleration.z;
  imu[3] = imu_->angular_velocity.x;
  imu[4] = imu_->angular_velocity.y;
  imu[5] = imu_->angular_velocity.z;
  Q_EMIT sigIMUUpdate();
}

void QNode::cmdCallback(const geometry_msgs::TwistConstPtr& cmd)
{
  cmd_vel[0] = cmd->linear.x;
  cmd_vel[1] = cmd->linear.y;
  cmd_vel[2] = cmd->linear.z;
  cmd_vel[3] = cmd->angular.x;
  cmd_vel[4] = cmd->angular.y;
  cmd_vel[5] = cmd->angular.z;
  Q_EMIT sigCMDUpdate();
}

void QNode::laserCallback(const sensor_msgs::LaserScanConstPtr& laser, int num)
{
  if (num < 0 || num >= static_cast<int>(laser_topic_previous_time.size()))
  {
    ROS_ERROR("Invalid laser callback number: %d", num);
    return;
  }

  ros::Time laser_topic_current_time = laser->header.stamp;
  if (!laser_topic_previous_time[num].isZero())
  {
    double time_diff = (laser_topic_current_time - laser_topic_previous_time[num]).toSec();
    if (time_diff > 1.0)
    {
      ROS_ERROR("LIDAR[%d] TIMEOUT. PLEASE CHECK YOUR CONNECTION.", num + 1);
      Q_EMIT sigLidarTimeout(num);
    }
    else if (time_diff < 1.0)
    {
      // ROS_INFO("LIDAR[%d] OK", num + 1);
      Q_EMIT sigLidarOK(num);
    }
  }
  laser_topic_previous_time[num] = laser_topic_current_time;
}

void QNode::eStopCallback(const std_msgs::BoolConstPtr& estop)
{
  emergencyStop();
}

void QNode::onTimer10ms()
{
  std_msgs::Bool boolean_msg;
  boolean_msg.data = true;
  comm_pub.publish(boolean_msg);
}

void QNode::onTimer1s()
{
  // comm_cnt++;
  // if (comm_cnt >= 5)
  // {
  //   ROS_ERROR("COMMUNICATION LOST. PLEASE CHECK YOUR CONNECTION.");
  //   Q_EMIT sigStatusUpdate(false);
  // }
}

void QNode::emergencyStop()
{
}

}  // namespace baemin_robot_master
