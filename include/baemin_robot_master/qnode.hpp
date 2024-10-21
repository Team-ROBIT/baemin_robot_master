/**
 * @file /include/baemin_robot_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef baemin_robot_master_QNODE_HPP_
#define baemin_robot_master_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <vector>
#include <QThread>
#include <QStringListModel>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QStringList>
#include <QTimer>

#include <hunter_msgs/HunterBmsStatus.h>
#include <hunter_msgs/HunterStatus.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baemin_robot_master
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  void emergencyStop();

  double battery_voltage = 0.0;
  double rpm[3] = {
    0,
  };

  double imu[6] = {
    0,
  };

  double cmd_vel[6] = {
    0,
  };

Q_SIGNALS:
  void rosShutdown();
  void sigStatusUpdate(bool status);
  void sigBatteryUpdate();
  void sigRPMUpdate();
  void sigIMUUpdate();
  void sigCMDUpdate();
  void sigLidarTimeout(int num);
  void sigLidarOK(int num);

public Q_SLOTS:

private:
  int init_argc;
  char** init_argv;

  QTimer* timer10ms;
  QTimer* timer1s;

  void readParams();

  std::string imu_topic;

  ros::Publisher comm_pub;
  ros::Subscriber comm_sub;
  int comm_cnt = 0;
  void commStatusCallback(const std_msgs::BoolConstPtr& stat);

  ros::Subscriber eStop;
  void eStopCallback(const std_msgs::BoolConstPtr& estop);

  ros::Subscriber BMS_sub;
  ros::Subscriber hunter_status_sub;
  void BMSStatusCallback(const hunter_msgs::HunterBmsStatusConstPtr& stat);
  void hunterStatusCallback(const hunter_msgs::HunterStatusConstPtr& stat);

  ros::Subscriber imu_sub;
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_);

  ros::Subscriber cmd_vel_sub;
  void cmdCallback(const geometry_msgs::TwistConstPtr& cmd);

  std::vector<ros::Subscriber> laser_sub_vec;
  std::vector<std::string> laser_topic_list;
  std::vector<ros::Time> laser_topic_previous_time;
  void laserCallback(const sensor_msgs::LaserScanConstPtr& laser, int num);

private slots:
  void onTimer10ms();
  void onTimer1s();
};

}  // namespace baemin_robot_master

#endif /* baemin_robot_master_QNODE_HPP_ */
