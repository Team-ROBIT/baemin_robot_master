/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/baemin_robot_master/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baemin_robot_master
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/b.png"));

  qnode.init();

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(sigStatusUpdate(bool)), this, SLOT(slotStatusUpdate(bool)));
  QObject::connect(&qnode, SIGNAL(sigBatteryUpdate()), this, SLOT(slotUpdateBattery()));
  QObject::connect(&qnode, SIGNAL(sigRPMUpdate()), this, SLOT(slotUpdateRPM()));
  QObject::connect(&qnode, SIGNAL(sigIMUUpdate()), this, SLOT(slotUpdateIMU()));
  QObject::connect(&qnode, SIGNAL(sigCMDUpdate()), this, SLOT(slotUpdateCMD()));
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::slotStatusUpdate(bool status)
{
  if (status)
  {
    ui.isconnected->setText("true");
    ui.isconnected->setStyleSheet("QLabel { color : green; }");
  }
  else if (!status)
  {
    ui.isconnected->setText("false");
    ui.isconnected->setStyleSheet("QLabel { color : red; }");
  }
}

void MainWindow::slotUpdateBattery()
{
  ui.battery_3->setText(QString::number(qnode.battery_voltage));
}

void MainWindow::slotUpdateRPM()
{
  ui.rpm_L->setText(QString::number(qnode.rpm[1]));
  ui.rpm_R->setText(QString::number(qnode.rpm[2]));
  ui.rpm_A->setText(QString::number(qnode.rpm[0]));
}

void MainWindow::slotUpdateIMU()
{
  ui.imu_lx->setText(QString::number(qnode.imu[0]));
  ui.imu_ly->setText(QString::number(qnode.imu[1]));
  ui.imu_lz->setText(QString::number(qnode.imu[2]));
  ui.imu_ax->setText(QString::number(qnode.imu[3]));
  ui.imu_ay->setText(QString::number(qnode.imu[4]));
  ui.imu_az->setText(QString::number(qnode.imu[5]));
}

void MainWindow::slotUpdateCMD()
{
  ui.linear_x->setText(QString::number(qnode.cmd_vel[0]));
  ui.angular_z->setText(QString::number(qnode.cmd_vel[5]));
}

void MainWindow::on_estop_clicked()
{
  qnode.emergencyStop();
  QMessageBox::information(this, "INFO", "ESTOP DONE. PLEASE RESET ROBOT");
}

void MainWindow::on_btn_hunter_clicked()
{
  if (!hunter_status)
  {
    system("roslaunch hunter_bringup hunter_robot_base.launch &");
    ui.btn_hunter->setStyleSheet(
        "QPushButton {"
        "background-color: green;"
        "border-style: solid;"
        "border-width: 0px;"
        "border-radius: 100px;"
        "border-color: green;"
        "max-width: 220px;"
        "max-height: 220px;"
        "min-width: 220px;"
        "min-height: 220px;"
        "}");
    hunter_status = true;
  }
  else
  {
    system("rosnode kill hunter_base_node &");
    ui.btn_hunter->setStyleSheet(
        "QPushButton {"
        "background-color: red;"
        "border-style: solid;"
        "border-width: 0px;"
        "border-radius: 100px;"
        "border-color: red;"
        "max-width: 220px;"
        "max-height: 220px;"
        "min-width: 220px;"
        "min-height: 220px;"
        "}");
    hunter_status = false;
  }
}

}  // namespace baemin_robot_master
