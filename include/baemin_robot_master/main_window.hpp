/**
 * @file /include/baemin_robot_master/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef baemin_robot_master_MAIN_WINDOW_H
#define baemin_robot_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include <QMessageBox>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace baemin_robot_master
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

public Q_SLOTS:
  void slotStatusUpdate(bool status);
  void slotUpdateBattery();
  void slotUpdateRPM();
  void slotUpdateIMU();
  void slotUpdateCMD();

  void on_estop_clicked();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace baemin_robot_master

#endif  // baemin_robot_master_MAIN_WINDOW_H
