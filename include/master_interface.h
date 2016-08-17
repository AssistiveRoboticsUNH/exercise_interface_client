#ifndef MASTER_INTERFACE_H
#define MASTER_INTERFACE_H

#include <string>
#include <ctime>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

#include <QMainWindow>
#include <QWidget>
#include <QTimer>


#include "ui_components.h"
#include "script_utils.h"

// The main window of the interface using QT. All onClick functionality is handled here
// When the ui is clicked, typically a rosmessage is sent out to trigger the other
// components of the project
class MasterInterface : public QMainWindow
{
  Q_OBJECT
public:
  explicit MasterInterface(QMainWindow* parent = 0);
  ~MasterInterface();
  //void resetColor(geometry_msgs::Quaternion msg);

private:
  ros::NodeHandle n;

  // These publishers are used to communicate to the other project components when a button is pressed
  ros::Publisher myoLaunch_pub, myoCalibrate_pub, trialBegin_pub, exerciseMode_pub;
  ros::Subscriber myo_sub_l, myo_sub_u, sphinx_sub, score_sub;

  // All gui components are defined and accessible through *uiComponents
  UIComponents *uiComponents;

  //bool myo_l_OK, myo_u_OK, speech_OK;
  void myo_l_detector(geometry_msgs::Quaternion msg);
  void myo_u_detector(geometry_msgs::Quaternion msg);
  void speech_detector(std_msgs::String msg);
  void score_display(std_msgs::Int32 msg);

// These functions are called when their corresponding button is pressed
// to link a button to a function, see: ui_components.h
private Q_SLOTS:  

  void on_myoCalibrate_clicked();
  
  void on_trialStop_clicked();
  void on_trialPractice1_clicked();
  void on_trialPractice2_clicked();
  void on_trialPractice3_clicked();
  void on_skipDemo_clicked();
  void on_home_clicked();

};
#endif 
