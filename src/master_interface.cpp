#include "../include/master_interface.h"
#include <ctime>
#include <iostream>
#include <string>
using namespace std;

// defines the functionality of the MasterInterface class

// Basic constructor for the MasterInterface. Sets up the
// ros objects and starts the ros
MasterInterface::MasterInterface(QMainWindow* parent) : QMainWindow(parent), uiComponents(new UIComponents()) {
    uiComponents->setupUi(this);
    ros::start();

    trialBegin_pub = n.advertise<std_msgs::Int32>("/exercise/begin_trial", 100);
    myoLaunch_pub = n.advertise<std_msgs::Int32>("/myo/launch", 100);
    myoCalibrate_pub = n.advertise<std_msgs::Int32>("/myo/calibrate", 100);
    exerciseMode_pub = n.advertise<std_msgs::Int32>("/exercise/mode", 100);

    myo_sub_l = n.subscribe("/myo/l/ort", 10, &MasterInterface::myo_l_detector, this);
    myo_sub_u = n.subscribe("/myo/u/ort", 10, &MasterInterface::myo_u_detector, this);
    //score_sub = n.subscribe("/exercise/score", 1, &MasterInterface::score_display, this);

    sphinx_sub = n.subscribe("/recognizer/output", 1, &MasterInterface::speech_detector, this);

    // unallows the user to change the size of the window
    setFixedSize(size());
}

void MasterInterface::myo_l_detector(geometry_msgs::Quaternion msg) {
    uiComponents->MyoLabel1->setText(QString("<h1><font color='green'>Myo: Lower Arm</font></h1>"));
    myo_sub_l.shutdown();
}

void MasterInterface::myo_u_detector(geometry_msgs::Quaternion msg) {
    uiComponents->MyoLabel2->setText(QString("<h1><font color='green'>Myo: Upper Arm</font></h1>"));
    myo_sub_u.shutdown();
}

void MasterInterface::score_display(std_msgs::Int32 msg) {
    //uiComponents->scoreLabel->setText(QString("<h1><font color='green'>Your score is</font> <font color='black'> %1 </font></h1>").arg(msg.data));
}

void MasterInterface::speech_detector(std_msgs::String msg) {
    QString output;
    std_msgs::Int32 display;
    
    output = QString("<h1><font color='green'>Speech Detected:<font size='5'> ") + QString(msg.data.c_str()) + QString("</font></font></h1>");
    uiComponents->SpeechOutputLabel->setText(output);

    if (msg.data == string("calibration")) {
        std_msgs::Int32 msg;
        
        msg.data = uiComponents->getMyoCount();
        myoCalibrate_pub.publish(msg);
        
        display.data = 2;
        exerciseMode_pub.publish(display);
        return;
    }
    if (msg.data==string("task one") || msg.data==string("task two") || msg.data==string("task three") || msg.data==string("home")) {
        display.data = 1;
        exerciseMode_pub.publish(display);
        return;
    }
    if (msg.data == string("stop")) {
        //uiComponents->scoreLabel->setText(QString("<h1><font color='green'>Computing your score ... </font></h1>"));
        display.data = -1;
        exerciseMode_pub.publish(display);
        sleep(2);
        display.data = 2;
        exerciseMode_pub.publish(display);
        return;
    }
    if (msg.data == string("skip")) {
        display.data = -2;
        exerciseMode_pub.publish(display);
    }
    if (msg.data == string("home")) {
        display.data = 100;
        exerciseMode_pub.publish(display);
        return;
    }
}

// Signals to enter the practice mode (exercise evaluation & mimicking)
void MasterInterface::on_trialPractice1_clicked() {
    std_msgs::Int32 msg;
    msg.data = 10;

    exerciseMode_pub.publish(msg);
}

void MasterInterface::on_trialPractice2_clicked() {
    std_msgs::Int32 msg;
    msg.data = 20;

    exerciseMode_pub.publish(msg);
}

void MasterInterface::on_trialPractice3_clicked() {
    std_msgs::Int32 msg;
    msg.data = 30;

    exerciseMode_pub.publish(msg);
}

void MasterInterface::on_trialStop_clicked() {
    std_msgs::Int32 msg;
    msg.data = -1;
    //uiComponents->scoreLabel->setText(QString("<h1><font color='green'>Computing your score ... </font></h1>"));

    exerciseMode_pub.publish(msg);
    sleep(2);
    msg.data = 2;
    exerciseMode_pub.publish(msg);
}

void MasterInterface::on_skipDemo_clicked() {
    std_msgs::Int32 msg;
    msg.data = -2;

    exerciseMode_pub.publish(msg);
}

void MasterInterface::on_home_clicked() {
    std_msgs::Int32 msg;
    msg.data = 100;

    exerciseMode_pub.publish(msg);
}

// Signals to reset the orientation component in the myo which is relative to north
void MasterInterface::on_myoCalibrate_clicked() {
    std_msgs::Int32 msg, display;
    msg.data = uiComponents->getMyoCount();
    myoCalibrate_pub.publish(msg);
    
    display.data = 2;
    exerciseMode_pub.publish(display);
}

MasterInterface::~MasterInterface() {
    delete uiComponents;
}
