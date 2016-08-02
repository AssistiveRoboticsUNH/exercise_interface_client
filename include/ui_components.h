#ifndef UI_ASDINTERFACE_H
#define UI_ASDINTERFACE_H

#include <QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QHeaderView>
#include <QLCDNumber>
#include <QPushButton>
#include <QWidget>
#include <QLabel>
#include <QComboBox>

QT_BEGIN_NAMESPACE

// Handles the specifics of the UI layout of the interface
class UIComponents {

public:

    static const int BSPACING = 80, BWIDTH = 300, BHEIGHT = 150,
                     COL1_X = 60, COL1_Y = 140, 
                     COL2_X = 450, COL2_Y = COL1_Y,
                     COL3_X = 840, COL3_Y = COL1_Y,
                     COL4_X = 1230, COL4_Y = COL1_Y;

    QPushButton *trialPractice1, *trialPractice2, *trialPractice3, 
                *trialStop, *myoCalibrate, *skipDemo, *home;
    QLabel *MyoLabel1, *MyoLabel2, *TabletLabel, *GlassLabel, 
                *SpeechOutputLabel, *scoreLabel;

    void setupUi(QWidget *MasterInterface) {
    
        // Create and size window    
        
        if(MasterInterface->objectName().isEmpty()) {
            MasterInterface->setObjectName(QString("MasterInterface"));
        }

        MasterInterface->resize(1640, 1140);

        /**First Column**/
        
        trialPractice1 = new QPushButton(MasterInterface);
        trialPractice1->setObjectName(QString("trialPractice1"));
        trialPractice1->setGeometry(QRect(COL1_X, COL1_Y+BHEIGHT+BSPACING, BWIDTH, BHEIGHT)); 

        skipDemo = new QPushButton(MasterInterface);
        skipDemo->setObjectName(QString("skipDemo"));
        skipDemo->setGeometry(QRect(COL1_X, COL1_Y+2*(BHEIGHT+BSPACING), BWIDTH, BHEIGHT));

        /**Second Column**/
        
        myoCalibrate = new QPushButton(MasterInterface);
        myoCalibrate->setObjectName(QString("myoCalibrate"));
        myoCalibrate->setGeometry(QRect(COL2_X, COL2_Y, BWIDTH, BHEIGHT));

        trialPractice2 = new QPushButton(MasterInterface);
        trialPractice2->setObjectName(QString("trialPractice2"));
        trialPractice2->setGeometry(QRect(COL2_X, COL2_Y+BHEIGHT+BSPACING, BWIDTH, BHEIGHT)); 
        
        trialStop = new QPushButton(MasterInterface);
        trialStop->setObjectName(QString("trialStop"));
        trialStop->setGeometry(QRect(COL2_X, COL2_Y+2*(BHEIGHT+BSPACING), BWIDTH, BHEIGHT)); 


        /**Third Column**/
        
        scoreLabel = new QLabel(MasterInterface);
        scoreLabel->setText(QString(""));
        scoreLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        scoreLabel->setGeometry(QRect(COL3_X, COL3_Y, BWIDTH, BHEIGHT));
        
        trialPractice3 = new QPushButton(MasterInterface);
        trialPractice3->setObjectName(QString("trialPractice3"));
        trialPractice3->setGeometry(QRect(COL3_X, COL3_Y+(BHEIGHT+BSPACING), BWIDTH, BHEIGHT)); 

        home = new QPushButton(MasterInterface);
        home->setObjectName(QString("home"));
        home->setGeometry(QRect(COL3_X, COL3_Y+2*(BHEIGHT+BSPACING), BWIDTH, BHEIGHT)); 

        /**Fourth Column**/
        
        MyoLabel1 = new QLabel(MasterInterface);
        MyoLabel1->setText(QString("<h1><font color='red'>Myo: Lower Arm</font></h1>"));
        MyoLabel1->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        MyoLabel1->setGeometry(QRect(COL4_X, COL4_Y, BWIDTH, 100));
 
        MyoLabel2 = new QLabel(MasterInterface);
        MyoLabel2->setText(QString("<h1><font color='red'>Myo: Upper Arm</font></h1>"));
        MyoLabel2->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        MyoLabel2->setGeometry(QRect(COL4_X, COL4_Y+120, BWIDTH, 100));       

//        TabletLabel = new QLabel(MasterInterface);
//        TabletLabel->setText(QString("<h1><font color='red'>Tablet</font></h1>"));
//        TabletLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
//        TabletLabel->setGeometry(QRect(COL4_X, COL4_Y+240, BWIDTH, 100));

//        GlassLabel = new QLabel(MasterInterface);
//        GlassLabel->setText(QString("<h1><font color='red'>Eyeglass</font></h1>"));
//        GlassLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
//        GlassLabel->setGeometry(QRect(COL4_X, COL4_Y+360, BWIDTH, 100));

        SpeechOutputLabel = new QLabel(MasterInterface);
        SpeechOutputLabel->setText(QString("<h1><font color='red'>Speech Detected: None</font></h1>"));
        SpeechOutputLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
        SpeechOutputLabel->setGeometry(QRect(COL4_X, COL4_Y+240, BWIDTH, 100));
        /**Polish**/
        
        retranslateUi(MasterInterface);

        // ! Connects the buttons to any listener functions in the MasterInterface object !
        // Maps buttons ObjectName to a onClick function. 
        // For example, on_myoLaunch_clicked() maps to the myoLaunch button which is declared above
        QMetaObject::connectSlotsByName(MasterInterface);
    }

    // Name window and buttons to be more user friendly  
    void retranslateUi(QWidget *MasterInterface){

        QFont f;
        f.setPointSize(22);
        MasterInterface->setWindowTitle(QApplication::translate("MasterInterface", "MasterInterface", 0));

        trialPractice1->setText("Task 1");
        trialPractice1->setFont(f);
        trialPractice2->setText("Task 2");
        trialPractice2->setFont(f);
        trialPractice3->setText("Task 3");
        trialPractice3->setFont(f);
        trialStop->setText("Stop Practice");
        trialStop->setFont(f);
        myoCalibrate->setText("Calibrate Myo");
        myoCalibrate->setFont(f);
        skipDemo->setText("Skip Demo");
        skipDemo->setFont(f);
        home->setText("Home/Reset");
        home->setFont(f);
   }

    // Because the character will be a number in ASCII format, if we subtract the '0' character, it will convert it to its number form
    int getMyoCount() {
        //return myoCount->currentText().toStdString()[0] - '0';
        return 2;
    } 

};

QT_END_NAMESPACE

#endif
