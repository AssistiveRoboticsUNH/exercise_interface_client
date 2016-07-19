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

    static const int BSPACING = 20, BWIDTH = 200, BHEIGHT = 100,
                     COL1_X = 40, COL1_Y = 140, 
                     COL2_X = 300, COL2_Y = COL1_Y,
                     COL3_X = 560, COL3_Y = COL1_Y;

    QPushButton *trialPractice1, *trialPractice2, *trialPractice3, 
                *trialStop, *myoCalibrate;
    

    void setupUi(QWidget *MasterInterface) {
    
        // Create and size window    
        
        if(MasterInterface->objectName().isEmpty()) {
            MasterInterface->setObjectName(QString("MasterInterface"));
        }

        MasterInterface->resize(820, 520);

        /**First Column**/
        
        trialPractice1 = new QPushButton(MasterInterface);
        trialPractice1->setObjectName(QString("trialPractice1"));
        trialPractice1->setGeometry(QRect(COL1_X, COL1_Y+BHEIGHT+BSPACING, BWIDTH, BHEIGHT)); 

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
        
        trialPractice3 = new QPushButton(MasterInterface);
        trialPractice3->setObjectName(QString("trialPractice3"));
        trialPractice3->setGeometry(QRect(COL3_X, COL3_Y+(BHEIGHT+BSPACING), BWIDTH, BHEIGHT)); 


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
        f.setPointSize(16);
        MasterInterface->setWindowTitle(QApplication::translate("MasterInterface", "MasterInterface", 0));

        trialPractice1->setText("Task 1");
        trialPractice1->setFont(f);
        trialPractice2->setText("Task 2");
        trialPractice2->setFont(f);
        trialPractice3->setText("Task 3");
        trialPractice3->setFont(f);
        trialStop->setText("Stop");
        trialStop->setFont(f);
        myoCalibrate->setText("Calibrate Myo");
        myoCalibrate->setFont(f);
   }

    // Because the character will be a number in ASCII format, if we subtract the '0' character, it will convert it to its number form
    int getMyoCount() {
        //return myoCount->currentText().toStdString()[0] - '0';
        return 2;
    } 

};

QT_END_NAMESPACE

#endif
