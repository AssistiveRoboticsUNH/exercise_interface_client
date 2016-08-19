Setup
=====
To run the interface you will need the following packages in the same catkin workspace:  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- myo_raw - https://github.com/ylmeng/myo_raw  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- (optional) myo_nao - https://github.com/idkm23/myo_nao  
  
##### Envirnonment setup  
```
sudo apt-get install python-pip
pip install --upgrade pip
sudo pip install scipy  
sudo pip install sklearn  
sudo adduser <username> dialout 
sudo apt-get install ros-indigo-qt-build 
sudo apt-get install ros-indigo-pocketsphinx
sudo apt-get install gstreamer0.10-gconf
``` 

If you are completely new to ros, install rosindigo. Then, go to a directory you would like the project in and do:
```
mkdir myo
cd myo
mkdir src
cd src
catkin_init_workspace
git clone https://github.com/ylmeng/myo_raw.git
git clone https://github.com/idkm23/myo_nao.git        # DO NOT DO THIS IF YOU HAVE NOT CONFIGURED THE NAO ON YOUR SYSTEM
git clone https://github.com/idkm23/exercise_interface.git
cd ..
catkin_make
source devel/setup.bash
```

Now your project is configured properly and you may launch the interface with:  
```roslaunch exercise_interface_client interface_client.launch``` 

There is also a bash script which automatically launches exercise_interface_client with some configurations. Please refer to https://github.com/ylmeng/myo_raw/blob/master/scripts/launch_client.sh
If you have correctly installed the myo_raw package, you should find it under the 'script' folder in that package. Go to that directory to run the script. In the script you may need to replace 'wlan2' with another keyword (e.g. 'eth0') depending on how your network is set up. Use the command ```ifconfig``` to check the setup. Also replace ```source ~/myocp/myo/devel/setup.bash``` with the correct path. 
   
**If you want to use the Nao in the interface you must do:**  
```NAO_IP=10.0.3.16 roslaunch exercise_interface interface.launch using_nao:=true```  
(hit the button on the Nao's chest if you don't know its IP)  

To install the nao sdk and other basics see this tutorial:
http://wiki.ros.org/nao/Tutorials/Installation  
You will need to login and download something from here: https://community.aldebaran.com/ so contact Momotaz for this information.
Also install this repo, as the naoqi has changed so we have preserved our version:  
```git clone https://github.com/idkm23/naoqi.git```

How to use the client interface
===
This interface is for patients to practice the exercises. Before you launch the program, make sure the data files in ```myo_raw/myo_mdp/data1```, ```myo_raw/myo_mdp/data2```, and ```myo_raw/myo_mdp/data3```are up to date. All the buttons have voice control functions too, so you can either click the buttons or use voice control.
- After you launch the interface, you should see a control panel. The Myo armbands should launch automatically. The corrsponding captions on the interface should turn green in a few seconds.
- The speech recognition program will launch automatically. Say the word "help" to test it. The corresponding caption should turn green.
- On the Android device, launch the Exercise Learner program. You should be able to see a 3D model.
- Say "calibration" (or click "Calibrate Myo") to calibrate Myo. You should be able to see your arm motion from the 3D model too.
- Say "task one" (or click "Task 1") to start practicing the first exercise. Or say "Task two", "task three" as you want.
- The program will demonstrate the task for you. Say "skip" if you do not want to see the demonstration.
- When you practice the exercise, you can always say "help" to get prompts.
- Say "stop" (or click "Stop Practice") to end the exercise. The program will compute your score and display it on the Android device.
- Start a task again. If it is the same task as before, the demonstration will be skipped automatically. I suggest you calibrate Myo before starting a task.
- You can always say "home" (or click "Home/Reset") to end the current task.

Note
===
- You need to calibrate the Myo armbands about every 2 minutes. Just say the word "calibration" or click the button.
- Sometimes the speech recognition program fails to launch (10~20% chance). Just close the interface and start over again. It only recognizes words in the vocabulary.
- If Myo fails to launch, usually it is because the power is low. Charge them whenever you finish exercises.
- If the Android program does not respond, restart it. Make sure the IP is correct.
