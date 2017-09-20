* this fork contains ros package for a base_link node controlling a differential robot and its corrisponding arduino code


- the Arduino folder contains the arduino code and the libraries you will need for it to compile

- the ros_arduino contains the ros packages, just create your work space :http://wiki.ros.org/catkin/Tutorials/create_a_workspace

cd ~/catkin_ws/src
cp -r <ros_arduino/ros_arduino> . #copy "ros_arduino" directory over to the "src" folder
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash # you could add this line to .bashrc if your planning on using this package frequently 



* Now you should be able to connect your arduino compile and upload the code and your base_link node using:
roslaunch ros_arduino_base base_bringup.launch 



