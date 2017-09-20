* this fork contains ros package for an arduino_base_controller node controlling a differential robot and its corresponding arduino code


- the Arduino folder contains the arduino code and the libraries you will need for it to compile

- the ros_arduino contains the ros packages, just create your work space :http://wiki.ros.org/catkin/Tutorials/create_a_workspace

cd ~/catkin_ws/src
cp -r <ros_arduino/ros_arduino> . #copy "ros_arduino" directory over to the "src" folder
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash # you could add this line to .bashrc if your planning on using this package frequently 



* Now you should be able to connect your arduino, compile and upload the code, and get your arduino controller running :
roslaunch ros_arduino_base base_bringup.launch 

optionally you can also test it out manualy using:
rosrun turtlesim turtle_teleop_key /turtle1/cmd_vel:=/cmd_vel



