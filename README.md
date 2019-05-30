# Milk-tea-robot
The task of the milk tea robot is defined to first grasp a bottle of milk tea, shake it well by tilting the bottle and deliver the milk tea to a customer.

A simulated UR5 robot was used to implement the task. It has 6 revolute joints. I created a ROS package that describes the UR5 robot kinematic chain and simulates the robot in Rviz. I created another ROS package that implemented the robot forward and inverse kinematics (both analytical and numerical). The analytical solutions are more stable and not dependent on the initial guess, so they are used in this project. The analytical solutions are adopted from [1,2].


The inverse kinematics is used to calculate the configurations of the key end-effector poses: the pose to grasp the bottle, the pose to shake the bottle and the pose to deliver to the customer. Between the key robot configurations, the interpolation was calculated to generate the robot movement. If there are multiple solutions for a robot end-effector pose, the one which is the closest to the current configuration is selected. The distance between the robot configurations is described using vector L2 norm .


For each generated robot configuration, the pose of the milk tea bottle is computed using the robot forward kinematics and is visualized using Rviz.


Command to run the program:
$ cd /path/to/IR_library_ros/IR_catkin_ws
$ catkin_make 
$ source devel/setup.bash 
$ roslaunch ur_description ur5_rviz.launch 
