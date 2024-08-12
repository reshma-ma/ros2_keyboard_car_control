This is a ROS2 package to control a robot using keyboard, which can take the user input from the keyboard and can control a car in a simulated environment.
Ubuntu version :18.04
ROS2 :Eloquent
Steps by step guide
1.Install Simulator
2.Install ROS2 eloquent from ROS2 documentation
3.create a work space
4.clone this package inside the workspace
5.Still in the root of your workspace,  build your new package: colcon build
6.source the setup file :source install/setup.bash
7.launch simulator 
8. run the nodes in different new terminal and soirce into the setup file.

ros2 run car_control keyboard_input_node (in one terminal)
ros2 run car_control velocity_publisher_node (in other terminal)

9.Simulate the car using the keyboard pressing keys w,a,d,x,s
