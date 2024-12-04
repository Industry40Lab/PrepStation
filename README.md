# PrepStation


This repositroy is being developed as one of usecases of  <a href="https://arise-middleware.eu/">ARISE</a>, co-funded by European Union, at the <a href="https://www.industry40lab.org/">INDUSTRY4.0</a> laboratory affiliated with Politecnico di Milano.

By the end of Dec 2024, the project will have completed its first year and the implementation progress has been reported. Initially, the usecase is briefly described. Followingly, the functions of each ros2 packages are explained. 

## Use Case Description:
A defective PCB is identified in the control station, then the control station transmits data of the PCB and the relevant non-functional components to the reworking station; in the reworking station, an UR5e cobot identifies appropriate tools
and prepares the setup of the workspace for the operator. In the meanwhile, an AGV transports the defective PCB from the control station to the reworking station. At the time of arrival, the UR5e cobot unloads the defective PCB. The workspace must be prepared before the arrival of the operator. 
The workflow of usecase is depicted below:
<p align="center">
  <img src="material/arise_usecase1.png" alt="Image 1" width="480"/></a>
</p>

The high level system architecture is shown:
<p align="center">
  <img src="material/arise_usecase1_system architecture.png" alt="Image 1" width="480"/></a>
</p>

 where the AGV operates on ROS1 Noetic and two cobots operate on ROS2. The ROS1/2 bridge is used to exchange data among devices.

## Packages in the repositroy:

The package "moveitinterface_cpp" is a wrapper of MoveIt C++ API where the cobot trajectory is planned.

The package "robot_interface" contains the service for open/close of ur5e gripper. 

The package "panda_info" contains the messages being communincatd between stations. 

The package "panda_workstation" is the package installed on the control station publishing the information of defective PCBs to the reworking station.

The package "rs_location_retrival" receives the RGB-D images from the RealSense camera D435i, localizes the desolering tools and publishes the corresponding coordinates.

## Prerequisites:
The robot trajectory has been planned using <a href="https://github.com/moveit/moveit2/tree/main">Moveit2</a>.

The driver of Universal Robot has been installed using official <a href="https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main"> Universal_Robots_ROS2_Driver</a> repositroy. 

The driver of Intel RealSense D435i camera has been installed using 
<a href="https://github.com/IntelRealSense/realsense-ros">librealsense</a> repositroy.

The driver of Robotiq Hande gripper is installed using 
<a href="https://github.com/patsyuk03/RobotiqHandeROS2Driver/tree/main">RobotiqHandeROS2Driver</a> repositroy.

The bridge communication between ROS1 and ROS2 is made using this
<a href="https://github.com/ros2/ros1_bridge"> repositroy</a>.

For the localization purpose, it is necessary to calibrate the camera's coordinates with respect to the robot's origin coordinate. The calibration is carried out using <a href="https://github.com/IFL-CAMP/easy_handeye">this repo</a> which is implemented in ROS1.
## Command lines executions

1. Run this command 
```bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 launch_rviz:=false
```
to establish the communication between the PC and the UR5e.

2. In a new terminal, run this command 
```bash
   ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:="ur5e" launch_rviz:=false
```
to launch moveit2 config for trajectory planning.

3. In a new terminal, run the command
```bash
  ros2 launch robotiq_hande_ros2_driver gripper_bringup.launch.py robot_ip:=192.168.0.100
```
to establish the communication with Hande gripper.

4. Run the following commands in the seperate terminals for the unloading of defective pcbs from AGV's tray
```bash
  ros2 launch moveitinterface_cpp agv_pick.launch.py ur_type:="ur5e"
```
and pick/place of the desolering tool on the table
```bash
  ros2 launch moveitinterface_cpp pick_component.launch.py ur_type:="ur5e"
```
5. In a new terminal, run 
```bash
  ros2 launch realsense2_camera rs_launch.py
```
to establish the communication with the D435i camera.

6. In a new terminal, run
```bash
  ros2 run rs_location_retrival localization 
```
for the localization of desolering tool. 