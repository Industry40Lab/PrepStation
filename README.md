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

## Packages Description:

The package "moveitinterface_cpp" is a wrapper of MoveIt C++ API where the cobot trajectory is planned.

The package "robot_interface" contains the service for open/close of ur5e gripper. 

The package "panda_info" contains the messages being communincatd between stations. 

The package "panda_workstation" is the package installed on the control station publishing the information of defective PCBs to the reworking station.

The package "rs_location_retrival" receives the RGB-D images from the RealSense camera D435i, localizes the desolering tools and publishes the corresponding coordinates.

## Packages dependencies:

moveit;
universal robot
realsense camera

ros1/2 bridge