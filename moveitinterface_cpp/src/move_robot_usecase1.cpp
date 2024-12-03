#include "rclcpp/rclcpp.hpp"
#include "moveitinterface_cpp/moverobotclass.hpp"
#include "robot_interface/srv/gripper_service.hpp"
#include <memory>
#include <chrono>
#include <thread>
class MoveitDummyNode : public rclcpp::Node{

    public:
        MoveitDummyNode(rclcpp::NodeOptions& node_opt):Node("dummy_move_group",node_opt){
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
        }
};


class NormalNode : public rclcpp::Node{

    public:
        NormalNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot,
        std::vector<double> robot_config_place1,
        std::vector<double> robot_config_place2):Node("ur5e_moveit_commander"), robot{robot} {
            RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
            robot_config_place1_ = robot_config_place1;
            robot_config_place2_ = robot_config_place2;
            position_from_camera = this->create_subscription<geometry_msgs::msg::Pose> (
            "objectpose/fromcamera", 1,std::bind(&NormalNode::callback_position_from_camera,this,std::placeholders::_1));//  
            client_gripper = this->create_client<robot_interface::srv::GripperService>("/gripper_service");  
            request_gripper = std::make_shared<robot_interface::srv::GripperService::Request>();
            while (!client_gripper->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
            }

        }

    void send_grip_request(int position, int speed,int force){
    
        request_gripper->position = position;
        request_gripper->speed    = speed;
        request_gripper->force    = force;
        client_gripper->async_send_request(request_gripper,std::bind(&NormalNode::callback_grip_request,this,std::placeholders::_1));
        
    }

    
    private:
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> position_from_camera;
    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> robot;

    std::shared_ptr<rclcpp::Client<robot_interface::srv::GripperService>> client_gripper;
    std::shared_ptr<robot_interface::srv::GripperService::Request> request_gripper;

    std::vector<double> robot_config_place1_;
    std::vector<double> robot_config_place2_;

    void callback_position_from_camera(const std::shared_ptr<geometry_msgs::msg::Pose> msg_pose){
        RCLCPP_INFO(this->get_logger(), "I heard:");
        std::cout<<"end effector: "<< msg_pose->position.x<<std::endl;
        geometry_msgs::msg::PoseStamped current_pose = robot->getCurrentPose();
        geometry_msgs::msg::Pose desiredpose = *msg_pose;
        desiredpose.orientation = current_pose.pose.orientation;
        geometry_msgs::msg::Pose midpose = desiredpose;
        midpose.position.z += 0.08;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(midpose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        robot->move_group.setPlanningTime(15);
        double jump_threshold = 5.0;
        double eef_step = 0.05;
        robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        robot->move_group.execute(trajectory);
        RCLCPP_INFO(this->get_logger(), "execution check1:");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        waypoints.pop_back();
        desiredpose.position.z = 0.211;
        waypoints.push_back(desiredpose);
        robot->move_group.setPlanningTime(10);
        jump_threshold = 5.0;
        eef_step = 0.05;
        robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        robot->move_group.execute(trajectory);
        RCLCPP_INFO(this->get_logger(), "execution check2:");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        this->send_grip_request(155,50,5); //close the gripper
        waypoints.pop_back(); 
        waypoints.push_back(midpose);// go up with the grapped object
        robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        robot->move_group.execute(trajectory);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        this->place_object();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        desiredpose = robot->getCurrentPose().pose;
        desiredpose.position.z += 0.08;
        waypoints.pop_back();
        waypoints.push_back(desiredpose);
        robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        robot->move_group.execute(trajectory);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::vector<double> robot_config_home_position {1.77919, -1.5818, 2.03653, 4.2764, -1.52085, 3.41678};
        robot->goToJointGoal(robot_config_home_position,"Rad");


    }


    
    void callback_grip_request(std::shared_future<std::shared_ptr<robot_interface::srv::GripperService::Response>> grip_response){
        auto response = grip_response.get();
        RCLCPP_INFO(this->get_logger(), "Received response from gripper: = %s", response->response.c_str());

    }

    void place_object(){
        robot->goToJointGoal(this->robot_config_place1_,"Rad");
        this->send_grip_request(20,50,5); // open the gripper

    }
};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<MoveitDummyNode> node = std::make_shared<MoveitDummyNode>(node_options);
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    
    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> ur5e = std::make_shared<moveit_interface_cpp::MoveRobotClass>("ur_manipulator", node);
    

    std::vector<double> robot_config_place1{1.24928, -1.24188, 2.02969, 4.01164, -1.54162, 2.80088};
    std::vector<double> robot_config_place2{1.29864, -1.11628, 1.85366, 4.01185, -1.54168, 2.80086};

    std::shared_ptr<NormalNode> node2 = std::make_shared<NormalNode>(ur5e,robot_config_place1,robot_config_place2);



    std::cout<<"Main thread is blocked "<<std::endl;
    node2->send_grip_request(20,50,10);
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
    
}



