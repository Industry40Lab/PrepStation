#include "rclcpp/rclcpp.hpp"
#include "moveitinterface_cpp/moverobotclass.hpp"
#include "robot_interface/srv/gripper_service.hpp"
#include <memory>
#include <chrono>
#include <thread>
class MoveitDummyNode : public rclcpp::Node{

    public:
        MoveitDummyNode(rclcpp::NodeOptions& node_opt):Node("moveit_dummy_mz_node",node_opt){
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
        }
};


class NormalNode : public rclcpp::Node{

    public:
        NormalNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>  robot,
        std::vector<double> robot_config_pcb_on_agv,
        std::vector<double> robot_config_pcb_on_agv_move_to_up,
        std::vector<double> robot_config_pcb_on_agv_move_to_place,
        std::vector<double> robot_config_home_to_midway_position,
        std::vector<double> robot_config_home_position 
        ):Node("subscriber_node"), robot{robot} {
            RCLCPP_INFO(this->get_logger(),"Normal node has been initialized");
            robot_config_home_to_midway_position_ = robot_config_home_to_midway_position;
            robot_config_pcb_on_agv_ = robot_config_pcb_on_agv;
            robot_config_pcb_on_agv_move_to_up_ = robot_config_pcb_on_agv_move_to_up;
            robot_config_pcb_on_agv_move_to_place_ = robot_config_pcb_on_agv_move_to_place;
            robot_config_home_position_ = robot_config_home_position;
            
        
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

        void pick_place_pcb(){


            RCLCPP_INFO(this->get_logger(), "Robot is going from home_to_midway_position");
            robot->goToJointGoal(robot_config_home_to_midway_position_,"Rad");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "Robot is going from midway_position to on top of agv");
            robot->goToJointGoal(robot_config_pcb_on_agv_move_to_up_,"Rad");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            RCLCPP_INFO(this->get_logger(), "Robot is going from on top of agv to the agv");
            robot->goToJointGoal(robot_config_pcb_on_agv_,"Rad");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "closing the gripper");
            this->send_grip_request(230,50,5); // close the gripper
            std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "Robot is going from the agv to  top of agv ");

            robot->goToJointGoal(robot_config_pcb_on_agv_move_to_up_,"Rad");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            RCLCPP_INFO(this->get_logger(), "Robot is going   top of agv to placing config ");

            robot->goToJointGoal(robot_config_pcb_on_agv_move_to_place_,"Rad");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "Robot is opening the gripper");

            this->send_grip_request(20,50,5); // open the gripper
            std::this_thread::sleep_for(std::chrono::seconds(1));

            RCLCPP_INFO(this->get_logger(), "Robot is going   top of board ");
            geometry_msgs::msg::Pose current_pose = robot->getCurrentPose().pose;
            double jump_threshold = 5.0;
            double eef_step = 0.05;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            moveit_msgs::msg::RobotTrajectory trajectory;
            current_pose.position.z += 0.05;
            waypoints.push_back(current_pose);
            robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            robot->move_group.execute(trajectory);
            std::this_thread::sleep_for(std::chrono::seconds(1));
             RCLCPP_INFO(this->get_logger(), "Robot is going  home ");
            robot->goToJointGoal(robot_config_home_position_,"Rad");
            RCLCPP_INFO(this->get_logger(), "Robot has been back to its home config");

    }



    
    private:
    
    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> robot;

    std::shared_ptr<rclcpp::Client<robot_interface::srv::GripperService>> client_gripper;
    std::shared_ptr<robot_interface::srv::GripperService::Request> request_gripper;

    std::vector<double> robot_config_home_position_;
    std::vector<double> robot_config_home_to_midway_position_;
    std::vector<double> robot_config_pcb_on_agv_;
    std::vector<double> robot_config_pcb_on_agv_move_to_up_;
    std::vector<double> robot_config_pcb_on_agv_move_to_place_;
            

   

    void callback_grip_request(std::shared_future<std::shared_ptr<robot_interface::srv::GripperService::Response>> grip_response){
        auto response = grip_response.get();
        RCLCPP_INFO(this->get_logger(), "Received response from gripper: = %s", response->response.c_str());

    }

    // void place_object(){
    //     robot->goToJointGoal(this->robot_config_place1_,"Rad");
    //     this->send_grip_request(20,50,5); // open the gripper
    // }
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
    std::vector<double> robot_config_home_position {1.77919, -1.5818, 2.03653, 4.2764, -1.52085, 3.41678};
    ur5e->goToJointGoal(robot_config_home_position,"Rad");

    std::vector<double> robot_config_pcb_on_agv{0.0876822, -0.423569, 1.41204, 3.76274, -1.55298, 3.18319 };
    std::vector<double> robot_config_pcb_on_agv_move_to_up {0.0840025, -1.30054, 1.41209, 4.575, -1.55296, 3.18322 };
    std::vector<double> robot_config_pcb_on_agv_move_to_place {0.804673, -1.55329, 2.18319, 4.18462, -1.6228, 3.97436 };
    std::vector<double> robot_config_home_to_midway_position {0.817575, -1.62226, 1.81344, 4.5373, -1.52095, 3.41685};
    
    std::shared_ptr<NormalNode> node2 = std::make_shared<NormalNode>(ur5e,
    robot_config_pcb_on_agv,
    robot_config_pcb_on_agv_move_to_up,
    robot_config_pcb_on_agv_move_to_place,
    robot_config_home_to_midway_position,
    robot_config_home_position
    );



    std::cout<<"Main thread is blocked "<<std::endl;
    
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    node2->send_grip_request(20,50,10);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    node2->pick_place_pcb();
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
    
}


// std::vector<double> robot_config_pcb_on_agv{0.0876822, -0.423569, 1.41204, 3.76274, -1.55298, 3.18319 };
// std::vector<double> robot_config_pcb_on_agv_move_to_up {0.0840025, -1.30054, 1.41209, 4.575, -1.55296, 3.18322 };
// std::vector<double> robot_config_pcb_on_agv_move_to_place {0.804673, -1.55329, 2.18319, 4.18462, -1.6228, 3.97436 };
// std::vector<double> robot_config_home_position {1.77919, -1.5818, 2.03653, 4.2764, -1.52085, 3.41678};
// std::vector<double> robot_config_home_to_midway_position {0.817575, -1.62226, 1.81344, 4.5373 ,-1.52095, 3.41685};

