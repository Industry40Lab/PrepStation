#include "rclcpp/rclcpp.hpp"
#include "moveitinterface_cpp/moverobotclass.hpp"

class MoveitDummyNode : public rclcpp::Node{

    public:
        MoveitDummyNode(rclcpp::NodeOptions& node_opt):Node("moveit_dummy_mz_node",node_opt){
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
        }
};


class NormalNode : public rclcpp::Node{

    public:
        NormalNode(std::shared_ptr<moveit_interface_cpp::MoveRobotClass>
        robot):Node("subscriber_node"),robot{robot} {
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
            position_from_camera = this->create_subscription<geometry_msgs::msg::Pose> (
            "objectpose/fromcamera", 1,std::bind(&NormalNode::callback_position_from_camera,this,std::placeholders::_1));//    
        }

    
    private:
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> position_from_camera;
    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> robot;
    void callback_position_from_camera(const std::shared_ptr<geometry_msgs::msg::Pose> msg_pose){
        RCLCPP_INFO(this->get_logger(), "I heard:");
        std::cout<<"end effector: "<< msg_pose->position.x<<std::endl;
        geometry_msgs::msg::PoseStamped current_pose = robot->getCurrentPose();
        geometry_msgs::msg::Pose desiredpose = current_pose.pose;
        std::cout<<"current Z value: "<<current_pose.pose.position.z<<std::endl;
        desiredpose.position.z = current_pose.pose.position.z + 0.05;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(desiredpose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        robot->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        // RCLCPP_INFO(node, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
        robot->move_group.execute(trajectory);
        robot->getPlanningFrame();
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
    std::shared_ptr<NormalNode> node2 = std::make_shared<NormalNode>(ur5e);

  
    std::cout<<"Main thread is blocked "<<std::endl;
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
    
}

