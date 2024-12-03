#include "rclcpp/rclcpp.hpp"
#include "moveitinterface_cpp/moverobotclass.hpp"

class PersonalNode : public rclcpp::Node{

    public:
        PersonalNode(rclcpp::NodeOptions& node_opt):Node("move_robot_node",node_opt){
            RCLCPP_INFO(this->get_logger(),"personal node has been initialized");
            

        }
};


int main(int argc, char **argv){

    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    std::shared_ptr<PersonalNode> node = std::make_shared<PersonalNode>(node_options);
    std::shared_ptr<PersonalNode> node2 = std::make_shared<PersonalNode>(node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    


    std::shared_ptr<moveit_interface_cpp::MoveRobotClass> ur5e = std::make_shared<moveit_interface_cpp::MoveRobotClass>("ur_manipulator", node);

    auto topic_callback =
      [node2,ur5e](const std::shared_ptr<geometry_msgs::msg::Pose> msg_pose) -> void {
        RCLCPP_INFO(node2->get_logger(), "I heard:");
        std::cout<<"end effector: "<< msg_pose->position.x<<std::endl;
        geometry_msgs::msg::PoseStamped current_pose = ur5e->getCurrentPose();
        geometry_msgs::msg::Pose desiredpose = current_pose.pose;
        std::cout<<"current Z value: "<<current_pose.pose.position.z<<std::endl;
        desiredpose.position.z = current_pose.pose.position.z + 0.05;
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(desiredpose);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        ur5e->move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ur5e->move_group.execute(trajectory);
        ur5e->getPlanningFrame();
      };

    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Pose>> position_from_camera;
    position_from_camera = node2->create_subscription<geometry_msgs::msg::Pose>(
            "objectpose/fromcamera", 1,topic_callback);

    std::cout<<"end effector:main "<<std::endl;
    rclcpp::executors::SingleThreadedExecutor executor2;
    executor2.add_node(node2);
    executor2.spin();
    std::cout<<"end effector:main 2"<<std::endl;

    return 0;
}