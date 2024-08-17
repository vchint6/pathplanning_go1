#include <ros/ros.h>
#include <open_manipulator_msgs/SetDrawingTrajectory.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    ros::init(argc, argv, "drawing_trajectory_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("goal_drawing_trajectory");

    std::string name;
    std::vector<double> arg(3);
    double path_time;

    std::cout << "Enter drawing trajectory type (line, circle, rhombus, heart): ";
    std::cin >> name;
    std::cout << "Enter Transpose X: ";
    std::cin >> arg[0];
    std::cout << "Enter Transpose Y: ";
    std::cin >> arg[1];
    std::cout << "Enter Transpose Z: ";
    std::cin >> arg[2];
    std::cout << "Enter path time: ";
    std::cin >> path_time;

    open_manipulator_msgs::SetDrawingTrajectory srv;
    srv.request.end_effector_name = "gripper";
    srv.request.drawing_trajectory_name = name;
    srv.request.path_time = path_time;
    srv.request.param = arg;

    if (client.call(srv)) {
        if (srv.response.is_planned) {
            ROS_INFO("Trajectory planned successfully.");
        } else {
            ROS_ERROR("Failed to plan trajectory.");
        }
    } else {
        ROS_ERROR("Failed to call service goal_drawing_trajectory");
    }

    return 0;
}

