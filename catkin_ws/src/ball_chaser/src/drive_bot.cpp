/**
 * This server node provides a ball_chaser/command_robot service to drive the robot around 
 * By setting its linear x and angular z velocities. 
 * The service server publishes messages containing the velocities for the wheel joints.
 */

#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher ;
std::vector<double>clamp_at_boundaries(float linear_x, float angular_z)
{
    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName();
    float clamped_vel_linear = linear_x;
    float clamped_vel_angular = angular_z;

    float min_linear_x, max_linear_x, min_angular_z, max_angular_z;
    nh.getParam(node_name + "/min_linear_x", min_linear_x);
    nh.getParam(node_name + "/max_linear_x", max_linear_x);
    nh.getParam(node_name + "/min_angular_z", min_angular_z);
    nh.getParam(node_name + "/max_angular_z", max_angular_z);

    if(clamped_vel_linear < min_linear_x || clamped_vel_linear > max_linear_x)
    {
        clamped_vel_linear = std::min( max_linear_x, std::max(clamped_vel_linear, min_linear_x));
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_linear_x, max_linear_x, clamped_vel_linear);
    }
    if(clamped_vel_angular < min_angular_z || clamped_vel_angular > max_angular_z)
    {
        clamped_vel_angular = std::min( max_angular_z, std::max(clamped_vel_angular, min_angular_z));
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_angular_z, max_angular_z, clamped_vel_angular);
    }
    std::vector<double> clamped_data{clamped_vel_linear, clamped_vel_angular};
    return clamped_data;
}
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request&req, ball_chaser::DriveToTarget::Response&res)
{
    ROS_INFO("ball_chaser/command_robot service requested with linear_x: %1.2f  angular_z: %1.2f", static_cast<float>(req.linear_x) , static_cast<float>(req.angular_z));
    // Create a motor_command object of type geometry_msgs::Twist
    std::vector<double> clampedVelocities = clamp_at_boundaries(static_cast<float>(req.linear_x), static_cast<float>(req.angular_z));
    
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = clampedVelocities[0];
    motor_command.angular.z = clampedVelocities[1];
    
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);
    ros::Duration(3).sleep();
    
    // Return a response message
    res.msg_feedback = "linear velocity set to "+ std::to_string(clampedVelocities[0]) + ",  angular velocity set to " + std::to_string(clampedVelocities[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}
int main (int argc, char**argv)
{
    // Initialize ROS Node 
    ros::init(argc, argv, "drive_bot");
    // Create a ROS NodeHandle object
    ros::NodeHandle nh;
    // Create a service Server
    ros::ServiceServer command_robot_server = nh.advertiseService("ball_chaser/command_robot", handle_drive_request);
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();
}

