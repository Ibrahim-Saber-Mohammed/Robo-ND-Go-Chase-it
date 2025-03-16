#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdint>
ros::ServiceClient client;


void requestRobotStop(void)
{
    ROS_INFO("Requesting the Robot to Stop");
    ball_chaser::DriveToTarget Robot_msg;
    Robot_msg.request.linear_x = 0.0;
    Robot_msg.request.angular_z = 0.0;
    if(!client.call(Robot_msg))
    {
        ROS_ERROR("Failed to call service ball_chaser/command_robot");
    }
}
// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO("Driving the Robot ");
    ball_chaser::DriveToTarget Robot_msg;
    Robot_msg.request.linear_x = lin_x;
    Robot_msg.request.angular_z = ang_z;
    if(!client.call(Robot_msg))
    {
        ROS_ERROR("Failed to call service ball_chaser/command_robot");
    }
}


// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image& image)
{
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    static constexpr std::uint8_t WHITE_PIXLE{255};
    bool isWhiteBallFound{false};
    int white_ball_position{-1};
    if(image.encoding != sensor_msgs::image_encodings::RGB8  )
    {
        ROS_ERROR("Required RGB image encoding to process");
    }
    else{
        for(decltype(image.height * image.step) iter{0}; iter < image.height * image.step; iter += 3)
        {
            // red channel   = image->data[i];
            // green channel = image->data[i + 1];
            // blue channel  = image->data[i + 2];
            if(image.data[iter] == WHITE_PIXLE && image.data[iter + 1] == WHITE_PIXLE && image.data[iter + 2] == WHITE_PIXLE)
            {
                ROS_INFO("White Ball Found");
                isWhiteBallFound = true;
                white_ball_position = iter;
                break;
            }
        }
        if(!isWhiteBallFound)
        {
            ROS_INFO("White Ball Not Found");
            requestRobotStop();
        }
        else{
            // Determine the position of the white ball
            // If it's in the left, mid, or right side of the image
            // Call the drive_bot function and pass velocities to it
            // Request a stop when there's no white ball seen by the camera
            int horizontal_position = (white_ball_position) % image.step;
            float left_threshold = image.step * 0.4;
	    float right_threshold = image.step * 0.65 ;
            if(horizontal_position < left_threshold)
            {
                drive_robot(0.5, 0.5); // turn left
            }
            else if(horizontal_position > right_threshold )
            {
                drive_robot(0.5, -0.50); // turn right
            }
            else 
            {
                drive_robot(0.5, 0.0); // move forward
            }
        }
    }
}
int main(int argc, char**argv)
{
    // Create Node 
    ros::init(argc, argv, "process_image");
    // Create NodeHandle
    ros::NodeHandle n;
    // Subscribe to /camera/rgb/image_raw topic
    ros::Subscriber image_data_Sub = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    // Create service client
    client = n.serviceClient<ball_chaser::DriveToTarget>("ball_chaser/command_robot");
    
    // Handle ROS communication events
    ros::spin();
    return 0;
}
