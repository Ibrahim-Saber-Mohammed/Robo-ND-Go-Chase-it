#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>

ros::ServiceClient client;
bool isWhiteBallFound{false};


void requestRobotStop(void)
{
    ROS_INFO("Requesting the Robot to Stop");
    ball_chaser::DriveToTarget Robot_msg;
    Robot_msg.linear_x = 0.0;
    Robot_msg.angular_z = 0.0;
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
    Robot_msg.linear_x = lin_x;
    Robot_msg.angular_z = ang_z;
    if(!client.call(Robot_msg))
    {
        ROS_ERROR("Failed to call service ball_chaser/command_robot");
    }
}


// This callback function continuously executes and reads the image data
void process_image_callback(sensor_msgs::Image& image)
{
        // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    constexpr std::uint8_t WHITE_PIXLE{255};
    decltype(image.height * image.step) white_ball_position{0};
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
            if(image.data[i] == WHITE_PIXLE && image.data[i + 1] == WHITE_PIXLE && image.data[i + 2] == WHITE_PIXLE)
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
            int horizontal_position = (white_ball_position / 3) % image.step;
            if(horizontal_position < image.width / 3)
            {
                drive_robot(0.0, 0.5); // turn left
            }
            else if((horizontal_position > (image.width / 3 )) && ( horizontal_position < ( 2 * image.width / 3 )))
            {
                drive_robot(0.5, 0.0); // move forward
            }
            else if(horizontal_position > (2 * image.width / 3))
            {
                drive_robot(0.0, -0.5); // turn right
            }
            else{}
        }
    }
}
inr main(int argc, char**argv)
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
