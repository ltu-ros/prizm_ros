#include <ros/ros.h>
#include <std_msgs/Bool.h>

bool green_led;
bool red_led;

void printState()
{
    for (int i = 0; i < 60; i++) ROS_INFO_STREAM("");
    ROS_INFO_STREAM("  Red LED: " << (red_led ?   "ON" : "OFF"));
    ROS_INFO_STREAM("Green LED: " << (green_led ? "ON" : "OFF"));
}

void greenCB(const std_msgs::Bool& msg)
{
    green_led = msg.data;
    printState();
}

void redCB(const std_msgs::Bool& msg)
{
    red_led = msg.data;
    printState();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "prizm_led_sim");
    ros::NodeHandle nh;

    green_led = false;
    red_led   = false;

    ros::Subscriber green_sub = nh.subscribe("/green_led", 10, greenCB);
    ros::Subscriber red_sub   = nh.subscribe("/red_led", 10, redCB);

    printState();

    ros::spin();
}
