// ROS
#include <ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt32.h>

// Tetrix Prizm
#include <PRIZM.h>
PRIZM prizm;

// Direction
#define FWD 1
#define BKWD 0

#define IR_PIN 15

// Callback function for the message
void motorCB( const geometry_msgs::Point& point)
{
    prizm.setMotorPowers((int)point.x,  (int)point.y);
}

void redLEDCB( const std_msgs::Bool& toggle_msg)
{
    prizm.setRedLED(toggle_msg.data ? HIGH : LOW);
}

void greenLEDCB( const std_msgs::Bool& toggle_msg)
{
    prizm.setGreenLED(toggle_msg.data ? HIGH : LOW);
}

std_msgs::UInt32 ir_msg;

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Point> mc_sub("motor_cmd", &motorCB);
ros::Subscriber<std_msgs::Bool>       red_led_sub("red_led",   &redLEDCB);
ros::Subscriber<std_msgs::Bool>       green_led_sub("green_led", &greenLEDCB);
ros::Publisher      ir_pub("ir_dist", &ir_msg);

void setup()
{
    nh.initNode();
    nh.subscribe(mc_sub);
    nh.subscribe(red_led_sub);
    nh.subscribe(green_led_sub);
    nh.advertise(ir_pub);


    prizm.PrizmBegin();
    prizm.setRedLED(LOW);
    prizm.setGreenLED(LOW);
    prizm.setMotorInvert(2, 1); // Invert right motor

    pinMode(IR_PIN, INPUT);
}


void loop()
{
    ir_msg.data = analogRead(IR_PIN);
    ir_pub.publish(&ir_msg);

    nh.spinOnce();
    delay(1);
}
