// ROS
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

// Tetrix Prizm
#include <PRIZM.h>
PRIZM prizm;

// Direction
#define FWD 1
#define BKWD 0

// Used for decoding the 16bit message into motor instructions
// Get the first byte
uint8_t fst(uint16_t u) { return u >> 8; }
// Get the second byte
uint8_t snd(uint16_t u) { return u & 0xFF; }

// The direction is the last bit
// If 0, speed will be negative
int dir(uint8_t u) 
{
  return (u >> 7) == BKWD ? -1 : 1;
}

// The speed is the first 7 bits scaled from 255 to 100
// Direction (dir()) will be either negative or positive
int spd(uint8_t u)
{
  return dir(u) * ( ((uint8_t)(u << 1)) * 100 / 255); 
}

// Callback function for the message
void motorCB( const std_msgs::UInt16& twist_msg)
{
    uint16_t data   = twist_msg.data;
    uint8_t motor_a = snd(data);
    uint8_t motor_b = fst(data);

    prizm.setMotorPowers(spd(motor_a),  spd(motor_b));
}

void redLEDCB( const std_msgs::Bool& toggle_msg)
{
    prizm.setRedLED(toggle_msg.data ? HIGH : LOW);
}

void greenLEDCB( const std_msgs::Bool& toggle_msg)
{
    prizm.setGreenLED(toggle_msg.data ? HIGH : LOW);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> twist_sub("motor_cmd", &motorCB);
ros::Subscriber<std_msgs::Bool> red_led_sub("red_led",   &redLEDCB);
ros::Subscriber<std_msgs::Bool> green_led_sub("green_led", &greenLEDCB);

void setup()
{
    nh.initNode();
    nh.subscribe(twist_sub);
    nh.subscribe(red_led_sub);
    nh.subscribe(green_led_sub);

    prizm.PrizmBegin();
    prizm.setRedLED(LOW);
    prizm.setGreenLED(LOW);
    prizm.setMotorInvert(2, 1); // Invert right motor

}

void loop()
{
    nh.spinOnce();
    delay(1);
}
