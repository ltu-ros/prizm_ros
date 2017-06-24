/*
 * L2Bot ROS Motor Controller
 * Reads input from "L2Bot/mc"
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <AFMotor.h>

AF_DCMotor motor1(4);
AF_DCMotor motor2(2);

ros::NodeHandle  nh;

uint16_t v = 0;
uint8_t current_motor = 0;
uint8_t motor_speed = 0;

const bool FWD = 1;
const bool BKWD = 0;

uint8_t fst(uint16_t u) {
  return u >> 8;
}

uint8_t snd(uint16_t u) {
  return u & 0xFF;
}

uint8_t spd(uint8_t u) {
  return u << 1;
}

bool dir(uint8_t u) {
  return u >> 7;
}



void messageCb( const std_msgs::UInt16& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led

  v = toggle_msg.data;

  // Motor A
  current_motor = fst(v);
  motor_speed = spd(current_motor);

  if (motor_speed == 0)
  {
    // If the speed is 0, stop the motor
    motor1.run(RELEASE);
  }
  else 
  {
    // Set the speed of the motor
    motor1.setSpeed(motor_speed);
    if (dir(current_motor) == FWD)
    {
      motor1.run(FORWARD);
    }
    else
    {
      motor1.run(BACKWARD);
    }
  }


  // Motor B
  current_motor = snd(v);
  motor_speed = spd(current_motor);

  if (motor_speed == 0)
  {
    // If the speed is 0, stop the motor
    motor2.run(RELEASE);
  }
  else 
  {
    // Set the speed of the motor
    motor2.setSpeed(motor_speed);
    if (dir(current_motor) == FWD)
    {
      motor2.run(FORWARD);
    }
    else
    {
      motor2.run(BACKWARD);
    }
  }
}

ros::Subscriber<std_msgs::UInt16> sub("L2Bot/mc", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  //motor1.setSpeed(200);
  //motor1.run(FORWARD);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

