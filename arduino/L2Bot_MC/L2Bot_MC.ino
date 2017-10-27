#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>

// Pins
#define A_MOTOR 13
#define A_BRAKE 8
#define A_SPEED 11
#define B_MOTOR 12
#define B_BRAKE 9
#define B_SPEED 3

#define FWD 1
#define BKWD 0

// Global variables
uint16_t v = 0;
uint8_t current_motor = 0;
uint8_t motor_speed = 0;


// Used for decoding the 16bit message into motor instructions
// Get the first byte
uint8_t fst(uint16_t u) { return u >> 8; }
// Get the second byte
uint8_t snd(uint16_t u) { return u & 0xFF; }
// The speed is the first 7 bits
uint8_t spd(uint8_t u) { return u << 1; }
// The direction is the last bit
bool dir(uint8_t u) { return u >> 7; }


// Callback function for the message
void messageCb( const std_msgs::UInt16& toggle_msg)
{
    v = toggle_msg.data;

    // Motor A
    current_motor = fst(v);
    motor_speed = spd(current_motor);

    if (motor_speed == 0)
    {
        // If the speed is 0, stop the motor
        digitalWrite(A_BRAKE, HIGH);  //Engage the Brake for Channel A
    }
    else
    {
        // Release the brake
        digitalWrite(A_BRAKE, LOW);
      
        // Set the direction of the motor
        if (dir(current_motor) == FWD)
            digitalWrite(A_MOTOR, LOW);
        else
            digitalWrite(A_MOTOR, HIGH);

        // Set the speed of the motor
        analogWrite(A_SPEED, motor_speed);
    }


    // Motor B
    current_motor = snd(v);
    motor_speed = spd(current_motor);

    if (motor_speed == 0)
    {
        // If the speed is 0, stop the motor
        digitalWrite(B_BRAKE, HIGH);  //Engage the Brake for Channel A
    }
    else
    {
        // Release the brake
        digitalWrite(B_BRAKE, LOW);
        
        // Set the direction of the motor
        if (dir(current_motor) == FWD)
            digitalWrite(B_MOTOR, HIGH);
        else
            digitalWrite(B_MOTOR, LOW);

        // Set the speed of the motor
        analogWrite(B_SPEED, motor_speed);
    }
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("l2bot/motor_cmd", &messageCb);

void setup()
{

    //Setup Channel A
    pinMode(A_MOTOR, OUTPUT); //Initiates Motor Channel A pin
    pinMode(A_BRAKE, OUTPUT); //Initiates Brake Channel A pin

    //Setup Channel B
    pinMode(B_MOTOR, OUTPUT); //Initiates Motor Channel A pin
    pinMode(B_BRAKE, OUTPUT);  //Initiates Brake Channel A pin

    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}


