// ROS and messages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>

/* L2Bot:
 * Interface node to convert from rb_drive to gem data
 *
 * Subscribe: Twist on "/rb_drive/rb_drive/twist_cmd"
 * Publish:   UInt16 on "~/motor_cmd"
 */

#define TWIST_INPUT "/rb_drive/rb_drive/twist_cmd"

class L2Bot {
public:
    L2Bot();

    // Convert a twist message into a MC instruction
    void driveCB(const geometry_msgs::Twist& twist);

    // Encode motor controller instructions into a single
    //   16 bit unsigned int
    uint16_t make_vec(
        uint8_t Aspeed, bool Adir,
        uint8_t Bspeed, bool Bdir);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber drive_sub_;

    float turn_multip_;
};


L2Bot::L2Bot() : nh_{"~"}
{
    // Publish a motor controller instruction in the l2bot namespace
    pub_ = nh_.advertise<std_msgs::UInt16>("motor_cmd", 1);

    drive_sub_ = nh_.subscribe(TWIST_INPUT, 1, &L2Bot::driveCB, this);

    // Load params
    turn_multip_ = 5.0f; //Default value
    if (!nh_.getParam("/l2bot/turn_multip", turn_multip_)) {
        ROS_INFO_STREAM("l2bot: could not load param '/l2bot/turn_multip'");
        ROS_INFO_STREAM("l2bot: using default value: " << turn_multip_);
        ROS_INFO_STREAM("l2bot: to load params use 'roslaunch l2bot l2bot.launch'");
    }
}

void L2Bot::driveCB(const geometry_msgs::Twist& twist)
{
    std_msgs::UInt16 t;

    if(std::abs(twist.linear.x) > 3 || std::abs(twist.angular.z) > 2)
    {
        t.data = make_vec(0, true, 0, true);
        pub_.publish(t);
        return;
    }

    float Aspeed = twist.linear.x;
    float Bspeed = twist.linear.x;
    bool Adir = true;
    bool Bdir = true;


    Aspeed -= twist.angular.z*turn_multip_;
    Bspeed += twist.angular.z*turn_multip_;

    Aspeed *= 255.0f/3.0f;
    Bspeed *= 255.0f/3.0f;

    if(Aspeed < 0)
    {
        Aspeed = -Aspeed;
        Adir = false;
    }

    if(Bspeed < 0)
    {
        Bspeed = -Bspeed;
        Bdir = false;
    }

    /*
     * When max rotation is added to max linear speed, values of
     * greater than 255 can be reached. To allow for a
     * reasonable combination of both commands, they are carried out
     * in the same ratio, but scaled down to within the 255 limit.
     */
    int max_speed = Aspeed > Bspeed ? Aspeed : Bspeed;
    if(max_speed > 255)
    {
        float mux = 255.0f / max_speed;
        Aspeed *= mux;
        Bspeed *= mux;
    }

    // Output motor speeds
    // ROS_ERROR_STREAM("A/B: " << Aspeed << " " << Bspeed);

    t.data = make_vec(Aspeed, Adir, Bspeed, Bdir);
    pub_.publish(t);
}

uint16_t L2Bot::make_vec(
    uint8_t Aspeed, bool Adir,
    uint8_t Bspeed, bool Bdir)
{
    uint8_t a = Aspeed >> 1 | (Adir << 7);
    uint8_t b = Bspeed >> 1 | (Bdir << 7);
    return (a << 8) + b;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "l2bot");
    L2Bot l2bot;

    // Handle callbacks
    ros::spin();

    return 0;
}
