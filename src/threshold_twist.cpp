// ROS and messages
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <dynamic_reconfigure/server.h>
#include <prizm_ros/TwistConfig.h>

/* Prizm:
 * Interface node to convert from Twist to Motor Controller CMD
 *
 * Subscribe: Twist on "~/twist_cmd"
 * Publish:   UInt16 on "~/motor_cmd"
 */

#define TWIST_INPUT "twist_cmd"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class BoundTwist {
public:
    BoundTwist();

    // Convert a twist message into a MC instruction
    void driveCB(const geometry_msgs::Twist& twist);

    // Encode motor controller instructions into a single
    //   16 bit unsigned int
    uint16_t make_vec(
        uint8_t Aspeed, bool Adir,
        uint8_t Bspeed, bool Bdir);

    // Dynamic reconfigure callback
    void configCallback(prizm_ros::TwistConfig& config_, uint32_t level);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber drive_sub_;
    double boundVal(double val, double minV, double maxV);

    float turn_multip_;
    prizm_ros::TwistConfig config_state;
    dynamic_reconfigure::Server<prizm_ros::TwistConfig> server;
};


BoundTwist::BoundTwist() : nh_{"~"}
{
    // Publish a motor controller instruction in the prizm namespace
    pub_ = nh_.advertise<geometry_msgs::Point>("/prizm/motor_cmd", 1);

    server.setCallback(boost::bind(&BoundTwist::configCallback,this, _1, _2));
    server.getConfigDefault(config_state);
    drive_sub_ = nh_.subscribe(TWIST_INPUT, 1, &BoundTwist::driveCB, this);

    // Load params
    turn_multip_ = 3.0f; //Default value
    if (!nh_.getParam("/prizm/turn_multip", turn_multip_)) {
        ROS_ERROR_STREAM("prizm: could not load param '~/turn_multip'");
        ROS_ERROR_STREAM("prizm: using default value: " << turn_multip_);
    }    

}

void BoundTwist::configCallback(prizm_ros::TwistConfig& config_, uint32_t level)
{
    config_state = config_;
}

double BoundTwist::boundVal(double val, double minV, double maxV){
    if(minV>maxV){
        ROS_ERROR_STREAM("Range is inverted... min value should never be greater than max value. Returning 0");
        return 0.0;        
    }
    
    double z=val;
    if(fabs(z)>0.05){
        if(fabs(z)<minV){
            z=sgn(z)*minV;
        }else if(fabs(z)>maxV){
            z=sgn(z)*maxV;
        }
    }
    return z;
}

void BoundTwist::driveCB(const geometry_msgs::Twist& twist)
{
    double x = boundVal(twist.linear.x, config_state.min_linear, config_state.max_linear);
    double z = boundVal(twist.angular.z, config_state.min_angular, config_state.max_angular);

    // Apply minimum values
    static constexpr double scale = 100.0;

    double Aspeed = -x;
    double Bspeed = -x;
    bool Adir = true;
    bool Bdir = true;

    Aspeed -= z*turn_multip_;
    Bspeed += z*turn_multip_;

    Aspeed *= scale / 3.0f;
    Bspeed *= scale / 3.0f;

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
     * greater than `scale` can be reached. To allow for a
     * reasonable combination of both commands, they are carried out
     * in the same ratio, but scaled down to within the `scale` limit.
     */
    int max_speed = Aspeed > Bspeed ? Aspeed : Bspeed;
    if(max_speed > scale)
    {
        float mux = scale / max_speed;
        Aspeed *= mux;
        Bspeed *= mux;
    }

    // Output motor speeds
    ROS_INFO_STREAM("A/B: " << Aspeed << " " << Bspeed);

    // Set direction
    int motorA = (Adir ? 1 : -1) * std::floor(Aspeed);
    int motorB = (Bdir ? 1 : -1) * std::floor(Bspeed);

    geometry_msgs::Point point;
    point.x = motorB;
    point.y = motorA;

    pub_.publish(point);
}

uint16_t BoundTwist::make_vec(
    uint8_t Aspeed, bool Adir,
    uint8_t Bspeed, bool Bdir)
{
    uint8_t a = Aspeed >> 1 | (Adir << 7);
    uint8_t b = Bspeed >> 1 | (Bdir << 7);
    return (a << 8) + b;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist_controller");
    BoundTwist prizm;

    // Handle callbacks
    ros::spin();

    return 0;
}
