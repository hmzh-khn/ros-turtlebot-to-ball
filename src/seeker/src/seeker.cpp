/* Hamzah Khan
 * Harvey Mudd College 2018
 * hikhan@hmc.edu
 * Created: 3/3/2018
 * Updated: 3/6/2018
 * seeker.cpp is a service that, upon request, begins continuously
 * - using the onboard Kinect to scan the visible area,
 * - publishing the relative displacement between the turtlebot and the closest
 *   point on the ball, and
 * - navigating the turtlebot towards the ball using proportional control.
 * It can be stopped by a service call to the 'enable' topic.
 */

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/SetBool.h"
#include "ros/ros.h"

#define PI_F 3.14159265358979f

// service and topic names
const std::string enable_service_name = "enable";
const std::string displacement_topic_name = "displacement";
const std::string movement_topic_name = "/cmd_vel_mux/input/teleop";
const std::string image_scan_topic_name = "/scan";

// thresholds within which the ball is defined to be found
const float stop_threshold_meters = 0.1;
const float stop_threshold_rad    = 0.05;

// ROS parameters
const int queue_size = 1000;
const int loop_freq_hz = 10;

// proportional control constants for ball tracker
const float prop_control_constant_dist  = 0.1;
const float prop_control_constant_angle = 1.0;

// exit constant
const int exit_success = 0;


class TurtlebotSeeker {
    /* This class contains all of the functionality necessary for the seeker 
     * running on a turtlebot Gazebo simulation.
     * When the seeker is enabled, it processes laser scan data, finds the ball
     * (if visible), and moves towards it.
     */

public:
    TurtlebotSeeker(float prop_control_constant_dist,
                    float prop_control_constant_angle);
    bool enableServerCallback(std_srvs::SetBool::Request&, 
                              std_srvs::SetBool::Response&);
    void imageScanCallback(const sensor_msgs::LaserScan::ConstPtr&);
    bool isEnabled();
    bool isBallVisible();
    geometry_msgs::Vector3 getDisplacement();
    geometry_msgs::Twist getControlVels();

private:
    bool m_is_enabled;
    geometry_msgs::Vector3 m_ball_displacement_meters;
    geometry_msgs::Twist m_control_vels_m_per_sec;
    float m_control_k_dist;
    float m_control_k_angle;
    void calcControlVelocities();
};


/*** PUBLIC METHODS ***/

// seeker should be disabled by default
TurtlebotSeeker::TurtlebotSeeker(float control_k_dist, float control_k_angle): 
    m_is_enabled(false),
    m_control_k_dist(control_k_dist),
    m_control_k_angle(control_k_angle) {

    // location of ball is initially unknown
    this->m_ball_displacement_meters.x = NAN;
    this->m_ball_displacement_meters.y = NAN;
    // this turtlebot can not fly
    this->m_ball_displacement_meters.z = 0.0;

    // turtlebot should not initially move
    this->m_control_vels_m_per_sec.linear.x = 0.0;
    this->m_control_vels_m_per_sec.linear.y = 0.0;
    this->m_control_vels_m_per_sec.linear.z = 0.0;
    this->m_control_vels_m_per_sec.angular.x = 0.0;
    this->m_control_vels_m_per_sec.angular.y = 0.0;
    this->m_control_vels_m_per_sec.angular.z = 0.0;
}

// make the enabled accessible outside the service
bool TurtlebotSeeker::enableServerCallback(std_srvs::SetBool::Request  &req,
                                           std_srvs::SetBool::Response &res) {
    this->m_is_enabled = req.data;

    res.success = true;
    res.message = (this->m_is_enabled)? "Service enabled.":"Service disabled.";
    ROS_INFO("%s", res.message.c_str());
    return true;
}

// image_scan callback identifies the ball position and calls a function to
// identify the velocities necessary to get there.
// NOTE: does not currently take into account timing between scans 
//       assumes scans are fast enough to be considered instantaneous
void TurtlebotSeeker::imageScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    // find minimum distance to ball + accompanying angle
    float min_dist_angle_rad = scan->angle_min;
    float min_dist_meters = INFINITY;

    // iterate over all angles to find minimum distance
    for (int i = 0; i < scan->ranges.size(); ++i) {
        float dist_meters = scan->ranges[i];

        // if no object is registered within range, set distance to infinity
        if (std::isnan(scan->ranges[i])) {
            dist_meters = INFINITY;
        }

        float angle_rad = scan->angle_min + scan->angle_increment * i;
        
        if (dist_meters < min_dist_meters) {
            min_dist_meters = dist_meters;
            min_dist_angle_rad = angle_rad;
        }
    }

    ROS_INFO("The closest point on the ball is at angle: %f.", min_dist_angle_rad);
    // find relative displacement to closest point on the ball
    // if no ball viewed close by, 
    //    set displacement to nan to indicate not found
    if (!std::isinf(min_dist_meters)) {
        this->m_ball_displacement_meters.x = min_dist_meters * std::cos(min_dist_angle_rad);
        this->m_ball_displacement_meters.y = min_dist_meters * std::sin(min_dist_angle_rad);
    } else {
        this->m_ball_displacement_meters.x = NAN;
        this->m_ball_displacement_meters.y = NAN;
    }
    
    // give this information to set the control velocities
    this->calcControlVelocities();
}

// gets whether service is enabled or not
bool TurtlebotSeeker::isEnabled() {
    return this->m_is_enabled;
}

// the robot sees the ball with the scanner if the ball displacement is not nan
bool TurtlebotSeeker::isBallVisible() {
    return !(std::isnan(this->m_ball_displacement_meters.x) || std::isnan(this->m_ball_displacement_meters.y));
}

// gets displacement of ball from robot
geometry_msgs::Vector3 TurtlebotSeeker::getDisplacement() {
    return this->m_ball_displacement_meters;
}

// gets next set of control velocities to move robot to ball
geometry_msgs::Twist TurtlebotSeeker::getControlVels() {
    return this->m_control_vels_m_per_sec;
}


/*** PRIVATE METHODS ***/

// a proportional controller to move the robot to the ball
void TurtlebotSeeker::calcControlVelocities() {
    /* The turtlebot has one dimension of translational control (x) and one of 
     * angular control (z). All others are kept at 0 since they don't change the
     * turtlebot's motion.
     */

    float dist_x_meters = this->m_ball_displacement_meters.x;
    float dist_y_meters = this->m_ball_displacement_meters.y;

    float dist_meters = std::sqrt(std::pow(dist_x_meters, 2) + std::pow(dist_x_meters, 2));
    float theta_rad   = std::atan2(dist_y_meters, dist_x_meters);
    
    // if ball is not visible, stop motion.
    if (!this->isBallVisible()) {
        this->m_control_vels_m_per_sec.linear.x  = 0.0;
        this->m_control_vels_m_per_sec.angular.z = 0.0;
        ROS_INFO("Ball out of sight.");
        return;
    }

    // normalize angle between (-pi, pi]
    theta_rad = (theta_rad <= PI_F)? theta_rad: theta_rad - 2*PI_F;

    // if the ball is within the goal threshold, stop moving
    if (dist_meters <= stop_threshold_meters && std::abs(theta_rad) <= stop_threshold_rad) {
        this->m_control_vels_m_per_sec.linear.x  = 0.0;
        this->m_control_vels_m_per_sec.angular.z = 0.0;
        ROS_INFO("Parked next to ball.");
        return;
    }

    // proportional controller to point track distance to the ball
    this->m_control_vels_m_per_sec.linear.x  = this->m_control_k_dist * dist_meters;
    // proportional controller to point track angle towards the ball
    this->m_control_vels_m_per_sec.angular.z = this->m_control_k_angle * theta_rad;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "seeker_node");

    ros::NodeHandle seeker_node;
    TurtlebotSeeker seeker(prop_control_constant_dist,
                           prop_control_constant_angle);

    // input information - is_enabled, laser scan
    ros::ServiceServer enable_server = seeker_node.advertiseService(enable_service_name,
                                                                    &TurtlebotSeeker::enableServerCallback, 
                                                                    &seeker);
    ros::Subscriber image_scan_sub = seeker_node.subscribe(image_scan_topic_name, 
                                                           queue_size, 
                                                           &TurtlebotSeeker::imageScanCallback, 
                                                           &seeker);
    
    // output information - displacement from closest point on ball, movement commands
    ros::Publisher displacement_pub = seeker_node.advertise<geometry_msgs::Vector3>(displacement_topic_name, queue_size);
    ros::Publisher movement_cmd_pub = seeker_node.advertise<geometry_msgs::Twist>(movement_topic_name, queue_size);

    // set a rate for delays between loops
    ros::Rate loop_rate(loop_freq_hz);

    while(ros::ok()) {
        // do nothing if service is disabled
        if (seeker.isEnabled()) {

            // publish the relative displacement of the ball
            displacement_pub.publish(seeker.getDisplacement());

            // set the turtlebot's control velocity parameters
            movement_cmd_pub.publish(seeker.getControlVels());
        }
        // one iteration of callback listening + loop delay 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return exit_success;
}
