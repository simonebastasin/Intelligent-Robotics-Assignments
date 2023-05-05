#include <ros/ros.h>
// As server
#include <actionlib/server/simple_action_server.h>
#include <tiago_iaslab_simulation/ProxyMotLawAction.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
//#include <ros/multi_threaded_spinner.h>


class MotionLawServer {

private:

    ros::Publisher vel_pub;

protected:
    
    ros::NodeHandle nh_;
    // as server toward client
    actionlib::SimpleActionServer<tiago_iaslab_simulation::ProxyMotLawAction> as_;
    std::string action_name_;
    // create messages that are used to publish feedback/result to the client
    tiago_iaslab_simulation::ProxyMotLawFeedback feedback_;
    tiago_iaslab_simulation::ProxyMotLawResult result_;
    
public:

    MotionLawServer(std::string name) : 
    as_(nh_, name, boost::bind(&MotionLawServer::executeCB, this, _1), false), action_name_(name) {
        as_.start();
        // Set up a publisher to the cmd_vel topic
        vel_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
    }
    ~MotionLawServer(void) { }

    // this callback is called when the class is instantiated
    void executeCB(const tiago_iaslab_simulation::ProxyMotLawGoalConstPtr &goal) {
        ROS_INFO("----------");
        ROS_INFO("Goal received and manual motion control law started");
        
        result_.all_good = true;
        ros::NodeHandle n;

        boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> pose_msg_sharePtr;
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        // subscribe to scan topic to get laser scan data
        pose_msg_sharePtr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose", n);
        if (pose_msg_sharePtr != NULL) {
            pose_msg = *pose_msg_sharePtr;
        }
        if (abs(pose_msg.pose.pose.position.x - goal->x) < 0.2 &&  abs(pose_msg.pose.pose.position.y - goal->y) < 0.2) {
            result_.all_good = false;
            ROS_INFO("Navigation ended: robot already in Pose_B");
            feedback_.current_status = "the robot is already in Pose_B";
            as_.publishFeedback(feedback_);
        }
        else if (pose_msg.pose.pose.position.x > 7.0) {
            result_.all_good = false;
            ROS_INFO("Navigation done: robot already passed the narrow corridor");
            feedback_.current_status = "the robot already passed the narrow corridor";
            as_.publishFeedback(feedback_);
            // fuori corridoio -> move_base
        }

        if (result_.all_good) {
            ros::Rate r(10); // 10 Hz

            while (ros::ok()) {
                boost::shared_ptr<sensor_msgs::LaserScan const> laser_msg_sharePtr;
                sensor_msgs::LaserScan laser_msg;
                // subscribe to scan topic to get laser scan data
                laser_msg_sharePtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", n);
                if (laser_msg_sharePtr != NULL) {
                    laser_msg = *laser_msg_sharePtr;
                }
                
                geometry_msgs::Twist vel;
                // Process the laser data to determine the distance to obstacles in the environment
                float min_distance = laser_msg.range_max;
                int toDiscard = 20;
                int ranges_size = laser_msg.ranges.size();
                int min_index = ranges_size;
                for (int i = toDiscard; i < ranges_size - toDiscard; i++) {
                    if (laser_msg.ranges[i] < min_distance) {
                        min_distance = laser_msg.ranges[i];
                        min_index = i;
                    }
                }
                // Set the linear velocity based on the minimum distance to obstacles
                vel.linear.x = 0.5; // starting linear velocity same as the max observed with move_base
                if (min_distance < 0.25) { // if closest wall is closer than 0.25
                    vel.linear.x = 0.0; // stops
                }
                else if (min_distance < 0.5) { // if closest wall is closer than 0.5
                    vel.linear.x = 0.2; // slower
                }

                // Set the angular velocity to steer the robot towards the center of the corridor
                float center_angle = laser_msg.angle_max + laser_msg.angle_min; // = 0 in this case
                float swing = 30;
                vel.angular.z = center_angle - laser_msg.angle_increment * swing;
                
                if (min_index < ranges_size / 2) { // if closest wall is at the right side of the robot
                    vel.angular.z = - vel.angular.z;
                }
                // Publish the velocities to move the robot
                feedback_.current_status = "the robot is moving";
                as_.publishFeedback(feedback_);
                vel_pub.publish(vel);

                float fi = 4.0; // parameter to check end of the narrow corridor
                // check if first or last range scanned give a distance larger than fi (i.e., end of the narrow corridor)
                if (laser_msg.ranges[toDiscard] > fi || laser_msg.ranges[ranges_size-toDiscard-1] > fi) {
                    ROS_INFO("Navigation done: sending back response to proxy");
                    feedback_.current_status = "the robot succeeded to navigate through the narrow corridor";
                    as_.publishFeedback(feedback_);
                    break;
                }
                ros::spinOnce();
                r.sleep();
            }
        }
        as_.setSucceeded(result_);
    }
};


int main(int argc, char** argv) {
    // instantiate node
    ros::init(argc, argv, "motion_contro_law_server");
    // instatiating class object that will act as action server
    MotionLawServer motionLawServer_instance("custom_motion_law_topic");
    ROS_INFO("Ready to receive Pose_B goal from proxy");
    ros::spin();
    return 0;
}