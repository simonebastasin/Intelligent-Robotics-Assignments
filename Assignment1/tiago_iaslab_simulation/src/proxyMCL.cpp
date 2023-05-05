/*
TiagoActionProxy node is a ROS action client/server that acts as a proxy between the action client and the custom motion law server  controller. 
*/

#include <ros/ros.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// As server
#include <actionlib/server/simple_action_server.h>
#include <tiago_iaslab_simulation/ClientProxyAction.h>
#include <cmath>
// As client to move_base
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
// As client to our defined motion control law server
#include <tiago_iaslab_simulation/ProxyMotLawAction.h>
// for detection 
#include "tiago_iaslab_simulation/ProxyDetection.h"

                                                                                                                           
// Called once when the goal becomes active (i.e. when the pose_B goal is sent to move_base)
void activeCb() {
    ROS_INFO("Action server move_base started, goal sent to it");
}

// Called once the goal sent to move_base has finished
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Action server move_base finished in state [%s]", state.toString().c_str());
}     

// Called once when the goal becomes active (i.e. when goal is sent to motion control law server)
void activeCbMCL() {
    ROS_INFO("Action server motion_law started, goal sent to it");
}

// Called once the goal sent to motion control law has finished
void doneCbMCL(const actionlib::SimpleClientGoalState& state, const tiago_iaslab_simulation::ProxyMotLawResultConstPtr& result) {
    ROS_INFO("Action server motion_law finished in state [%s]", state.toString().c_str());
}


class TiagoActionProxyML {

private:

    // variables to store previous robots pose and check if a movement has occurred
    float x_prec, y_prec, yaw_prec; 

protected:
    
    ros::NodeHandle nh_;
    // as server toward client
    actionlib::SimpleActionServer<tiago_iaslab_simulation::ClientProxyAction> as_;
    std::string action_name_;
    // create messages that are used to publish feedback/result to the client
    tiago_iaslab_simulation::ClientProxyFeedback feedback_;
    tiago_iaslab_simulation::ClientProxyResult result_;
    
public:

    TiagoActionProxyML(std::string name) : 
    as_(nh_, name, boost::bind(&TiagoActionProxyML::executeCB, this, _1), false), action_name_(name) {
        as_.start();
        x_prec = 0; y_prec = 0; yaw_prec = 0;
    }
    ~TiagoActionProxyML(void) { }

    // Called every time feedback is received for the goal dispatched to move_base
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        // retrieve from move_base feedback the current x, y, yaw positions
        float x = feedback->base_position.pose.position.x;
        float y = feedback->base_position.pose.position.y;
        // convert from Quaternion to roll=0, pitch=0, yaw
        tf2::Quaternion myQuaternion(feedback->base_position.pose.orientation.x, feedback->base_position.pose.orientation.y,
            feedback->base_position.pose.orientation.z, feedback->base_position.pose.orientation.w);
        myQuaternion.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(myQuaternion);
        m.getRPY(roll, pitch, yaw);
        // check robot status by comparing current pose values with previous ones
        if(x != x_prec || y != y_prec || yaw != yaw_prec) {
            feedback_.current_status = "the robot is moving";
        }
        else { // if(x == x_prec && y == y_prec && yaw == yaw_prec) {
            feedback_.current_status = "the robot is stopped";
        }
        x_prec = x; y_prec = y; yaw_prec = yaw;
        as_.publishFeedback(feedback_);
    }

    // Called every time feedback is received for the goal dispatched to motion control law
    void feedbackCbMCL(const tiago_iaslab_simulation::ProxyMotLawFeedbackConstPtr& feedback) {
        feedback_.current_status = feedback->current_status;
        as_.publishFeedback(feedback_);
    }

    
    // this callback is called when the class is instantiated
    void executeCB(const tiago_iaslab_simulation::ClientProxyGoalConstPtr &goal) {
        ROS_INFO("----------");


        // ---------------------- MOTION CONTROL LAW ----------------------

        // create connection towards motionLawServer
        actionlib::SimpleActionClient<tiago_iaslab_simulation::ProxyMotLawAction> ac_mcl("custom_motion_law_topic", true); // tell action client to spin a thread by default
        ROS_INFO("Goal received, waiting for our motion control law server to start");
        ac_mcl.waitForServer(); // wait for action server to start (for infinite time)

        tiago_iaslab_simulation::ProxyMotLawGoal mcl_goal;
        mcl_goal.x = goal->x;
        mcl_goal.y = goal->y;
        mcl_goal.yaw_radians = goal->yaw_degrees * M_PI / 180.0;
        ac_mcl.sendGoal(mcl_goal, &doneCbMCL, &activeCbMCL, boost::bind(&TiagoActionProxyML::feedbackCbMCL, this, _1)); // push goal to move_base node
        ac_mcl.waitForResult(); // wait for goal to move_base to finish
        
        // check if goal (reach PoseB) succeded or failed
        if(ac_mcl.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            feedback_.current_status = "robot had an error during manual motion control law";
            as_.publishFeedback(feedback_);
        }
        ROS_INFO("Manual motion control law ended, switching to move_base server");
        feedback_.current_status = "the robot motion control is switched to move_base";
        as_.publishFeedback(feedback_);


        // ---------------------- MOVE_BASE ----------------------

        // action client to communicate with "move_base" action that adheres to MoveBaseAction interface
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); // tell action client to spin a thread by default
        ROS_INFO("Waiting for move_base server to start");
        ac.waitForServer(); // wait for action server to start (for infinite time)

        // convert yaw input parameter to Quaternion (x,y,z,w)
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, goal->yaw_degrees * M_PI / 180.0); // roll=0, pitch=0, yaw
        myQuaternion.normalize();
        // goal to send to move_base server
        move_base_msgs::MoveBaseGoal mb_goal; // position (x,y,z), orientation (x,y,z,w)
        mb_goal.target_pose.header.frame_id = "map";
        mb_goal.target_pose.header.stamp = ros::Time::now();
        mb_goal.target_pose.pose.position.x = goal->x;
        mb_goal.target_pose.pose.position.y = goal->y;
        mb_goal.target_pose.pose.position.z = 0.0;
        mb_goal.target_pose.pose.orientation.x = myQuaternion.getX();
        mb_goal.target_pose.pose.orientation.y = myQuaternion.getY();
        mb_goal.target_pose.pose.orientation.z = myQuaternion.getZ();
        mb_goal.target_pose.pose.orientation.w = myQuaternion.getW();
        // binding is need as feedbackCB is defined inside the class, since has to access its private variables
        ac.sendGoal(mb_goal, &doneCb, &activeCb, boost::bind(&TiagoActionProxyML::feedbackCb, this, _1)); // push goal to move_base node

        ac.waitForResult(); // wait for goal to move_base to finish
        bool success = true;

        // check if goal (reach PoseB) succeded or failed
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            feedback_.current_status = "the robot is stopped and reached Pose_B";
            as_.publishFeedback(feedback_);
        }
        else {
            ROS_ERROR("Failed to reach Pose_B");
            success = false; // Pose_B not reached
            feedback_.current_status = "the robot is stopped and did NOT reach Pose_B";
            as_.publishFeedback(feedback_);
            // received action set to aborted due to not reaching Pose_B (and will not perform detection)
            as_.setAborted();
        }


        // ---------------------- OBSTACLES DETECTION ----------------------

        // perform detection only if Pose_B has been reached
        if(success) {

            // Calling detMovObs node through a service to get obstacle centers
            ROS_INFO("Pose_B reached, sending request for detection");
            ros::NodeHandle n;
            // client for a service (if persistent connection: when last handle reference cleared, connection closed)
            ros::ServiceClient client = n.serviceClient<tiago_iaslab_simulation::ProxyDetection>("laserScan_topic");

            // new service of our service defined type
            tiago_iaslab_simulation::ProxyDetection srv;
            srv.request.x = goal->x;
            srv.request.y = goal->y;
            srv.request.yaw_radians = goal->yaw_degrees * M_PI / 180.0;

            // informing client that the robot has started obstacles detection
            feedback_.current_status = "the robot started the detection of the obstacles";
            as_.publishFeedback(feedback_);

            result_.x_positions.clear(); result_.x_positions_wrf.clear();
            result_.y_positions.clear(); result_.y_positions_wrf.clear();
            // invoke an RPC service with [service: contains req and res msgs], returns true on success
            if (client.call(srv)) {
                ROS_INFO("Detection done: sending back response to client");
                // copy obstacle positions in action result
                for (int i = 0; i < srv.response.x_positions.size(); i++) {
                    // robot reference frame
                    result_.x_positions.push_back(srv.response.x_positions[i]);
                    result_.y_positions.push_back(srv.response.y_positions[i]);
                    // world reference frame
                    result_.x_positions_wrf.push_back(srv.response.x_positions_wrf[i]);
                    result_.y_positions_wrf.push_back(srv.response.y_positions_wrf[i]);
                }
                // informing client that the detection has finished 
                feedback_.current_status = "the detection is finished";
                as_.publishFeedback(feedback_);
                // setting to success client request
                as_.setSucceeded(result_);
            }
            else {
                ROS_ERROR("Failed to call movable obstacles detection service");
                feedback_.current_status = "the detection failed";
                as_.publishFeedback(feedback_);
                // received action set to aborted due to obstacle detection gone wrong
                as_.setAborted();
            }
        }
    }
};


int main(int argc, char** argv) {
    // instantiate node
    ros::init(argc, argv, "proxy_mcl"); // proxy for motion control law
    // instatiating class object that will act as action server
    TiagoActionProxyML tiagoActionProxyML_instance("proxy_topic");
    ROS_INFO("Ready to receive client requests");
    ros::spin();
    return 0;
}