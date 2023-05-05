/*
TiagoActionProxy node is a ROS action client/server that acts as a proxy between the action client and move_base action server. 

When a goal is received from client as simple x,y,yaw, it constructs the goal as required by move_base,
it creates an action client towards the "move_base' action server and dispatches the goal to it. 

It also publishes feedback back to the client through as_ (actionlib::simpleActionServer) object at a rate of 1Hz. 
The feedback includes the current status of the robot, which is determined by checking the position and orientation of the robot as reported by the 'move_base" action. 
This is done by the "feedbackcb" function that is called every time feedback is received for the goal from move_base. 
This function updates the "current_status" field of the feedback message and publishes it to the as_ object.

Once the Robot has reached the Pose_B goal asks to detMovObs node through a service to return the centers of the movable obstacles, 
and prints them at terminal.

If all goes well sets to succeed the goal received by the client.
*/

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
// As server
#include <actionlib/server/simple_action_server.h>
#include <tiago_iaslab_simulation/ClientProxyAction.h>
#include <cmath>
// As client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "tiago_iaslab_simulation/ProxyDetection.h"


// Called once when the goal becomes active (i.e. when the pose_B goal is sent to move_base)
void activeCb() {
    ROS_INFO("Action server move_base started, goal sent to it");
}

// Called once the goal sent to move_base has finished
void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Action server move_base finished in state [%s]", state.toString().c_str());
}


class TiagoActionProxy {

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

    TiagoActionProxy(std::string name) : 
    as_(nh_, name, boost::bind(&TiagoActionProxy::executeCB, this, _1), false), action_name_(name) {
        as_.start();
        x_prec = 0; y_prec = 0; yaw_prec = 0;
    }
    ~TiagoActionProxy(void) { }

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
        //as_.publishFeedback(feedback_);
    }
    
    // this callback is called when the class is instantiated
    void executeCB(const tiago_iaslab_simulation::ClientProxyGoalConstPtr &goal) {
        ROS_INFO("----------");

        // ---------------------- MOVE_BASE ----------------------

        // action client to communicate with "move_base" action that adheres to MoveBaseAction interface
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true); // tell action client to spin a thread by default
        ROS_INFO("Goal received, waiting for move_base server to start");
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
        ac.sendGoal(mb_goal, &doneCb, &activeCb, boost::bind(&TiagoActionProxy::feedbackCb, this, _1)); // push goal to move_base node

        ac.waitForResult(); // wait for goal to move_base to finish
        bool success = true;

        // check if goal (reach PoseB) succeded or failed
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            feedback_.current_status = "the robot is stopped and reached Pose_B";
            as_.publishFeedback(feedback_);
            as_.setSucceeded();
        }
        else {
            ROS_ERROR("Failed to reach Pose_B");
            success = false;
            feedback_.current_status = "the robot is stopped and did NOT reach Pose_B";
            as_.publishFeedback(feedback_);
            // received action set to aborted due to not reaching Pose_B (and will not perform detection)
            as_.setAborted();
        }

        success = false; // for assigment 2 detection is disabled, remove this line to enable it

        // ---------------------- OBSTACLES DETECTION ----------------------

        // perform detection only if Pose_B has been reached
        if(success) {

            // Calling detMovObs node through a service to get obstacle centers
            ROS_INFO("Sending request for detection");
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
    ros::init(argc, argv, "proxy");
    // instatiating class object that will act as action server
    TiagoActionProxy tiagoActionProxy_instance("proxy_topic");
    ROS_INFO("Ready to receive client requests");
    ros::spin();
    return 0;
}