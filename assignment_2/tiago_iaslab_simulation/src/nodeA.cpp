#include <ros/ros.h>
// the action client node uses actionlib simple_action_client class
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// our defined action structure generated when building the ClientProxy.action file to communicate with our proxy for navigation
#include <tiago_iaslab_simulation/ClientProxyAction.h>
#include <tiago_iaslab_simulation/PickAndPlaceAction.h>
// service file towards human_node to retrieve IDs picking order
#include "tiago_iaslab_simulation/Objs.h"
#include <vector>
// to move the torso
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
// to move the head
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/shared_ptr.hpp>
// service file to perform object detection through nodeB
#include "tiago_iaslab_simulation/ObjDetection.h"
#include "geometry_msgs/PoseArray.h"

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient; 
typedef boost::shared_ptr<PointHeadClient> PointHeadClientPtr;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> PointTorsoClient; 
typedef boost::shared_ptr<PointTorsoClient> PointTorsoClientPtr;

// global position variables wrt "map" reference frame (frame wrt proxy send pose goal to move_base)
// these can be assumed a priori, not the position of the objects on the table
const vector<float> wayPointToPick {8.5, 0, -90};
const vector<float> wayPointToGreen {8.5, -4, 0};

//offsets are to be far enough from table
const vector<float> pickGreenPose {6.579991 + 1.186584, - 1.369981 - 1.967163 - 0.8, 90}; 
const vector<float> pickBluePose {6.579991 + 1.521553 - 0.2, - 1.369981 - 1.278487 + 0.8, - 90}; 
const vector<float> pickRedPose {6.579991 + 0.939518 + 0.2, - 1.369981 - 1.284885 + 0.8, - 90}; 

const vector<float> placeRedPose {6.579991 + 4.007962 - 0.1, - 1.369981 + 1.015966 + 0.8, - 90 - 5};
const vector<float> placeGreenPose {6.579991 + 5.007404 - 0.1, - 1.369981 + 1.015966 + 0.8, - 90 - 5};
const vector<float> placeBluePose {6.579991 + 6.007146 - 0.1, - 1.369981 + 1.015966 + 0.8, - 90 - 5};

const vector<vector<float>> pickPoses {pickBluePose, pickGreenPose, pickRedPose}; // ordered as ID blue=1, green=2, red=3
const vector<vector<float>> placePoses {placeBluePose, placeGreenPose, placeRedPose}; 


// __________ ACTION CLIENT METHODS __________


// Called once when the goal becomes active
void activeCb() {
	ROS_INFO("Action proxy started, goal sent to it");
}

// This callback is called every time a feedback is received from the proxy for the sended goal
void feedbackCb(const tiago_iaslab_simulation::ClientProxyFeedbackConstPtr& feedback) {
	ROS_INFO("Tiago status: %s", feedback->current_status.c_str());
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const tiago_iaslab_simulation::ClientProxyResultConstPtr& result) {
	// check if goal succeded or failed and output a message accordingly
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Tiago successfully reached the goal pose"); 
	}
	else {
		ROS_INFO("Tiago was NOT able to reach the goal pose");
	}
}

void activeCbPnP() {
	ROS_INFO("PickAndPlace started, goal sent to it");
}

void feedbackCbPnP(const tiago_iaslab_simulation::PickAndPlaceFeedbackConstPtr& feedback) {
	ROS_INFO("Tiago status: %s", feedback->current_status.c_str());
}

void doneCbPnP(const actionlib::SimpleClientGoalState& state, const tiago_iaslab_simulation::PickAndPlaceResultConstPtr& result) {
	// check if goal succeded or failed and output a message accordingly
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Tiago successfully performed pick and place task"); 
	}
	else {
		ROS_INFO("Tiago was NOT able to perform pick and place task");
	}
}


// __________ FUNCTIONS __________


void moveToPose(int id, bool going_to_place) {
	ROS_INFO("\nMovePickAndPlace routine for ID: %d", id);
	
	// connect to proxy from assigment1 to send pose goals towards which make the robot move
	actionlib::SimpleActionClient<tiago_iaslab_simulation::ClientProxyAction> ac("/proxy_topic", true);
    ROS_INFO("Waiting for action proxy to start");
	ac.waitForServer();

    // goal to action proxy to move robot
	tiago_iaslab_simulation::ClientProxyGoal goal_pose;

	// set waypoint only to go away from table after pick of green green triangle
	if(going_to_place && id == 2) {
		goal_pose.x = wayPointToGreen[0];
		goal_pose.y = wayPointToGreen[1];
		goal_pose.yaw_degrees = wayPointToGreen[2];	
		ac.sendGoal(goal_pose, &doneCb, &activeCb, &feedbackCb);
		ac.waitForResult();	
	}

	// set waypoint to go pick
	goal_pose.x = wayPointToPick[0];
	goal_pose.y = wayPointToPick[1];
	goal_pose.yaw_degrees = wayPointToPick[2];
	if(going_to_place) { // if tiago is going to place targets
		goal_pose.yaw_degrees = - wayPointToPick[2]; // reverse yaw_degrees to go place
	}
	ac.sendGoal(goal_pose, &doneCb, &activeCb, &feedbackCb);
	ac.waitForResult();

	if(!going_to_place) { // set pose in front of pick table
		goal_pose.x = pickPoses[id - 1][0]; // get pickColorPose from pickPoses vector by ID (-1 because ID is in range [1,3])
		goal_pose.y = pickPoses[id - 1][1];
		goal_pose.yaw_degrees = pickPoses[id - 1][2];
	}
	else { // pose in front of place target
		goal_pose.x = placePoses[id - 1][0]; // get pickColorPose from pickPoses vector by ID -1
		goal_pose.y = placePoses[id - 1][1];
		goal_pose.yaw_degrees = placePoses[id - 1][2];
	}	
	ac.sendGoal(goal_pose, &doneCb, &activeCb, &feedbackCb);
	ac.waitForResult();
}


void moveTorso(bool head_back_down) {
	PointTorsoClientPtr PointTorsoClientPtr;
	PointTorsoClientPtr.reset( new PointTorsoClient("/torso_controller/follow_joint_trajectory") );
	PointTorsoClientPtr->waitForServer();

	trajectory_msgs::JointTrajectory jointTraj;
	trajectory_msgs::JointTrajectoryPoint jointTrajPoint;
	jointTraj.joint_names.push_back("torso_lift_joint");
	if(!head_back_down) {
		jointTrajPoint.positions.push_back(0.34);
	}
	else {
		jointTrajPoint.positions.push_back(0.15);
	}
    jointTrajPoint.time_from_start = ros::Duration(2.5);
	jointTraj.points.push_back(jointTrajPoint);

	// build the action goal
	control_msgs::FollowJointTrajectoryGoal goalTorso;
	// the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
	goalTorso.trajectory = jointTraj;

	PointTorsoClientPtr->sendGoal(goalTorso);
}


void moveHead(bool head_back_up) {
	// move head to look down to table
	// we can stay at same x,y (global raw position) while z can be etracted from clicking on the object 
	PointHeadClientPtr pointHeadclientPtr;
	pointHeadclientPtr.reset( new PointHeadClient("/head_controller/point_head_action") );
	pointHeadclientPtr->waitForServer();

	geometry_msgs::PointStamped pointStamped;
	pointStamped.header.frame_id = "/xtion_rgb_optical_frame";
	pointStamped.header.stamp = ros::Time::now();

	// compute normalized coordinates of the selected pixel
	double x = 0.0; 
	double y = 0.8; 
	double Z = 1.0; 
	if(head_back_up) y = -y;
	pointStamped.point.x = x * Z;
	pointStamped.point.y = y * Z;
	pointStamped.point.z = Z;

	// build the action goal
	control_msgs::PointHeadGoal goalHead;
	// the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
	goalHead.pointing_frame = "/xtion_rgb_optical_frame";
	goalHead.pointing_axis.x = 0.0;
	goalHead.pointing_axis.y = 0.0;
	goalHead.pointing_axis.z = 1.0;
	goalHead.min_duration = ros::Duration(1.0);
	goalHead.max_velocity = 0.25;
	goalHead.target = pointStamped;

	pointHeadclientPtr->sendGoal(goalHead);
}


void detectObstacles(geometry_msgs::PoseArray &obstacles, vector<int> &pose_ids) {
	// send request to nodeB to perform objects detection 
	// Calling detMovObs node through a service to get obstacle centers
	ROS_INFO("Sending request for objects detection");
	ros::NodeHandle n;
	// client for a service (if persistent connection: when last handle reference cleared, connection closed)
	ros::ServiceClient client = n.serviceClient<tiago_iaslab_simulation::ObjDetection>("/nodeB_service");

	// new service of our service defined type
	tiago_iaslab_simulation::ObjDetection srv;
	srv.request.ready = true;

	if(client.call(srv)) {
		obstacles = srv.response.pose_array;
		pose_ids = srv.response.pose_ids;
		ROS_INFO("Detection done: print detected object poses");
		ROS_INFO("outside for, size: %d", srv.response.pose_ids.size());
		for (int i = 0; i < srv.response.pose_ids.size(); i++) {
			ROS_INFO("inside for");
			ROS_INFO("ID: %d, pos_x: %f, pos_y: %f, pos_z: %f, orient_x: %f, orient_y: %f, orient_z: %f, orient_w: %f", 
				srv.response.pose_ids[i], obstacles.poses[i].position.x,
				obstacles.poses[i].position.y, obstacles.poses[i].position.z,
				obstacles.poses[i].orientation.x, obstacles.poses[i].orientation.y,
				obstacles.poses[i].orientation.z, obstacles.poses[i].orientation.w);
		}
	}
}


void pickAndPlace(bool pick, int id, vector<int> ids, geometry_msgs::PoseArray pose_array){
	actionlib::SimpleActionClient<tiago_iaslab_simulation::PickAndPlaceAction> ac("/nodeC_topic", true);
    ROS_INFO("Waiting for action proxy to start");
	ac.waitForServer();

	tiago_iaslab_simulation::PickAndPlaceGoal pap_goal;
	pap_goal.pickOrPlace = pick; // true (pick) or false (place)
	pap_goal.id = id;
	if(pick) { // only if going to pick
		pap_goal.pose_ids = ids;
		pap_goal.pose_array = pose_array;
	}
	ac.sendGoal(pap_goal, &doneCbPnP, &activeCbPnP, &feedbackCbPnP);
	ac.waitForResult();
}


// __________ MAIN __________


int main(int argc, char** argv) {
	ros::init(argc, argv, "nodeA");

    // Calling human_node node through a service to get ids order
    ROS_INFO("Sending request to human_node for IDs order by which to pick the objects");
    ros::NodeHandle n;
    // client for a service 
    ros::ServiceClient client = n.serviceClient<tiago_iaslab_simulation::Objs>("/human_objects_srv");
    ros::service::waitForService("/human_objects_srv");
    
    tiago_iaslab_simulation::Objs srv; 
    srv.request.ready = true;
    srv.request.all_objs = true; // to get all IDs at once
    vector<int> ids; // from response: int32[] ids
    // invoke an RPC service with [service: contains req and res msgs], returns true on success
    if(client.call(srv)) {
		ids = srv.response.ids;
        ROS_INFO("%d | %d | %d", (int)ids[0], (int)ids[1], (int)ids[2]);
    }
    else {
        ROS_ERROR("Failed to get IDs from human_node");
    }

    for(int i = 0; i < ids.size(); i++) {
		geometry_msgs::PoseArray obstacles;
		vector<int> pose_ids;

		moveToPose(ids[i], false);
		moveTorso(false);
		moveHead(false);
		ros::Duration(3.0).sleep();

		detectObstacles(obstacles, pose_ids);
		pickAndPlace(true, ids[i], pose_ids, obstacles);

		moveHead(true);
		moveTorso(true);
		moveToPose(ids[i], true);

		moveHead(false);
		moveTorso(false);
		ros::Duration(3.0).sleep();

		pickAndPlace(false, ids[i], pose_ids, obstacles);

		moveHead(true);
		moveTorso(true);
	}
	
	return 0;
}