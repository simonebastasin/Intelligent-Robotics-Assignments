/* 
This ROS node implements an "action client" that prompts the user to insert a Pose_B (location and orientation goals) towards which to send Tiago.
The action client sends the Pose_B goal to a Proxy node which dispatches the request to MoveBase server or our future MotionControlLaw server.
The action client also implements callbacks to the feedback of the action proxy and prints the task's current status in the terminal.
*/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h> // the action client nodes uses actionlib simple_action_client class
#include <actionlib/client/terminal_state.h>
#include <tiago_iaslab_simulation/ClientProxyAction.h> // our defined action structure generated when building the ClientProxy.action file


// Called once when the goal becomes active
void activeCb() {
	ROS_INFO("Action proxy started, goal sent to it");
}

// This callback is called every time a feedback is received from the proxy for the sended goal
// may be eg: robot is moving, robots is stopped, robot has reached Pose_B goal etc.
void feedbackCb(const tiago_iaslab_simulation::ClientProxyFeedbackConstPtr& feedback) {
	ROS_INFO("Tiago status: %s", feedback->current_status.c_str());
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const tiago_iaslab_simulation::ClientProxyResultConstPtr& result) {
	// check if goal succeded or failed and output a message accordingly
	if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Tiago successfully detected %d obstacles from Pose_B", (int)result->x_positions.size());
	}
	else {
		ROS_INFO("Tiago was NOT able to complete the task");
	}
	// if at least one obstacle has been detected, print positions
	if (result->x_positions.size() != 0) {
		// print obstacles positions in Robot reference frame
		ROS_INFO("Detected obstacles positions (x, y) w.r.t Robot reference frame:" );
		for (int i = 0; i < result->x_positions.size(); i++) {
			ROS_INFO("(%lf, %lf)", (float)result->x_positions[i], (float)result->y_positions[i]);
		}
		// print obstacles positions in World reference frame
		ROS_INFO("Detected movable obstacles positions (x, y) w.r.t World reference frame:" );
		for (int i = 0; i < result->x_positions_wrf.size(); i++) {
			ROS_INFO("(%lf, %lf)", (float)result->x_positions_wrf[i], (float)result->y_positions_wrf[i]);
		}
	}
}


int main(int argc, char** argv) {
	// user are asked to insert only (x,y,yaw) values beacuse it's always z=0, roll=0, pitch=0
	ros::init(argc, argv, "client");
	if (argc != 4) {
		ROS_INFO("Please pass as arguments: pose_x pose_y yaw (the two coordinates for Pose_B and the yaw angle in degrees)");
		return 1;
	}
	// initializing action client 
	actionlib::SimpleActionClient<tiago_iaslab_simulation::ClientProxyAction> ac("proxy_topic", true);
	// connecting to proxy
    ROS_INFO("Waiting for action proxy to start");
    // wait for action server to start (for infinite time)
	ac.waitForServer();

    // sending the goal to action proxy
	tiago_iaslab_simulation::ClientProxyGoal goal;
	goal.x = atof(argv[1]);
	goal.y = atof(argv[2]);
	goal.yaw_degrees = atof(argv[3]);
	// push goal to move_base node for processing
	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	// wait for goal to finish (wait for the action to return)
	ac.waitForResult();
	
	return 0;
}