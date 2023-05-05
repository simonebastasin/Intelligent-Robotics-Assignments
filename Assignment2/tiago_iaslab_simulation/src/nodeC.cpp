/*
nodeC implements a ROS action server that performs the pick and place routine by means of the MoveIt library. 

The Goal field contains info about the detected objects on the table (detected by nodeB), to be added as collision objects to the scene. 
Only the pose of these objects are passed (wrt to base_link), their size can be determined and used A PRIORI. (private fields of the NodeC class)
Similarly also the tables from which to pick and where to place can be added here with pose and dimensions known A PRIORI.

A boolean pickOrPlace indicates if the node has to perform the pick or place task
In case pick task is requested the ID of the object to be picked is needed

During the actions the server writes in current_status to inform the client NodeA about what is doing ("moving arm", "closing gripper", ...)

A boolean success field is used to indicate if everything went successfully.
*/

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tiago_iaslab_simulation/PickAndPlaceAction.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_ros_link_attacher/Attach.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/PoseArray.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <cmath>
#include <vector>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> MoveGripperClient; 
typedef boost::shared_ptr<MoveGripperClient> MoveGripperClientPtr;


// __________ CLASS __________


class NodeC {

private:
    const vector<float> placeRedPose {6.579991 + 4.007962, - 1.369981 + 1.015966};
    const vector<float> placeGreenPose {6.579991 + 5.007404, - 1.369981 + 1.015966};
    const vector<float> placeBluePose {6.579991 + 6.007146, - 1.369981 + 1.015966};
    const vector<vector<float>> placePoses {placeBluePose, placeGreenPose, placeRedPose};
    const vector<double> place_table_dim{0.69, 0.21}; // heigth, radius

    const vector<double> table_pose{6.579991 + 1.245143, - 1.369981 - 1.613171, 0.7550, 0, 0, 0};
    const vector<double> table_dim{0.913, 0.913, 0.04};
    
    const vector<double> hexagon_dim{0.1, 0.025}; // blue
    const vector<double> triangle_dim{0.05, 0.065, 0.0325}; // green
    const vector<double> cube_dim{0.05}; // red
    const vector<vector<double>> object_dims{hexagon_dim, triangle_dim, cube_dim};
    const vector<double> golden_obstacle_dim{0.2 + 0.02, 0.05 + 0.02}; // other obstacles with id
    
    const float gripper_lenght = 0.2;
    const vector<string> joint_names {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"};
    const vector<vector<float>> safe_arm_joint_pose{
        {0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0}, //{0.20, -1.34, -0.20, 1.94, -1.57, 1.37, 0.0}, {0.21, -1.02, -0.20, 1.94, -1.57, 1.52, 0.0}, 
        {0.21, 0.35, -0.2, 2.0, -1.57, 1.52, 0.0},
        {0.21, 0.35, -0.2, 0.0, -1.57, 0.2, 0.0}, {0.21, 0.35, -3.0, 0.0, -1.57, 0.2, 0.0},
        {0.05, -0.07, -3.0, 1.5, -1.57, 0.2, 0.0}  };

protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tiago_iaslab_simulation::PickAndPlaceAction> as_;
    std::string action_name_;
    // create messages that are used to publish feedback/result to the client
    tiago_iaslab_simulation::PickAndPlaceFeedback feedback_;
    tiago_iaslab_simulation::PickAndPlaceResult result_;

public:

    NodeC(std::string name) : 
    as_(nh_, name, boost::bind(&NodeC::executeCB, this, _1), false), action_name_(name) {
        as_.start();
        // perform any initialization to the private variables if needed
    }
    ~NodeC(void) { }

    
    // ___________ FUNCTIONS ___________

    void closeGripper(bool close) {
        MoveGripperClientPtr MoveGripperClientPtr;
        MoveGripperClientPtr.reset( new MoveGripperClient("/parallel_gripper_controller/follow_joint_trajectory") );
        MoveGripperClientPtr->waitForServer();

        trajectory_msgs::JointTrajectory jointTraj;
        trajectory_msgs::JointTrajectoryPoint jointTrajPoint;
        jointTraj.joint_names.push_back("gripper_left_finger_joint");
        jointTraj.joint_names.push_back("gripper_right_finger_joint");
        if(close) {
            jointTrajPoint.positions.push_back(0.02);
            jointTrajPoint.positions.push_back(0.02);
        }
        else {
            jointTrajPoint.positions.push_back(0.1);
            jointTrajPoint.positions.push_back(0.1);
        }
        jointTrajPoint.time_from_start = ros::Duration(2.5);
        jointTraj.points.push_back(jointTrajPoint);

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = jointTraj;

        MoveGripperClientPtr->sendGoal(goal);
        ros::Duration(2.0).sleep();
    }

    bool planMovement(moveit::planning_interface::MoveGroupInterface &move_group, 
                      moveit::planning_interface::MoveGroupInterface::Plan &plan,
                      const geometry_msgs::Pose &target_pose) {
        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(target_pose);
            
        if(move_group.plan(plan)) {
            feedback_.current_status = "Moving to position"; 
            as_.publishFeedback(feedback_);
            move_group.move();
            ros::Duration(2.0).sleep();
            return true;
        }
        else{
            ROS_ERROR("Failed to plan path to target pose");
            return false;
        }
    }

    void planCartesian(moveit::planning_interface::MoveGroupInterface &move_group, 
                       moveit::planning_interface::MoveGroupInterface::Plan &plan,
                       const geometry_msgs::Pose &target_pose) {
        move_group.setStartStateToCurrentState();
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(target_pose); 
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = move_group.computeCartesianPath(waypoints, 0.03, 0.0, trajectory);
        plan.trajectory_ = trajectory;
        move_group.execute(plan);
        ros::Duration(2.0).sleep();
    }

    bool safePosition(bool moveTo,
                      moveit::planning_interface::MoveGroupInterface &move_group,
                      moveit::planning_interface::MoveGroupInterface::Plan &plan) {
        move_group.setStartStateToCurrentState();
        if(moveTo) {
            for(int i = 0; i < safe_arm_joint_pose.size(); i++) {
                for(int j = 0; j < joint_names.size(); j++) {
                    move_group.setJointValueTarget(joint_names[j], safe_arm_joint_pose[i][j]);
                }
                if(move_group.plan(plan)) {
                    feedback_.current_status = "Moving to safe position"; 
                    as_.publishFeedback(feedback_);
                    move_group.move();
                }
                else {
                    ROS_INFO("Unable to perform joint space planning (safe position) num: %d", i);
                    return false;
                }
            }
        }
        else {
            for(int i = safe_arm_joint_pose.size() - 1 ; i >= 0 ; i--) {
                for(int j = 0; j < joint_names.size(); j++) {
                    move_group.setJointValueTarget(joint_names[j], safe_arm_joint_pose[i][j]);
                }
                if(move_group.plan(plan)) {
                    feedback_.current_status = "Moving from safe to close position"; 
                    as_.publishFeedback(feedback_);
                    move_group.move();
                }
                else {
                    ROS_INFO("Unable to perform joint space plannign (close position) num: %d", i);
                    return false;
                }
            }
        }
        ros::Duration(2.0).sleep();
        return true;
    }

    bool attachGazebo(bool attach, int ID) {
        string model,link;
        switch(ID) {
            case 1:
                model = "Hexagon";
                link = "Hexagon_link";
                break;
            case 2:
                model = "Triangle";
                link = "Triangle_link";
                break;
            case 3:
                model = "cube";
                link = "cube_link";
                break;
        }
        ros::NodeHandle nh;
        ros::ServiceClient attach_client;
        if(attach) {
            attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        }
        else {
            attach_client = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
        }
        gazebo_ros_link_attacher::Attach attach_srv;
        attach_srv.request.model_name_1 = "tiago";
        attach_srv.request.link_name_1 = "arm_7_link";
        attach_srv.request.model_name_2 = model;
        attach_srv.request.link_name_2 = link;
        if(attach_client.call(attach_srv)) {
            ROS_INFO("Links attached");
            return true;
        }
        else {
            ROS_ERROR("Failed to attach links");
            return false;
        }
    }


    // __________ ACTION SERVER METHOD __________


    // this callback is called when the class is instantiated
    void executeCB(const tiago_iaslab_simulation::PickAndPlaceGoalConstPtr &goal) {
        ROS_INFO("node C started");
        
        moveit::planning_interface::MoveGroupInterface move_group("arm"); // or arm_torso
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        move_group.setPlannerId("SBLkConfigDefault");
        move_group.setPoseReferenceFrame("base_footprint"); // or base_link
        move_group.setStartStateToCurrentState();
        move_group.setMaxVelocityScalingFactor(1.0);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        geometry_msgs::PoseStamped safe_pose;
        geometry_msgs::Pose approach_pose;
        tf2::Quaternion target_orientation;
        int ID = goal->id;
         

        // ---------- PICK ----------


        if(goal->pickOrPlace) {
            bool status;

            geometry_msgs::PoseArray obstacles = goal->pose_array;
            geometry_msgs::PoseStamped closed_pose = move_group.getCurrentPose();
            geometry_msgs::Pose target_pose;
            vector<moveit_msgs::CollisionObject> collision_objects;

            int ID_index = 0, num_obstacles = goal->pose_ids.size();
            vector<string> remove_ids(num_obstacles + 1);
            double roll, pitch, yaw, yaw_module, new_yaw, pose_offset;

            for(int i = 0; i < num_obstacles; i++) {
                moveit_msgs::CollisionObject collision_object;
                shape_msgs::SolidPrimitive primitive;
                geometry_msgs::Pose object_pose = goal->pose_array.poses[i];
                tf2::Quaternion triangle_orientation;

                int currID = goal->pose_ids[i];
                collision_object.header.frame_id = "base_footprint";
                collision_object.id = to_string(currID); // current ID of for iteration

                if(currID == ID) {
                    target_pose = object_pose;
                    ID_index = i;
                    tf2::fromMsg(target_pose.orientation, target_orientation);
                    tf2::Matrix3x3(target_orientation).getRPY(roll, pitch, yaw);
                }
                
                switch(currID) {
                    case 1:
                        primitive.type = primitive.CYLINDER;
                        primitive.dimensions.resize(2);
                        primitive.dimensions[0] = hexagon_dim[0];
                        primitive.dimensions[1] = hexagon_dim[1];
                        object_pose.position.z -= hexagon_dim[0] / 2;
                        yaw_module = 2*M_PI/6;
                        break;
                    case 2:
                        primitive.type = primitive.BOX;
                        primitive.dimensions.resize(3);
                        primitive.dimensions[0] = triangle_dim[0];
                        primitive.dimensions[1] = triangle_dim[1];
                        primitive.dimensions[2] = triangle_dim[2];
                        tf2::fromMsg(object_pose.orientation, triangle_orientation);
                        tf2::Matrix3x3(triangle_orientation).getRPY(roll, pitch, yaw);
                        triangle_orientation.setRPY(0, 0, yaw);
                        object_pose.orientation = tf2::toMsg(triangle_orientation);
                        yaw -= M_PI/2;
                        yaw_module = 2*M_PI/2;
                        pose_offset = triangle_dim[1]*0.25;
                        object_pose.position.x += cos(yaw)*pose_offset;
                        object_pose.position.y += sin(yaw)*pose_offset;
                        break;
                    case 3:
                        primitive.type = primitive.BOX;
                        primitive.dimensions.resize(3);
                        primitive.dimensions[0] = cube_dim[0];
                        primitive.dimensions[1] = cube_dim[0];
                        primitive.dimensions[2] = cube_dim[0];
                        object_pose.position.z -= cube_dim[0] / 2;
                        yaw_module = 2*M_PI/4;
                        break;
                    default:
                        primitive.type = primitive.CYLINDER;
                        primitive.dimensions.resize(2);
                        primitive.dimensions[0] = golden_obstacle_dim[0];
                        primitive.dimensions[1] = golden_obstacle_dim[1];
                        object_pose.position.z -= golden_obstacle_dim[0] / 2;
                        yaw_module = 2*M_PI/6;
                        break;
                }

                if(currID == ID) {
                    target_pose = object_pose;
                    new_yaw = fmod(yaw, yaw_module);
                    target_pose.position.z -= 0.02;
                }

                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(object_pose);
                collision_object.operation = collision_object.ADD;

                remove_ids.push_back(to_string(currID));
                collision_objects.push_back(collision_object);
            }
             
            moveit_msgs::CollisionObject collision_object;
            shape_msgs::SolidPrimitive primitive;
            geometry_msgs::Pose object_pose;
            tf2::Quaternion orientation;

            collision_object.id = "table";
            collision_object.header.frame_id = "map";
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = table_dim[0] + 0.03;
            primitive.dimensions[1] = table_dim[1] + 0.03;
            primitive.dimensions[2] = table_pose[2] - 0.02; 
            
            object_pose.position.x = table_pose[0];
            object_pose.position.y = table_pose[1];
            object_pose.position.z = (table_dim[2] + table_pose[2]) / 2;
            orientation.setRPY(table_pose[3], table_pose[4], table_pose[5]);
            tf2::convert(orientation, object_pose.orientation);

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(object_pose);
            collision_object.operation = collision_object.ADD;

            remove_ids.push_back("table");
            collision_objects.push_back(collision_object);
            
            planning_scene_interface.applyCollisionObjects(collision_objects);
            
            tf2::fromMsg(target_pose.orientation, target_orientation);
            tf2::Matrix3x3(target_orientation).getRPY(roll, pitch, yaw);

            ROS_INFO("ID: %d, pos_x: %f, pos_y: %f, pos_z: %f, orient_x: %f, orient_y: %f, orient_z: %f, orient_w: %f, yaw_angle: %f, new_yaw: %f", 
				ID, target_pose.position.x, target_pose.position.y, target_pose.position.z,
				target_pose.orientation.x, target_pose.orientation.y,
				target_pose.orientation.z, target_pose.orientation.w, yaw/(2*M_PI)*360, new_yaw/(2*M_PI)*360);
            
            // -> Planning
            
            move_group.setPlanningTime(3.0);

            // --- MOVE 1: ARM (joint space) to SAFE POSITION (arm up next to face)

            safePosition(true, move_group, plan);

            // --- MOVE 2: ARM on top of target pose

            safe_pose = move_group.getCurrentPose();

            float gripper_offset = cos(M_PI/8) * gripper_lenght;
            float x_appr = cos(new_yaw) * gripper_offset;
            float y_appr = sin(new_yaw) * gripper_offset;
            
            target_orientation.setRPY(M_PI/2, M_PI/8, new_yaw);
            target_pose.position.x -= x_appr;
            target_pose.position.y -= y_appr;
            target_pose.position.z += 0.3;
            target_pose.orientation = tf2::toMsg(target_orientation);

            status = planMovement(move_group, plan, target_pose);
            if(!status) {
                planMovement(move_group, plan, safe_pose.pose);
                safePosition(false, move_group, plan);
                planning_scene_interface.removeCollisionObjects(remove_ids);
                result_.success = false;
                as_.setSucceeded(result_);
                return;
            }
           
            // --- MOVE 3: ARM in target pose to get object (after gripper opened)

            closeGripper(false); // open gripper

            approach_pose = target_pose;
            approach_pose.position.z -= 0.3;
            planCartesian(move_group, plan, approach_pose);
            ROS_INFO("Objection position reached");

            // --- MOVE 4: ARM back on top of target pose (after gripper closed and object attached)
            
            closeGripper(true); // close gripper

            // remove attached object as collision object
            vector<string> remove_vector {to_string(ID)};
            planning_scene_interface.removeCollisionObjects(remove_vector);

            attachGazebo(true, ID); // attach target object to arm
            ros::Duration(2.0).sleep();

            planMovement(move_group, plan, target_pose);

            // --- MOVE 5: ARM back to SAFE POSITION (arm up next to face)

            planMovement(move_group, plan, safe_pose.pose);

            safePosition(false, move_group, plan);

            result_.success = true;
            as_.setSucceeded(result_);

            // remove all collision objects
            while(planning_scene_interface.getObjects().size() != 0) {
                ROS_INFO("number of collision objects: %d", (int)planning_scene_interface.getObjects().size());
                planning_scene_interface.removeCollisionObjects(remove_ids);
            }
            ROS_INFO("Collision objects removed");
        }


        // ---------- PLACE ----------
        

        else {

            vector<moveit_msgs::CollisionObject> collision_objects;
            moveit_msgs::CollisionObject collision_object;
            shape_msgs::SolidPrimitive primitive;
            geometry_msgs::Pose object_pose, place_pose;
            tf2::Quaternion orientation;
            
            collision_object.id = "place_table";
            collision_object.header.frame_id = "map";
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = place_table_dim[0] + 0.03;
            primitive.dimensions[1] = place_table_dim[1] - 0.04;
            object_pose.position.x = placePoses[ID - 1][0];
            object_pose.position.y = placePoses[ID - 1][1];
            object_pose.position.z = place_table_dim[0] / 2;
            orientation.setRPY(0, 0, 0);
            tf2::convert(orientation, object_pose.orientation);

            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(object_pose);
            collision_object.operation = collision_object.ADD;

            collision_objects.push_back(collision_object);
            planning_scene_interface.applyCollisionObjects(collision_objects);

            // -> Planning

            safePosition(true, move_group, plan);
            safe_pose = move_group.getCurrentPose();

            // --- MOVE 6: ARM on top of place pose

            float gripper_offset = cos(M_PI/8) * gripper_lenght;
            float x_appr = cos(M_PI/4) * gripper_offset;
            float y_appr = sin(M_PI/4) * gripper_offset;
            target_orientation.setRPY(M_PI/2, M_PI/8, M_PI/4);

            place_pose.position.x = 0.8 - x_appr;
            place_pose.position.y = - y_appr;
            place_pose.position.z = place_table_dim[0] - 0.04 + 0.3;

            place_pose.orientation = tf2::toMsg(target_orientation);

            planMovement(move_group, plan, place_pose);

            // --- MOVE 7: ARM in place pose to release object

            approach_pose = place_pose;
            approach_pose.position.z -= 0.3;
            planCartesian(move_group, plan, approach_pose);

            // --- MOVE 8: ARM back on top of target pose (after object detached and gripper opened)

            closeGripper(false); // open gripper
            attachGazebo(false, ID); // detach target object to arm
            ros::Duration(2.0).sleep();

            planCartesian(move_group, plan, place_pose);
            
            // --- MOVE 9: ARM back to SAFE POSITION (arm up next to face)

            planMovement(move_group, plan, safe_pose.pose);
            safePosition(false, move_group, plan);
            
            result_.success = true;
            as_.setSucceeded(result_);
        }

        // remove cylinder place table from collision objects
        vector<string> remove_vector {"place_table"};
        while(planning_scene_interface.getObjects().size() != 0) {
            ROS_INFO("number of collision objects: %d", (int)planning_scene_interface.getObjects().size());
            planning_scene_interface.removeCollisionObjects(remove_vector);
        }
    }
};


// __________ MAIN __________


int main(int argc, char** argv) {
    // instantiate node
    ros::init(argc, argv, "nodeC");
    // instatiating class object that will act as action server
    NodeC nodeC_instance("/nodeC_topic");
    ROS_INFO("Ready to receive client requests");
    ros::spin();
    return 0;
}
