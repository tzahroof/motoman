#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/planning_scene/planning_scene.h>


  /*
  
  Function: addObstacles


  Purpose: Adds preset obstacle environments to the planning scene
  */


static void addObstacles(moveit::planning_interface::PlanningSceneInterface *planning_scene_interface, moveit_msgs::PlanningScene *planning_scene_msg, const std::string &environment)
{
  bool flag = false;
  if(environment == "box")
  {
    //SQUARE BOX
    //^^^^^^^^^^^
    moveit_msgs::CollisionObject collision_object;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.4;
    primitive.dimensions[2] = 0.4;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.35;
    box_pose.position.y = 0.1;
    box_pose.position.z = 0.9;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_object.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(collision_object);
    flag = true;
  }

  if(environment == "sphere")
  {
    //SPHERE OBJECT
    //^^^^^^^^^^^^^
    moveit_msgs::CollisionObject sphere_object;

    shape_msgs::SolidPrimitive sphere_primitive;
    sphere_primitive.type = sphere_primitive.SPHERE;
    sphere_primitive.dimensions.resize(1);
    sphere_primitive.dimensions[0] = 0.1;

    geometry_msgs::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = 0.35;
    sphere_pose.position.y = 0.1;
    sphere_pose.position.z = 0.9;

    sphere_object.primitives.push_back(sphere_primitive);
    sphere_object.primitive_poses.push_back(sphere_pose);
    sphere_object.operation = sphere_object.ADD;
    sphere_object.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(sphere_object);
    flag = true;
  }

  if(environment == "clutter")
  {
    //MULTIPLE OBJECTS
    //^^^^^^^^^^^^^
    moveit_msgs::CollisionObject sphere_object;
    sphere_object.id="sphere_object";
    shape_msgs::SolidPrimitive sphere_primitive;
    sphere_primitive.type = sphere_primitive.SPHERE;
    sphere_primitive.dimensions.resize(1);
    sphere_primitive.dimensions[0] = 0.1;

    geometry_msgs::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = 0.35;
    sphere_pose.position.y = 0.1;
    sphere_pose.position.z = 1.05;

    sphere_object.primitives.push_back(sphere_primitive);
    sphere_object.primitive_poses.push_back(sphere_pose);
    sphere_object.operation = sphere_object.ADD;
    sphere_object.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(sphere_object);



    moveit_msgs::CollisionObject sphere_object2;
    sphere_object2.id="sphere_object2";
    shape_msgs::SolidPrimitive sphere_primitive2;
    sphere_primitive2.type = sphere_primitive2.SPHERE;
    sphere_primitive2.dimensions.resize(1);
    sphere_primitive2.dimensions[0] = 0.1;

    geometry_msgs::Pose sphere_pose2;
    sphere_pose2.orientation.w = 1.0;
    sphere_pose2.position.x = 0.15;
    sphere_pose2.position.y = -0.3;
    sphere_pose2.position.z = 0.9;

    sphere_object2.primitives.push_back(sphere_primitive2);
    sphere_object2.primitive_poses.push_back(sphere_pose2);
    sphere_object2.operation = sphere_object2.ADD;
    sphere_object2.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(sphere_object2);


    moveit_msgs::CollisionObject sphere_object3;
    sphere_object3.id="sphere_object3";
    shape_msgs::SolidPrimitive sphere_primitive3;
    sphere_primitive3.type = sphere_primitive3.SPHERE;
    sphere_primitive3.dimensions.resize(1);
    sphere_primitive3.dimensions[0] = 0.1;

    geometry_msgs::Pose sphere_pose3;
    sphere_pose3.orientation.w = 1.0;
    sphere_pose3.position.x = 0.0;
    sphere_pose3.position.y = 0.28;
    sphere_pose3.position.z = 1.5;

    sphere_object3.primitives.push_back(sphere_primitive3);
    sphere_object3.primitive_poses.push_back(sphere_pose3);
    sphere_object3.operation = sphere_object3.ADD;
    sphere_object3.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(sphere_object3);

    moveit_msgs::CollisionObject sphere_object4;
    sphere_object4.id="sphere_object4";
    shape_msgs::SolidPrimitive sphere_primitive4;
    sphere_primitive4.type = sphere_primitive4.SPHERE;
    sphere_primitive4.dimensions.resize(1);
    sphere_primitive4.dimensions[0] = 0.1;

    geometry_msgs::Pose sphere_pose4;
    sphere_pose4.orientation.w = 1.0;
    sphere_pose4.position.x = 0.0;
    sphere_pose4.position.y = 0.4;
    sphere_pose4.position.z = 0.3;

    sphere_object4.primitives.push_back(sphere_primitive4);
    sphere_object4.primitive_poses.push_back(sphere_pose4);
    sphere_object4.operation = sphere_object4.ADD;
    sphere_object4.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(sphere_object4);


    moveit_msgs::CollisionObject collision_object;
    collision_object.id="collision_object";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.5;
    primitive.dimensions[2] = 0.05;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.4;
    box_pose.position.y = 0.15;
    box_pose.position.z = 0.83;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    collision_object.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(collision_object);

    moveit_msgs::CollisionObject collision_object1;
    collision_object1.id="collision_object1";
    shape_msgs::SolidPrimitive primitive1;
    primitive1.type = primitive1.BOX;
    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 0.15;
    primitive1.dimensions[1] = 0.15;
    primitive1.dimensions[2] = 0.15;

    geometry_msgs::Pose box_pose1;
    box_pose1.orientation.w = 1.0;
    box_pose1.position.x = 0.0;
    box_pose1.position.y = 0.45;
    box_pose1.position.z = 0.85;

    collision_object1.primitives.push_back(primitive1);
    collision_object1.primitive_poses.push_back(box_pose1);
    collision_object1.operation = collision_object1.ADD;
    collision_object1.header.frame_id = "/base_link";
    planning_scene_msg->world.collision_objects.push_back(collision_object1);


    flag = true;
  }


  ROS_INFO("Adding the object into the world");
  if(flag)
  {
	planning_scene_interface->applyCollisionObjects(planning_scene_msg->world.collision_objects);
  }


  ros::Duration(1).sleep();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "motoman");
	ros::NodeHandle node_handle("~");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//SETUP

	static const std::string PLANNING_GROUP = "manipulator";

	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	//Visualization

	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
	visual_tools.deleteAllMarkers();

	visual_tools.loadRemoteControl();

	visual_tools.trigger();

	//Getting Basic Information
	ROS_INFO_NAMED("fmt.cpp", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("fmt.cpp", "End Effector Link: %s", move_group.getEndEffectorLink().c_str());

	int max_Iter;
	double allowed_planning_time;
	std::string environment = "no_environment";

	if(!node_handle.getParam("max_Iter",max_Iter))
	{
		ROS_INFO_STREAM("No max_Iter parameter specified. Defaulting to 50");
		max_Iter = 50;
	}
	if(!node_handle.getParam("allowed_planning_time",allowed_planning_time))
	{
		ROS_INFO_STREAM("No allowed_planning_time parameter specified. Defaulting to 3");
		allowed_planning_time = 3.0;
	}
	if(!node_handle.getParam("environment",environment))
	{
		ROS_INFO_STREAM("No collision environment loaded");
	}

	//Start demo
	ros::Duration(2).sleep();

	moveit_msgs::PlanningScene planning_scene_msg;
	addObstacles(&planning_scene_interface,&planning_scene_msg, environment);
	move_group.setPlanningTime(allowed_planning_time);

	for(int main_loop_iter = 0; main_loop_iter < max_Iter; main_loop_iter++)
	{
		robot_state::RobotState start_state = *(move_group.getCurrentState());
		start_state.setToDefaultValues(start_state.getJointModelGroup(PLANNING_GROUP),"home");
		move_group.setStartState(start_state);


	    robot_state::RobotState goal_state = *(move_group.getCurrentState());
        goal_state.setToDefaultValues(goal_state.getJointModelGroup(PLANNING_GROUP),"goal_state");
		move_group.setJointValueTarget(goal_state);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO_NAMED("motoman", "Visualizing plan %d :: %s", main_loop_iter, success?"Success":"Failed");

		//Visualizing plans
		//^^^^^^^^^^^^^^^^
		visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
		visual_tools.trigger();
		visual_tools.prompt("Press 'next' in the RVizVisualToolsGUI window to continue");
	}
}