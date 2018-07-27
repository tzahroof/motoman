
/*
Motion_plannning_api_tutorial.cpp headers
*/

#include <chrono>
#include <complex>
#include <math.h> 

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

//#include <boost/scoped_ptr.h>

#include <moveit/collision_distance_field/collision_detector_hybrid_plugin_loader.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/robot_state/conversions.h>

//for writing files
#include <iostream>
#include <fstream>


/* GLOBAL VARIABLES */



/*Start Code */

static double determineCost(trajectory_msgs::JointTrajectory *joint_trajectory)
{

    double cost = 0;
    std::vector<int>::size_type numJointsTariq = joint_trajectory->points[0].positions.size();
    std::vector<int>::size_type size1 = joint_trajectory->points.size();

    for(unsigned j = 0; j < numJointsTariq; j++) {
      double costOfCurrentMovement = 0;
      for(unsigned iter = 0; iter < size1-1; iter++) {
            costOfCurrentMovement += 
              fabs((joint_trajectory->points[iter+1].positions[j] - joint_trajectory->points[iter].positions[j]));
      }
      cost += sqrt(costOfCurrentMovement);
    }
    return cost;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "motoman");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  const int max_Iter = 50;
  int numStompFails = 0;
  int numSeedFails = 0;
  double avgTime = 0.0;

 // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel and a
  // PlanningScene. We will start by instantiating a `RobotModelLoader`_
  // object, which will look up the robot description on the ROS
  // parameter server and construct a :moveit_core:`RobotModel` for us
  // to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    
    const std::string PLANNING_GROUP = "manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name; //"/ompl_interface/OMPLPlanner";



    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    

    //Ensure that the marker array panel (subscribed to /rviz_visual_tools) is active

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");


    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "This is a test to display text", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    /* Sleep a little to allow time to startup rviz, etc..
       This ensures that visual_tools.prompt() isn't lost in a sea of logs*/


    ros::Publisher display_publisher_stomp =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path_stomp", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory_stomp;

    ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ros::Publisher planning_scene_diff_publisher = 
      node_handle.advertise<moveit_msgs::PlanningScene>("/motoman/planning_scene",1);

    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ROS_INFO_STREAM("Waiting to Connect to Planning Scene topic /motoman/planning_scene");
      ros::Duration(0.5).sleep();
    }

    ros::Duration(5).sleep();

    /*
      The following adds a collision obstacle into the environment for navigational challenge
    */

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

    ROS_INFO("Adding the object into the world");
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.world.collision_objects.push_back(collision_object);
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);

    planning_scene->processCollisionObjectMsg(collision_object);

    ros::Duration(2).sleep();




  for(int main_loop_iter = 0; main_loop_iter <max_Iter; main_loop_iter++) { //change number of iterations

      double tempTime = 0.0;
      // Pose Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.

        // We will get the name of planning plugin we want to loadix
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!node_handle.getParam("sbp_plugin", planner_plugin_name)) //TODO: fix this to sbp_plugin
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }



    ROS_INFO("About to move into the Planning section");
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = PLANNING_GROUP;
    if(!node_handle.getParam("seed_allowed_planning_time", req.allowed_planning_time))
    {
      ROS_ERROR_STREAM("Could not find the seed allowed planning time parameter. Defaulting to 5s");
      req.allowed_planning_time = 5;
    }

    //UNCOMMENT following to use EE pose. Said method does NOT work for BFMT, FMT, or STOMP
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // geometry_msgs::PoseStamped pose;

    // //PoseStamped apparently uses quaternion to avoid singularities (orientation)
    // pose.header.frame_id = "/base_link";
    // pose.pose.position.x = 1.3306;
    // pose.pose.position.y = 2;
    // pose.pose.position.z = 3.64;
    // pose.pose.orientation.w = 1.0;

    //   // A tolerance of 0.01 m is specified in position
    // // and 0.01 radians in orientation
    // std::vector<double> tolerance_pose(3, 0.01);
    // std::vector<double> tolerance_angle(3, 0.01);

    //   // We will create the request as a constraint using a helper function available
    // // from the
    // // `kinematic_constraints`_
    // // package.
    // //
    // // .. _kinematic_constraints:
    // //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html
    // moveit_msgs::Constraints pose_goal =
    //     kinematic_constraints::constructGoalConstraints("armLink7square", pose, tolerance_pose, tolerance_angle);
    // req.goal_constraints.push_back(pose_goal);


    //Set the start state of the robot
    robot_state::RobotState start_state(robot_model);
    start_state.setToDefaultValues(start_state.getJointModelGroup(PLANNING_GROUP),"home");
    moveit::core::robotStateToRobotStateMsg(start_state,req.start_state);

   //UNCOMMENT Code if you want to set goal state from joint values instead of from SRDF file
   //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   // robot_state::RobotState goal_state_fmt(robot_model);
   // std::vector<double> joint_values = {-2.0 , 1.05 , 1.30 , -1.0 , -1.9 , 2.1 , 0.0 };
   // goal_state_fmt.setJointGroupPositions(joint_model_group, joint_values);
   
    robot_state::RobotState goal_state_fmt(robot_model);
    goal_state_fmt.setToDefaultValues(goal_state_fmt.getJointModelGroup(PLANNING_GROUP),"goal_state");

    moveit_msgs::Constraints joint_goal_fmt = kinematic_constraints::constructGoalConstraints(goal_state_fmt, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal_fmt);

    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);

    bool continueToSTOMP = true;

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      continueToSTOMP = false;
      numSeedFails++;
      ROS_ERROR("Could not compute plan successfully");
    }


    if(continueToSTOMP) 
    {
      // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^


    /* Visualize the trajectory */
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);


    tempTime += response.planning_time;


    // UNCOMMENT the following if you wish to display seed trajectory
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // display_trajectory.trajectory_start = response.trajectory_start;
    // display_trajectory.trajectory.push_back(response.trajectory);
    // display_publisher.publish(display_trajectory);


    //Displays the Cost
    ROS_INFO("Seed Cost :: %f",determineCost(&(response.trajectory.joint_trajectory)));

    
    /* We can also use visual_tools to wait for user input */
   // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");



    /////////////////////////
    //SECOND PART of the Code 
    /////////////////////////


    //Load the STOMP planner

    if (!node_handle.getParam("stomp_plugin", planner_plugin_name)) 
      ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
        ROS_FATAL_STREAM("Could not initialize planner instance");
      ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (std::size_t i = 0; i < classes.size(); ++i)
        ss << classes[i] << " ";
      ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                           << "Available plugins: " << ss.str());
    }

  //UNCOMMENT following code to use the Hybrid COllision Detector
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //planning_scene->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create(), true);
    
    if(!node_handle.getParam("stomp_allowed_planning_time",req.allowed_planning_time))
    {
      ROS_ERROR_STREAM("No default STOMP Planning Time provided. Defaulting to 5s");
      req.allowed_planning_time = 5;
    }

    // Joint Space Goals
    // ^^^^^^^^^^^^^^^^^
    //Set the start state of the robot again
    moveit::core::robotStateToRobotStateMsg(start_state,req.start_state);

    //Now, setup a joint space goal. Note: the goal state is obtained from the goal used by the seed trajectory
    robot_state::RobotState goal_state(robot_model);
    goal_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal);



    // The following seeds the previously generated trajectory into trajectory_constraints. trajectory_constraints
    // is not actually used by MoveIt!, so STOMP uses the variable as a way to seed in an initial trajectory. Since
    // MoveIt! is a geometric planner, it just needs the waypoints (no need for accelerations or velocities). The 
    // following code converts the BFMT* plan from a JointTrajectory to a Trajectory_Constraintsbrockhamp

    trajectory_msgs::JointTrajectory seed = response.trajectory.joint_trajectory;
    const auto dof = seed.joint_names.size();

    for (size_t i = 0; i < seed.points.size(); ++i) // for each time step
    {
      moveit_msgs::Constraints c;

      if (seed.points[i].positions.size() != dof)
        throw std::runtime_error("All trajectory position fields must have same dimensions as joint_names");

      for (size_t j = 0; j < dof; ++j) // for each joint
      {
        moveit_msgs::JointConstraint jc;
        jc.joint_name = seed.joint_names[j];
        jc.position = seed.points[i].positions[j];

        c.joint_constraints.push_back(jc);
      }
      req.trajectory_constraints.constraints.push_back(std::move(c));
    }


    context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    /* Call the Planner */
    context->solve(res);
    /* Check that the planning was successful */
    bool stompPassed = true;
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully");
      stompPassed = false;
    }

    /* Visualize the trajectory */

    if(stompPassed) 
    {
      // Uncomment following if you want to see successful STOMP seeds. Be sure to change the path
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // std::string path = "/home/tariq/Documents/STOMPRecords.m";
      // std::ofstream outputFile;
      // outputFile.open(path,std::ios::app);
      // outputFile << "STOMPSucc{" +std::to_string(main_loop_iter- numStompFails) + "} = [";
      // for(unsigned iter = 0; iter < size1; iter++) { //goes through all points 
      //   for(unsigned j = 0; j < numJointsTariq; j++) {
      //     outputFile << response.trajectory.joint_trajectory.points[iter].positions[j];
      //     outputFile << " ";
      //   }
      //   if(iter+1 != size1){
      //     outputFile << ";" <<std::endl;
      //   } else {
      //     outputFile << "]" << std::endl;
      //   }
      // }
      // outputFile << std::endl;
      // outputFile.close();
      // ROS_INFO_STREAM("Finished creating log file");




      /* Visualize the trajectory */
      ROS_INFO("Visualizing the trajectory");
      res.getMessage(response);

      avgTime += tempTime + response.planning_time;


      display_trajectory_stomp.trajectory_start = response.trajectory_start;
      display_trajectory_stomp.trajectory.push_back(response.trajectory);
      display_publisher_stomp.publish(display_trajectory_stomp);




      ROS_INFO("STOMP Cost :: %f",determineCost(&(response.trajectory.joint_trajectory)));


    } else {

      numStompFails++;
      // UNCOMMENT following code to see which STOMP seeds failed. Be sure to change the path.
      //  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // std::string path = "/home/tariq/Documents/STOMPRecords.m";
      // std::ofstream outputFile;
      // outputFile.open(path,std::ios::app);
      // outputFile << "STOMPFail{" +std::to_string(numStompFails) + "} = [";
      // for(unsigned iter = 0; iter < size1; iter++) { //goes through all points 
      //   for(unsigned j = 0; j < numJointsTariq; j++) {
      //     outputFile << response.trajectory.joint_trajectory.points[iter].positions[j];
      //     outputFile << " ";
      //   }
      //   if(iter+1 != size1){
      //     outputFile << ";" <<std::endl;
      //   } else {
      //     outputFile << "]" << std::endl;
      //   }
      // }
      // outputFile << std::endl;
      // outputFile.close();
      // ROS_ERROR_STREAM("Finished creating log file");
      // ROS_INFO_STREAM("number of STOMP fails currently");
      // ROS_INFO_STREAM(numStompFails);
      // ROS_INFO_STREAM("Publishing Sampling Based Planning trajectory without smoothing");
      // rqt_publisher.publish(response.trajectory.joint_trajectory);
    }
  }

    ROS_INFO_STREAM("Current iteration :: ");
    ROS_INFO_STREAM(main_loop_iter);

  }

  avgTime = avgTime/(max_Iter-numStompFails);
  ROS_INFO_STREAM("Average Time ::");
  ROS_INFO_STREAM(avgTime);
  ROS_INFO_STREAM("Number of STOMP failures");
  ROS_INFO_STREAM(numStompFails);
  ROS_INFO_STREAM("Number of SEED failures");
  ROS_INFO_STREAM(numSeedFails);

}

