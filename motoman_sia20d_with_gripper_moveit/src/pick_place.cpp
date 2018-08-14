/*
Motion_plannning_api_tutorial.cpp headers
*/

#include <chrono>
#include <complex>
#include <math.h> 

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

/*
  This file includes all of the shortcut code
*/
#include <motoman/motoman_sia20d_with_gripper_moveit/fmt_shortcut.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

//for time parameterization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//for writing files
#include <iostream>
#include <fstream>


/*
  GLOBAL VARIABLES
*/

const std::string PLANNING_GROUP = "manipulator";
const std::string EEF_PLANNING_GROUP = "hand";
std::string environment = "";
double max_EdgeLength_Discretization = 0.1; //determines how big the maxEdgeLength should be for checking discretization
double max_EdgeLength_Waypoint_Injection = 0.2; //determines how big the maxEdgeLength should be for WayPoint injection during populate()
double allowed_planning_time = 5; //how long the planner is allowed to plan before we cut it off
static bool seedSuccess = false; //a global variable we use to check if the basic BFMT* managed to generate a plan given our start and goal states
static int numSeedFails = 0;
static int adaptive_repetitions =5;
static std::string shortcutMethod ="";
static int numShortcutLoops = 30;
static moveit_msgs::PlanningScene stateOfTheWorld;
static std::shared_ptr<shortcut::Shortcut_Planner> shortcut_planner;
static double avgSeedTime = 0.0;
static double avgSeedCost = 0.0;

/*
  Prototype Functions
*/
static double determineCost(trajectory_msgs::JointTrajectory *joint_trajectory);
static void visualizePlot(trajectory_msgs::JointTrajectory *joint_trajectory, ros::Publisher *rqt_publisher);
static double timeParameterize(moveit_msgs::MotionPlanResponse *response, robot_model::RobotModelPtr robot_model, robot_state::RobotState *start_state);
static void addObstacles(planning_scene::PlanningScenePtr planning_scene, ros::Publisher *planning_scene_diff_publisher, moveit_msgs::PlanningScene *planning_scene_msg, std::string environment);


static moveit_msgs::MotionPlanResponse solve( robot_state::RobotState *start_state, robot_state::RobotState *goal_state, planning_scene::PlanningScenePtr planning_scene, 
       planning_interface::PlannerManagerPtr planner_instance, robot_model::RobotModelPtr robot_model, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, const std::string which_group)
{
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    moveit_msgs::MotionPlanResponse response;
    req.group_name = which_group;
    req.allowed_planning_time = allowed_planning_time;
    double avgTime = 0.0;
    seedSuccess= true;


    //Explicitly sets start state
    //Start state explicitly set from SRDF file in config
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    moveit::core::robotStateToRobotStateMsg(*start_state,req.start_state);
   


    //Explicitly sets goal state
    //^^^^^^^^^^^^^^^^^^^^^^^^^^
    moveit_msgs::Constraints joint_goal_fmt = kinematic_constraints::constructGoalConstraints(*goal_state, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal_fmt);


    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      numSeedFails++;
      seedSuccess = false;
      ROS_ERROR("Could not compute plan successfully");
    }


    if(seedSuccess)
    {
      res.getMessage(response);
 
      avgTime += response.planning_time;
      avgSeedTime += response.planning_time;
      avgSeedCost += determineCost(&(response.trajectory.joint_trajectory));

      //Displays the Cost
      ROS_INFO_STREAM("BFMT Cost :: " + std::to_string(determineCost(&(response.trajectory.joint_trajectory))));


      //Uncomment the following code to make the simulation require an input before continuing
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //visual_tools.prompt("Please press next in the RvizVisualToolsGui for Shortcut Algorithm. Make sure to add the button via the Panels menu in the top bar.");


    /////////////////////////
    //SECOND PART of the Code 
    /////////////////////////


      // Send the trajectory to the Shortcut Method to prune unnecessary movements
      ROS_INFO_STREAM("Pre-processing size :: " + std::to_string(response.trajectory.joint_trajectory.points.size()));
      if(shortcutMethod == "Regular")
      {
        avgTime += shortcut_planner->RegularShortcut(&(response.trajectory.joint_trajectory),&planning_scene,joint_model_group,&(*robot_state),30,3);
      }
      else if(shortcutMethod == "AdaptivePartial")
      {
        avgTime += shortcut_planner->PartialShortcut(&(response.trajectory.joint_trajectory), &planning_scene, joint_model_group, &(*robot_state), numShortcutLoops);
      }
      else if (shortcutMethod == "Adaptive")
      {
        avgTime += shortcut_planner->AdaptiveShortcut(&(response.trajectory.joint_trajectory),&planning_scene,joint_model_group,&(*robot_state),numShortcutLoops,3);
      }
      else if (shortcutMethod == "Partial")
      {
        avgTime += shortcut_planner->PartialShortcut(&(response.trajectory.joint_trajectory), &planning_scene, joint_model_group, &(*robot_state), numShortcutLoops);
      }
      


      //Time parameterize said trajectory again (waypoints were removed and added from previous method)
      if(response.trajectory.joint_trajectory.points.size() >2)
      {
        double tempTime = timeParameterize(&response, robot_model, start_state);
        if(tempTime !=-1) //reprsents that time parameterization found a solution
        {
          avgTime += tempTime;
        }
      }
      response.planning_time = avgTime;
      ROS_INFO_STREAM("Final Plan Cost :: " + std::to_string(determineCost(&(response.trajectory.joint_trajectory))));

    }

    return response;

}

/*

Function: main()


Purpose: This is where the magic happens. Most of the methods are either helper methods or ways to implement
         the shortcut algorithm (which is not natively built into MoveIt!) Check out the motion planning api
         tutorial of MoveIt! online for more information on the basics.


*/


int main(int argc, char** argv) {
  ros::init(argc, argv, "motoman");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");
  int max_Iter = 1; //change this to alter the number of simulations
  double avgTime = 0.0;
  double avgCost = 0.0;

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


    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    const robot_state::JointModelGroup* eef_joint_model_group = robot_state->getJointModelGroup(EEF_PLANNING_GROUP);

    /* Create an IterativeParabolicTimeParameterization object to add velocity/acceleration to waypoints after Shortcut*/

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    //Set up a publisher to advertise the JointTrajectories to the graphing tool
	  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher rqt_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/rqt_publisher/", 1);
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/motoman/planning_scene",1);


    moveit_msgs::DisplayTrajectory display_trajectory;
    moveit_msgs::MotionPlanResponse response_main;

    
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ros::Duration(0.5).sleep();
    }
   

    if(!node_handle.getParam("max_Iter", max_Iter))
    {
      ROS_INFO_STREAM("No max_Iter value provided. Defaulting to 50 iterations instead");
    }
   
    if(!node_handle.getParam("allowed_planning_time", allowed_planning_time))
    {
      ROS_ERROR_STREAM("Could not find the allowed_planning_time parameter");
    }
    if(!node_handle.getParam("environment",environment))
    {
      ROS_INFO_STREAM("No obstacle environment specified");
    }
    if(!node_handle.getParam("numShortcutLoops",numShortcutLoops))
    {
      numShortcutLoops = 30;
      ROS_INFO_STREAM("No numShortcutLoops provided. Defaulting to 30");
    }
    if(!node_handle.getParam("shortcut", shortcutMethod))
    {
      shortcutMethod = "AdaptivePartial";
      ROS_INFO_STREAM("No Shortcut Method provided. Defaulting to Adaptive Shortcut instead");
    }

    if(shortcutMethod== "Regular" || shortcutMethod == "Adaptive")
    {
      if(!node_handle.getParam("max_EdgeLength_Discretization",max_EdgeLength_Discretization))
      {
        ROS_ERROR_STREAM("Could not load the max_EdgeLength_Discretization Parameter");
      }

      if(!node_handle.getParam("max_EdgeLength_Waypoint_Injection", max_EdgeLength_Waypoint_Injection))
      {
        ROS_ERROR_STREAM("Could not load the max_EdgeLength_Waypoint_Injection Parameter");
      }

      if(shortcutMethod == "Adaptive")
      {
        if(!node_handle.getParam("adaptive_repetitions", adaptive_repetitions))
        {
          ROS_ERROR_STREAM("Could not load the adaptive_repetitions Parameter");
        }
        shortcut_planner.reset(new shortcut::Shortcut_Planner(shortcutMethod, PLANNING_GROUP, allowed_planning_time, max_EdgeLength_Discretization, max_EdgeLength_Waypoint_Injection,adaptive_repetitions));
      } else 
      {
        shortcut_planner.reset(new shortcut::Shortcut_Planner(shortcutMethod, PLANNING_GROUP, allowed_planning_time, max_EdgeLength_Discretization, max_EdgeLength_Waypoint_Injection));
      }      
    } else 
    {
      shortcut_planner.reset(new shortcut::Shortcut_Planner(shortcutMethod, PLANNING_GROUP, allowed_planning_time));
    }


    // Visualization
    // ^^^^^^^^^^^^^
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script


    //Ensure that the marker array panel (subscribed to /rviz_visual_tools) is active

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script
       via buttons and keyboard shortcuts in RViz
       Currently, no remote control is being used */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "This is a test to display text", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();


    /* Sleep a little to allow time to startup rviz, etc..
       This ensures that visual_tools.prompt() isn't lost in a sea of logs*/
    ROS_INFO_STREAM("About to enter 5 seconds of sleep to let startup occur properly");
    ros::Duration(5).sleep();

    moveit_msgs::PlanningScene planning_scene_msg;
    addObstacles(planning_scene,&planning_scene_diff_publisher,&planning_scene_msg, environment );

   // UNCOMMENT the following line  to only start the program upon button press
   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo. Make sure to add the button via the Panels menu in the top bar.");

  for(int main_loop_iter = 0; main_loop_iter <max_Iter; main_loop_iter++) { //change number of iterations


    double timeTaken = 0.0;
    double costOfPlan = 0.0;
    // Joint Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.

    // We will get the name of planning plugin we want to loadix
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.

    // Due to the way that FMT* is programmed, it needs an explicit joint goal (the other OMPL planners can infer one from an end-effector pose)
    // BFMT* requires an explicit start state as well

    //CREATE OBJECT for PICKING UP

    moveit_msgs::CollisionObject pick_object;

    shape_msgs::SolidPrimitive pick_primitive;
    pick_primitive.type = pick_primitive.BOX;
    pick_primitive.dimensions.resize(3);
    pick_primitive.dimensions[0] = 0.035;
    pick_primitive.dimensions[1] = 0.1;
    pick_primitive.dimensions[2] = 0.15;

    geometry_msgs::Pose pick_box_pose;
    pick_box_pose.orientation.w = 1.0;
    pick_box_pose.position.x = 0.4575;
    pick_box_pose.position.y = 0.580;
    pick_box_pose.position.z = 0.075;

    pick_object.id = "pick_object";
    pick_object.primitives.push_back(pick_primitive);
    pick_object.primitive_poses.push_back(pick_box_pose);
    pick_object.operation = pick_object.ADD;
    pick_object.header.frame_id = "/base_link";

    //Display the object to pick-up in RViz
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(pick_object);
    planning_scene_msg.robot_state.attached_collision_objects.clear();
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);


    ros::Duration(1).sleep();


    //Add the objects to our simulation and planning process
    planning_scene->processPlanningSceneWorldMsg(planning_scene_msg.world);

    if (!node_handle.getParam("sbp_plugin", planner_plugin_name)) //sbp_plugin is the ompl library from the launch file
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

    
    /////////////////
    //HOME TO PICK //
    /////////////////


    //Start state explicitly set from SRDF file in config
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    robot_state::RobotState start_state(robot_model);
    start_state.setToDefaultValues(start_state.getJointModelGroup(EEF_PLANNING_GROUP), "open");
    start_state.setToDefaultValues(start_state.getJointModelGroup(PLANNING_GROUP),"home");

    //Displays the starting state in MoveIt! via display_robot_state topic
    planning_scene_msg.world.collision_objects.clear();
    moveit::core::robotStateToRobotStateMsg(start_state,planning_scene_msg.robot_state);
    planning_scene_msg.robot_state.is_diff = true;
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);


    //Close Pick state explicitly set from SRDF file in config
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    robot_state::RobotState close_pick_state(robot_model);
    close_pick_state.setToDefaultValues(close_pick_state.getJointModelGroup(EEF_PLANNING_GROUP), "open");
    close_pick_state.setToDefaultValues(close_pick_state.getJointModelGroup(PLANNING_GROUP),"close_pick_state");

    //Solve the plan using the BFMT+Shortcut algorithm
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    do{
      response_main = solve(&start_state, &close_pick_state,planning_scene,planner_instance,robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    //UNCOMMENT following line if you wish to make code stop here until a button press is received.
    visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");


    ////////////////
    // PICK STATE //
    ////////////////


    robot_state::RobotState ungrasped_pick_state(robot_model);
    ungrasped_pick_state.setToDefaultValues(ungrasped_pick_state.getJointModelGroup(EEF_PLANNING_GROUP), "open");
    ungrasped_pick_state.setToDefaultValues(ungrasped_pick_state.getJointModelGroup(PLANNING_GROUP),"pick_state");


    do {
      response_main = solve(&close_pick_state, &ungrasped_pick_state, planning_scene, planner_instance, robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    planning_scene_msg.world.collision_objects.clear();
    moveit::core::robotStateToRobotStateMsg(ungrasped_pick_state,planning_scene_msg.robot_state);
    planning_scene_msg.robot_state.is_diff = true;
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);


    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    //Uncomment the following code to make the simulation require an input before continuing
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Please press next in the RvizVisualToolsGui for Placement. Make sure to add the button via the Panels menu in the top bar.");



    /////////////////
    //CLOSE GRIPPER//
    /////////////////

    robot_state::RobotState grasped_pick_state(robot_model);
    grasped_pick_state.setToDefaultValues(grasped_pick_state.getJointModelGroup(EEF_PLANNING_GROUP), "closed");
    grasped_pick_state.setToDefaultValues(grasped_pick_state.getJointModelGroup(PLANNING_GROUP),"pick_state");

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "sia20d_leftfinger";
    attached_object.touch_links = {"sia20d_leftfinger", "sia20d_rightfinger","sia20d_hand"};
    attached_object.object = pick_object;


    moveit_msgs::CollisionObject remove_object;
    remove_object.id="pick_object";
    remove_object.header.frame_id = "/base_link";
    remove_object.operation = remove_object.REMOVE;

    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(remove_object);
    moveit::core::robotStateToRobotStateMsg(grasped_pick_state,planning_scene_msg.robot_state);
    planning_scene_msg.robot_state.is_diff = true;
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene_msg.is_diff = true;

    planning_scene->processCollisionObjectMsg(remove_object);
    planning_scene->processAttachedCollisionObjectMsg(attached_object);

    do{
      response_main = solve(&grasped_pick_state,&grasped_pick_state,planning_scene,planner_instance,robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    planning_scene_diff_publisher.publish(planning_scene_msg);
    ros::Duration(0.5).sleep();


    //Uncomment the following code to make the simulation require an input before continuing
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    visual_tools.prompt("Please press next in the RvizVisualToolsGui for Placement. Make sure to add the button via the Panels menu in the top bar.");


    ////////////////////////
    //PICK TO CLOSE PLACE //
    ///////////////////////


    //Close Pick state explicitly set from SRDF file in config
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    robot_state::RobotState close_place_state(robot_model);
    close_place_state.setToDefaultValues(close_place_state.getJointModelGroup(EEF_PLANNING_GROUP), "closed");
    close_place_state.setToDefaultValues(close_place_state.getJointModelGroup(PLANNING_GROUP),"close_place_state");

    //Solve the plan using the BFMT+Shortcut algorithm
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    do{
      response_main = solve(&grasped_pick_state, &close_place_state,planning_scene,planner_instance,robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    planning_scene_msg.world.collision_objects.clear();
    moveit::core::robotStateToRobotStateMsg(close_place_state,planning_scene_msg.robot_state);
    planning_scene_msg.robot_state.is_diff = true;
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);


    //UNCOMMENT following line if you wish to make code stop here until a button press is received.
    visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");


    /////////////////////////
    //CLOSE PLACE TO PLACE///
    ////////////////////////

    robot_state::RobotState place_state(robot_model);
    place_state.setToDefaultValues(place_state.getJointModelGroup(EEF_PLANNING_GROUP), "closed");
    place_state.setToDefaultValues(place_state.getJointModelGroup(PLANNING_GROUP),"place_state");

    //Solve the plan using the BFMT+Shortcut algorithm
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    do{
      response_main = solve(&close_place_state, &place_state,planning_scene,planner_instance,robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");


    /////////////
    //DROP OFF///
    /////////////

    robot_state::RobotState released_place_state(robot_model);
    released_place_state.setToDefaultValues(released_place_state.getJointModelGroup(EEF_PLANNING_GROUP),"open");
    released_place_state.setToDefaultValues(released_place_state.getJointModelGroup(PLANNING_GROUP),"place_state");

    do{
      response_main = solve(&released_place_state, &released_place_state,planning_scene,planner_instance,robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    moveit_msgs::AttachedCollisionObject detach_object;
    detach_object.object.id = "pick_object";
    detach_object.link_name = "sia20d_leftfinger";
    detach_object.object.operation = moveit_msgs::CollisionObject::REMOVE;

    // geometry_msgs::Pose detached_box_pose;
    // detached_box_pose.orientation.w = 1.0;
    // detached_box_pose.position.x = 0.704;
    // detached_box_pose.position.y = -0.218;
    // detached_box_pose.position.z = 0.040;

    // attached_object.object.primitive_poses.clear();
    // attached_object.object.primitive_poses.push_back(detached_box_pose);

    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    moveit::core::robotStateToRobotStateMsg(released_place_state, planning_scene_msg.robot_state);
    planning_scene_msg.robot_state.attached_collision_objects.clear();
    planning_scene_msg.robot_state.attached_collision_objects.push_back(detach_object);
    planning_scene_diff_publisher.publish(planning_scene_msg);
    planning_scene_msg.robot_state.is_diff = true;
    planning_scene_msg.is_diff = true;

    planning_scene->processAttachedCollisionObjectMsg(detach_object);

    visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");

    //////////////
    //HOME //////
    ////////////

    do{
      response_main = solve(&released_place_state, &start_state,planning_scene,planner_instance,robot_model,joint_model_group,&(*robot_state),PLANNING_GROUP);
    } while(seedSuccess == false);

    timeTaken += response_main.planning_time;
    costOfPlan += determineCost(&(response_main.trajectory.joint_trajectory));

    display_trajectory.trajectory_start = response_main.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
    display_trajectory.trajectory.clear();
    display_trajectory.trajectory.push_back(response_main.trajectory);
    display_publisher.publish(display_trajectory);

    visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");


    avgTime += timeTaken;
    avgCost += costOfPlan;
    ROS_INFO_STREAM("Cost of Complete Plan :: " + std::to_string(costOfPlan)); 
    ROS_INFO_STREAM("Planning Time :: " + std::to_string(timeTaken));
    ROS_INFO_STREAM("Current iteration :: "+std::to_string(main_loop_iter));
  }

  avgTime = avgTime/(max_Iter);
  avgSeedCost = avgSeedCost/(max_Iter);
  avgSeedTime = avgSeedTime/(max_Iter);
  avgCost = avgCost/(max_Iter);
  ROS_INFO_STREAM("Average Seed Time :: " +std::to_string(avgSeedTime));
  ROS_INFO_STREAM("Average Full Plan Time :: " +std::to_string(avgTime));
  ROS_INFO_STREAM("Average Seed Cost :: " +std::to_string(avgSeedCost));
  ROS_INFO_STREAM("Average Full Plan Cost :: " +std::to_string(avgCost));
  ROS_INFO_STREAM("Number of SEED failures along the Process:: " +std::to_string(numSeedFails));

}


/*

Function: timeParameterize


Purpose: Time Parameterizes a trajectory to provide accelerations/velocities that fit the robot's requirements. Pass in response to have its 
         RobotTrajectory modified accordingly. Returns a double, which represents how long the time parameterization method took. 
         Returning -1 represents a failed solution.


*/

static double timeParameterize(moveit_msgs::MotionPlanResponse *response, robot_model::RobotModelPtr robot_model, robot_state::RobotState *start_state)
{
  double time = ros::Time::now().toSec();
  trajectory_processing::IterativeParabolicTimeParameterization iptp;


  robot_trajectory::RobotTrajectory rt(robot_model, PLANNING_GROUP);
  rt.setRobotTrajectoryMsg(*start_state, response->trajectory.joint_trajectory);
  bool time_par_suc = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp:: %s", time_par_suc?"SUCCESS":"FAILURE");
  if(time_par_suc)
  {
    rt.getRobotTrajectoryMsg(response->trajectory);
    return (ros::Time::now().toSec() - time);
  }

  return -1;

}

/*

Function: determineCost


Purpose: Simple function that prints out the cost of a trajectory. Takes the 2-norm of the total distance
         each joint travels


*/

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

  /*
  
  Function: visualizePlot


  Purpose: Publishes the trajectory to rqt_publisher (initialized in main method)
           Uncomment the code to add arbitrary time parameterization for ease-of-
           visualization via rqt plot plugin
  */

static void visualizePlot(trajectory_msgs::JointTrajectory *joint_trajectory, ros::Publisher *rqt_publisher)
{

    std::vector<int>::size_type size1 = joint_trajectory->points.size();

    // UNCOMMENT BELOW IF THERE IS NO ASSOCIATED TIME WITH THE TRAJECTORY
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // for(unsigned iter = 0; iter < size1; iter++)
    // {
    // 	joint_trajectory->points[iter].time_from_start = ros::Duration(0.1*iter);
    // }
    rqt_publisher->publish(*joint_trajectory);

}

/*
  Function: addObstacles()

  Purpose: Adds specified obstacles to the environment
*/

static void addObstacles(planning_scene::PlanningScenePtr planning_scene, ros::Publisher *planning_scene_diff_publisher, moveit_msgs::PlanningScene *planning_scene_msg, std::string environment)
{
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
    planning_scene->processCollisionObjectMsg(collision_object);
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
    planning_scene->processCollisionObjectMsg(sphere_object);
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
    planning_scene->processCollisionObjectMsg(sphere_object);



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
    planning_scene->processCollisionObjectMsg(sphere_object2);


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
    planning_scene->processCollisionObjectMsg(sphere_object3);

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
    planning_scene->processCollisionObjectMsg(sphere_object4);


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
    planning_scene->processCollisionObjectMsg(collision_object);

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
    planning_scene->processCollisionObjectMsg(collision_object1);
  }


  ROS_INFO("Adding the object into the world");
  
  planning_scene_msg->is_diff = true;
  planning_scene_diff_publisher->publish(*planning_scene_msg);


  ros::Duration(1).sleep();
}
