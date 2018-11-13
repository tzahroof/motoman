/*
Motion_plannning_api_tutorial.cpp headers
*/

#include <chrono>
#include <complex>
#include <math.h> 
#include <limits>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

/*
  THIS HEADER FILE has all of the shortcut code. 
*/
#include <motoman/motoman_sia20d_moveit_updated/fmt_shortcut.h>

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

//for time parameterization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//for Eigen and Shape Conversions
#include <eigen_conversions/eigen_msg.h>
#include <geometric_shapes/shape_operations.h>

//for writing files
#include <iostream>
#include <fstream>


/*
  GLOBAL VARIABLES
*/

const std::string PLANNING_GROUP = "manipulator";
double max_EdgeLength_Discretization; //determines how big the maxEdgeLength should be for checking discretization
double max_EdgeLength_Waypoint_Injection; //determines how big the maxEdgeLength should be for WayPoint injection during populate()
int defaultNumPoints = 5; //TODO: change this
int numShortcutLoops = 30;
int adaptive_repetitions = 5;
std::string shortcutMethod = "Partial";
double allowed_planning_time;
/*
  Prototype Functions
*/
static double determineCost(trajectory_msgs::JointTrajectory *joint_trajectory);
static void visualizePlot(trajectory_msgs::JointTrajectory *joint_trajectory, ros::Publisher *rqt_publisher);
static bool checkIfPathHasCollisions(moveit_msgs::MotionPlanResponse *response, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state);
static double timeParameterize(moveit_msgs::MotionPlanResponse *response, robot_model::RobotModelPtr robot_model, robot_state::RobotState *start_state);
static void addObstacles(planning_scene::PlanningScenePtr planning_scene, ros::Publisher *planning_scene_diff_publisher, moveit_msgs::PlanningScene *planning_scene_msg, std::string environment, ros::Publisher &vis_pub);

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
  std::string environment = "";
  moveit_msgs::MotionPlanResponse bestResponse;
  double bestCost = 100000;

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

    /* Create an IterativeParabolicTimeParameterization object to add velocity/acceleration to waypoints after Shortcut*/

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    moveit_msgs::PlanningScene planning_scene_msg;
    // We will now construct a loader to load a planner, by name.
    // Note that we are using the ROS pluginlib library here.
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;



    //Set up a publisher to advertise the JointTrajectories to the graphing tool
	  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher rqt_publisher = node_handle.advertise<trajectory_msgs::JointTrajectory>("/rqt_publisher/", 1);
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("/motoman/planning_scene",1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
      ros::Duration(0.5).sleep();
    }
    ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
    while(vis_pub.getNumSubscribers() < 1)
    {
      std::cout<<"Waiting for Marker to connect to /camera_navigation/visualization_marker"<<std::endl;
      ros::Duration(0.5).sleep();
    }

    if(!node_handle.getParam("max_Iter", max_Iter))
    {
      ROS_INFO_STREAM("No max_Iter value provided. Defaulting to 50 iterations instead");
    }
    if(!node_handle.getParam("shortcut", shortcutMethod))
    {
      shortcutMethod = "Partial";
      ROS_INFO_STREAM("No Shortcut Method provided. Defaulting to Partial Shortcut instead");
    }
    if(!node_handle.getParam("numShortcutLoops",numShortcutLoops))
    {
      numShortcutLoops = 30;
      ROS_INFO_STREAM("No numShortcutLoops provided. Defaulting to 30");
    }
    if(!node_handle.getParam("environment",environment))
    {
      ROS_INFO_STREAM("No obstacle environment specified");
    }
    if(!node_handle.getParam("allowed_planning_time", allowed_planning_time))
    {
      allowed_planning_time = 5;
      ROS_ERROR_STREAM("Could not find the allowed_planning_time parameter. Defaulting to 5");
    }


    addObstacles(planning_scene, &planning_scene_diff_publisher, &planning_scene_msg,environment,vis_pub);


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


// for (int gtts = 0; gtts < 4; gtts++)
// {
    /*
      Create a pointer for the shortcut planner. The shortcut planner determines which shortcut method to implement
    */
    int numSeedFails = 0;
    int numWeirdSeeds = 0;
    double avgTime = 0.0;
    double avgSeedTime = 0.0;
    double avgSeedCost = 0.0;
    double avgShortcutCost = 0.0;

    //TODO: Remove this section to prevent looping through all the shortcuts
    // switch(gtts)
    // {
    //   case 0:
    //     shortcutMethod = "Regular";
    //     break;
    //   case 1:
    //     shortcutMethod = "Adaptive";
    //     break;
    //   case 2:
    //     shortcutMethod = "Partial";
    //     break;
    //   case 3:
    //     shortcutMethod = "AdaptivePartial";
    //     break;
    //   default:
    //     std::cout<<"We should not be here! Failed looping through shortcuts"<<std::endl;
    // }

    std::shared_ptr<shortcut::Shortcut_Planner> shortcut_planner;

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


   // UNCOMMENT the following line  to only start the program upon button press
   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo. Make sure to add the button via the Panels menu in the top bar.");

  for(int main_loop_iter = 0; main_loop_iter <max_Iter; main_loop_iter++) { //change number of iterations

    // Joint Goal
    // ^^^^^^^^^
    // We will now create a motion plan request for the arm of the Panda
    // specifying the desired pose of the end-effector as input.

    // We will get the name of planning plugin we want to loadix
    // from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.

    // Due to the way that FMT* is programmed, it needs an explicit joint goal (the other OMPL planners can infer one from an end-effector pose)
    // BFMT* requires an explicit start state as well

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
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = PLANNING_GROUP;
    req.allowed_planning_time = allowed_planning_time;


    //UNCOMMENT the following code if a desired final end-effector position is known
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //  //NOTE: STOMP, FMT, and BFMT need JointState goals. look into the setApproximateJointState if
    //  //position is desired. The code will determine a joint state by inverse kinematics.
    // geometry_msgs::PoseStamped pose;

    // //PoseStamped apparently uses quaternion to avoid singularities (orientation)
    // pose.header.frame_id = "base_link";
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
    // //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
    // 
    // moveit_msgs::Constraints pose_goal =
    //     kinematic_constraints::constructGoalConstraints("armLink7square", pose, tolerance_pose, tolerance_angle);
    // // req.goal_constraints.push_back(pose_goal);  //TODO: change this



   //The following code is used if final joint values are known
   //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	  //Explicitly sets start state
    //Start state explicitly set from SRDF file in config
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    robot_state::RobotState start_state(robot_model);
    std::string start_name;
    std::string end_name;
    if(environment == "floating")
    {
      start_name = "start_state_float";
      end_name = "goal_state_float";
    }
    else
    {
      start_name = "start_state_satellite";
      end_name = "goal_state_satellite";
    }
    start_state.setToDefaultValues(start_state.getJointModelGroup(PLANNING_GROUP),start_name);
    moveit::core::robotStateToRobotStateMsg(start_state,req.start_state);
    start_state.updateCollisionBodyTransforms();
    //planning_scene->setCurrentState(start_state);

   
    //Displays the starting state in MoveIt! via display_robot_state topic    
    planning_scene_msg.world.collision_objects.clear();

    planning_scene_msg.robot_state = req.start_state;
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);



    //TODO: REMOVE THE FOLLOWING CODE LATER



    Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    // std::cout<<"Planning Scene transformation frame :: "<< planning_scene->getPlanningFrame();

    // std::cout<<"has Transformation from tool0 :: " << planning_scene->knowsFrameTransform("tool0")<<std::endl;
    Eigen::Affine3d transform_From_BaseLink = planning_scene->getFrameTransform("tool0");

    // std::cout<<"translation from tool0 :: \n"<<transform_From_BaseLink.translation().format(OctaveFmt)<<std::endl;
    // std::cout<<"translation from tool0 :: \n"<<transform_From_BaseLink.rotation().format(OctaveFmt)<<std::endl;
    
    // robot_state::RobotState random_state(robot_model);
    // random_state.setToDefaultValues(random_state.getJointModelGroup(PLANNING_GROUP),"goal_state");
    robot_state::RobotState origState = planning_scene ->getCurrentState();
    // planning_scene->setCurrentState(random_state);

    // std::cout<<"has Transformation from base_link :: " << planning_scene->knowsFrameTransform("tool0")<<std::endl;
    Eigen::Affine3d transform_From_BaseLink2 = planning_scene->getFrameTransform("tool0");

    // std::cout<<"translation from tool0 :: \n"<<transform_From_BaseLink2.translation().format(OctaveFmt)<<std::endl;
    // std::cout<<"translation from tool0 :: \n"<<transform_From_BaseLink2.rotation().format(OctaveFmt)<<std::endl;
    // /*
    //     The following is to create a sphere to visualize the point that the robot is fixated on

    // */
    Eigen::Vector3d pointToFocus;
    Eigen::Vector3d globPointToFocus;
    //pointToFocus << 1.5,0.5,0.6;



    std::string Camera_Link = "tool0";

    robot_state::RobotState test_state(robot_model);
    test_state.setToDefaultValues(test_state.getJointModelGroup(PLANNING_GROUP),start_name);
    test_state.update();


    /*

        GLOBAL VISION CONSTRAINT
    */

    bool useGlobalVisionConstraint = false;
    node_handle.getParam("useGlobalVisionConstraint", useGlobalVisionConstraint);
    std::cout<<"useGlobalVisionConstraint :: " << useGlobalVisionConstraint<<std::endl;

    planning_scene_msg.world.collision_objects.clear();

    if(useGlobalVisionConstraint)
    {
      double globPointToFocusX= 0.0;
      double globPointToFocusY =0.0;
      double globPointToFocusZ =0.0;

      node_handle.getParam("globPointToFocusX",globPointToFocusX);
      node_handle.getParam("globPointToFocusY",globPointToFocusY);
      node_handle.getParam("globPointToFocusZ",globPointToFocusZ);

      globPointToFocus << (double)globPointToFocusX, (double)globPointToFocusY, (double)globPointToFocusZ;


      //Global Camera specs
      //^^^^^^^^^^^^^^^^^^^

      double globCameraAngle = 3.14159/3; //60 degrees for 120 degree vision
      Eigen::Vector3d globDir;
      globDir << 0.0,1.0,0.0;
      planning_scene -> addGlobalCameraConstraint(globCameraAngle, globPointToFocus, globDir);
      planning_scene -> setCameraLink(Camera_Link);

      // moveit_msgs::CollisionObject sphere_object;
      // sphere_object.id="globPoint";
      // shape_msgs::SolidPrimitive sphere_primitive;
      // sphere_primitive.type = sphere_primitive.SPHERE;
      // sphere_primitive.dimensions.resize(1);
      // sphere_primitive.dimensions[0] = 0.1;


      // sphere_object.primitives.push_back(sphere_primitive);
      // sphere_object.primitive_poses.push_back(sphere_pose);
      // sphere_object.operation = sphere_object.ADD;
      // sphere_object.header.frame_id = "base_link";
      // planning_scene_msg.world.collision_objects.push_back(sphere_object);
      // planning_scene_msg.is_diff = true;

      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time::now();
      marker.ns="globPoint";
      marker.id = 20; //ARBITRARY NUMBER
      marker.action = visualization_msgs::Marker::ADD;
      marker.type=visualization_msgs::Marker::MESH_RESOURCE;

      geometry_msgs::Pose sphere_pose;
      sphere_pose.orientation.w = 1.0;
      sphere_pose.position.x = globPointToFocus[0];
      sphere_pose.position.y = globPointToFocus[1];
      sphere_pose.position.z = globPointToFocus[2];

      marker.pose = sphere_pose;
      marker.scale.x=1;
      marker.scale.y=1;
      marker.scale.z=1;
      marker.color.a = 1.0;
      marker.color.r = 0.5;
      marker.color.b= 0.0;
      marker.color.g = 0.0;

      marker.mesh_resource = "package://motoman_sia20d_moveit_updated/src/camera.dae";

      vis_pub.publish(marker);
      
    
      //std::cout<< "Is Global Target in View :: " << (planning_scene->checkGlobalInView(test_state))<<std::endl;


      /*
        check if state works
      */


      planning_scene_diff_publisher.publish(planning_scene_msg);



    }


    planning_scene_msg.world.collision_objects.clear();

    /*

        Regular Vision Constraint

    */


    bool useVisionConstraint = false;
    node_handle.getParam("useVisionConstraint", useVisionConstraint);
    std::cout<<"useVision :: "<<useVisionConstraint<<std::endl;
    if(useVisionConstraint)
    {
      double pointToFocusX= 0.0;
      double pointToFocusY =0.0;
      double pointToFocusZ =0.0;

      node_handle.getParam("pointToFocusX",pointToFocusX);
      node_handle.getParam("pointToFocusY",pointToFocusY);
      node_handle.getParam("pointToFocusZ",pointToFocusZ);
    
      pointToFocus << (double)pointToFocusX,(double)pointToFocusY,(double)pointToFocusZ;

      // moveit_msgs::CollisionObject sphere_object;
      // sphere_object.id="Point_to_Visualize";
      // shape_msgs::SolidPrimitive sphere_primitive;
      // sphere_primitive.type = sphere_primitive.SPHERE;
      // sphere_primitive.dimensions.resize(1);
      // sphere_primitive.dimensions[0] = 0.1;

      // sphere_object.primitives.push_back(sphere_primitive);
      // sphere_object.primitive_poses.push_back(sphere_pose);
      // sphere_object.operation = sphere_object.ADD;
      // sphere_object.header.frame_id = "base_link";
      // planning_scene_msg.world.collision_objects.push_back(sphere_object);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time::now();
      marker.ns="visPoint";
      marker.id = 20; //ARBITRARY NUMBER
      marker.action = visualization_msgs::Marker::ADD;
      marker.type=visualization_msgs::Marker::MESH_RESOURCE;

      geometry_msgs::Pose sphere_pose;
      sphere_pose.orientation.w = 1.0;
      sphere_pose.position.x = pointToFocus[0];
      sphere_pose.position.y = pointToFocus[1];
      sphere_pose.position.z = pointToFocus[2];

      marker.pose = sphere_pose;
      marker.scale.x=1;
      marker.scale.y=1;
      marker.scale.z=1;
      marker.color.a = 1.0;
      marker.color.r = 0.5;
      marker.color.b= 0.0;
      marker.color.g = 0.0;

      marker.mesh_resource = "package://motoman_sia20d_moveit_updated/src/camera.dae";

      vis_pub.publish(marker);


      double camera_angle = 3.14159/6;  //30 degrees; formerly 45
      Eigen::Vector3d visionPoint;


      //EDIT THIS BACK IN FOR A CAMERA CONSTRAINT
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      planning_scene->addCameraConstraint(camera_angle, Camera_Link, pointToFocus);


      planning_scene_msg.is_diff = true;

      planning_scene_diff_publisher.publish(planning_scene_msg);
    }

    visual_tools.prompt("Press next to continue");


    //Problem definition


    robot_state::RobotState goal_state_fmt(robot_model);
    goal_state_fmt.setToDefaultValues(goal_state_fmt.getJointModelGroup(PLANNING_GROUP),end_name);

    moveit_msgs::Constraints joint_goal_fmt = kinematic_constraints::constructGoalConstraints(goal_state_fmt, joint_model_group);
    req.goal_constraints.clear();
    req.goal_constraints.push_back(joint_goal_fmt);


    // We now construct a planning context that encapsulate the scene,
    // the request and the response. We call the planner using this
    // planning context
    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);


    bool seedSuccess = true;

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      numSeedFails++;
      seedSuccess = false;
      ROS_ERROR("Could not compute plan successfully");
    }

    moveit_msgs::MotionPlanResponse response;
    moveit_msgs::DisplayTrajectory display_trajectory;

    if(seedSuccess)  //check if cost is reasonable.if cost > 25, we can assume that the planner wigged out and created an infeasible plan
    {
      res.getMessage(response);
      double checkCost = determineCost(&(response.trajectory.joint_trajectory));
      if(checkCost > 25) 
      {
      //   display_trajectory.trajectory_start = response.trajectory_start;
      //   display_trajectory.trajectory.push_back(response.trajectory);
      //   display_publisher.publish(display_trajectory);

      // //UNCOMMENT FOR STATE-BY-STATE ANALYSIS
      // //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // for(auto iter = 0; iter < response.trajectory.joint_trajectory.points.size(); iter++)
      // {
      //   robot_state::RobotState thisRobotState(robot_model);
      //   thisRobotState.setVariablePositions(response.trajectory.joint_trajectory.points[iter].positions);
      //   thisRobotState.update();
      //   if(useVisionConstraint)
      //   {
      //     std::cout<<"Target Point "<<iter<<" : " << (planning_scene->checkInView(thisRobotState) ? "in view\n":"NOT IN VIEW\n");
      //   }
      //   if(useGlobalVisionConstraint)
      //   {
      //     std::cout<<"Target Point "<<iter<<" : " << (planning_scene->checkGlobalInView(thisRobotState) ? "in view\n":"NOT IN VIEW\n");
      //   }


      //   collision_detection::CollisionRequest c_req;
      //   collision_detection::CollisionResult c_res;
      //   c_req.group_name = PLANNING_GROUP; //replace this for improved modularity later
      //   c_req.contacts = true;
      //   c_req.max_contacts = 100;
      //   c_req.max_contacts_per_pair = 5;
      //   c_req.verbose = false;

      //   planning_scene ->checkCollision(c_req,c_res, thisRobotState);
      //   std::cout<<"    Collision :: "<< ((c_res.collision)?"IN COLLISION\n":"safe\n");

      //   for(auto lmo = 0; lmo <2; lmo++)
      //   {
      //      moveit_msgs::AttachedCollisionObject aco;
      //      if(lmo == 0)
      //      {
      //       //TARGET CAMERA
      //       aco.object = planning_scene->addUnobstructedVision(thisRobotState,pointToFocus, "test_target_vision");
      //      } 
      //      else if(lmo == 1)
      //      {
      //       aco.object = planning_scene->addUnobstructedVision(thisRobotState,globPointToFocus, "test_global_vision");
      //      }
           
      //      std::string id = aco.object.id;
      //      std::vector<shapes::ShapeConstPtr> shapes;
      //      shapes::Shape* s = shapes::constructShapeFromMsg(aco.object.primitives[0]);
      //      shapes.push_back(shapes::ShapeConstPtr(s));

      //      EigenSTL::vector_Affine3d poses;
      //      Eigen::Affine3d p; //this pose is in global frame
      //      tf::poseMsgToEigen(aco.object.primitive_poses.front(), p);

      //      //what is in my pose?
      //      geometry_msgs::Pose vision_pose = aco.object.primitive_poses.front();
      //      poses.push_back(p);
      //      std::vector<std::string> touch_links;
      //      trajectory_msgs::JointTrajectory emptyDetach;

      //      std::string link_name = "base_link";

      //      thisRobotState.attachBody(id,shapes, poses, touch_links, link_name,emptyDetach);

      //      moveit_msgs::RobotState thisRobotStateMsg;
      //      moveit::core::robotStateToRobotStateMsg(thisRobotState,thisRobotStateMsg);

      //      planning_scene_msg.robot_state = thisRobotStateMsg;
      //      planning_scene_msg.robot_state.is_diff=true;

      //      planning_scene_msg.world.collision_objects.clear();
      //      planning_scene_msg.is_diff=true;
      //      planning_scene_diff_publisher.publish(planning_scene_msg);
      //  }

      //    ros::Duration(0.1).sleep();
      // }
        if(planning_scene->isPathValid(response.trajectory_start, response.trajectory, PLANNING_GROUP) == false)
        {
          seedSuccess = false;
          numSeedFails++;
        }
      // visual_tools.prompt("Woah, what do we have here?");
      }
      
    }


    if(seedSuccess)
    {

	    
	   

      avgTime += response.planning_time;
      avgSeedTime += response.planning_time;

      //UNCOMMENT FOLLOWING CODE IF YOU WANT THE FMT TRAJECTORY BROADCASTED to MoveIt! without smoothing
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      /* Visualize the trajectory */
      ////^^^^^^^^^^^^^^^^^^^^^^^^^^^^

     //  ROS_INFO("Visualizing the trajectory");
	    // display_trajectory.trajectory_start = response.trajectory_start;
      // display_trajectory.trajectory.clear();
	    // display_trajectory.trajectory.push_back(response.trajectory);
	    // display_publisher.publish(display_trajectory);




      // UNCOMMENT FOLLOWING CODE IF YOU WISH TO SAVE THE FMT TRAJECTORY TO A FILE. DON't
      // FORGET TO CHANGE THE PATH DIRECTORY. Time parameterization not saved.
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // std::string path = "/home/tariq/Documents/fmt_shortcut_RECORDS.m";
      // std::ofstream outputFile;
      // outputFile.open(path,std::ios::app);
      // outputFile << "FMT{" +std::to_string(main_loop_iter) + "} = [";
      // std::vector<int>::size_type numJointsTariq = response.trajectory.joint_trajectory.points[0].positions.size();
      // std::vector<int>::size_type size1 = response.trajectory.joint_trajectory.points.size();
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




	    //Extra Maneuvers
	    //^^^^^^^^^^^^^^^

	    //Displays the Cost
      double seedCost = determineCost(&(response.trajectory.joint_trajectory));
      avgSeedCost += seedCost;
      ROS_INFO_STREAM("BFMT Cost :: " + std::to_string(seedCost));



	    //Uncomment below to Publish JointTrajectory message for rqt plot visualization
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	    //visualizePlot(&(response.trajectory.joint_trajectory));


      //Uncomment the following code to make the simulation require an input before continuing
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //visual_tools.prompt("Please press next in the RvizVisualToolsGui for Shortcut Algorithm. Make sure to add the button via the Panels menu in the top bar.");


    /////////////////////////
    //SECOND PART of the Code 
    /////////////////////////

      // UNCOMMENT FOR SHORTCUT
      // Send the trajectory to the Shortcut Method to prune unnecessary movements
      ROS_INFO_STREAM("Pre-processing size :: " + std::to_string(response.trajectory.joint_trajectory.points.size()));
      if(shortcutMethod == "AdaptivePartial" || shortcutMethod == "Partial")
      {
        avgTime += shortcut_planner->PartialShortcut(&(response.trajectory.joint_trajectory), &planning_scene, joint_model_group, &(*robot_state), numShortcutLoops);
      } else if(shortcutMethod == "Regular") { //has to be regular Shortcut
        avgTime +=  shortcut_planner->RegularShortcut(&(response.trajectory.joint_trajectory),&planning_scene,joint_model_group,&(*robot_state),numShortcutLoops,3);
      } else if(shortcutMethod == "Adaptive") {
        avgTime += shortcut_planner->AdaptiveShortcut(&(response.trajectory.joint_trajectory),&planning_scene,joint_model_group,&(*robot_state),numShortcutLoops,3);
      }
      


      //Time parameterize said trajectory again (waypoints were removed and added from previous method)
      double tempTime = timeParameterize(&response, robot_model, &start_state);
      if(tempTime !=-1) //reprsents that time parameterization found a solution
      {
        avgTime += tempTime;
      }
      ROS_INFO_STREAM("Post-processing size :: " +  std::to_string(response.trajectory.joint_trajectory.points.size()));

      //Send the trajectory to RViz for Visualization
      display_trajectory.trajectory_start = response.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
      display_trajectory.trajectory.clear();
      display_trajectory.trajectory.push_back(response.trajectory);
      display_publisher.publish(display_trajectory);

      // visual_tools.prompt("Please press next in the RvizVisualToolsGui to do state by state analysis");

      // UNCOMMENT FOR STATE-BY-STATE ANALYSIS
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      for(auto iter = 0; iter < response.trajectory.joint_trajectory.points.size(); iter++)
      {
        robot_state::RobotState thisRobotState(robot_model);
        thisRobotState.setVariablePositions(response.trajectory.joint_trajectory.points[iter].positions);
        thisRobotState.update();
        if(useVisionConstraint)
        {
          std::cout<<"Target Point "<<iter<<" : " << (planning_scene->checkInView(thisRobotState) ? "in view\n":"NOT IN VIEW\n");
        }
        if(useGlobalVisionConstraint)
        {
          std::cout<<"Target Point "<<iter<<" : " << (planning_scene->checkGlobalInView(thisRobotState) ? "in view\n":"NOT IN VIEW\n");
        }


        collision_detection::CollisionRequest c_req;
        collision_detection::CollisionResult c_res;
        c_req.group_name = PLANNING_GROUP; //replace this for improved modularity later
        c_req.contacts = true;
        c_req.max_contacts = 100;
        c_req.max_contacts_per_pair = 5;
        c_req.verbose = false;

        planning_scene ->checkCollision(c_req,c_res, thisRobotState);
        std::cout<<"    Collision :: "<< ((c_res.collision)?"IN COLLISION\n":"safe\n");

        for(auto lmo = 0; lmo <2; lmo++)
        {
           moveit_msgs::AttachedCollisionObject aco;
           if(lmo == 0)
           {
            //TARGET CAMERA
            aco.object = planning_scene->addUnobstructedVision(thisRobotState,pointToFocus, "test_target_vision");
           } 
           else if(lmo == 1)
           {
            aco.object = planning_scene->addUnobstructedVision(thisRobotState,globPointToFocus, "test_global_vision");
           }
           
           std::string id = aco.object.id;
           std::vector<shapes::ShapeConstPtr> shapes;
           shapes::Shape* s = shapes::constructShapeFromMsg(aco.object.primitives[0]);
           shapes.push_back(shapes::ShapeConstPtr(s));

           EigenSTL::vector_Affine3d poses;
           Eigen::Affine3d p; //this pose is in global frame
           tf::poseMsgToEigen(aco.object.primitive_poses.front(), p);

           //what is in my pose?
           geometry_msgs::Pose vision_pose = aco.object.primitive_poses.front();
           poses.push_back(p);
           std::vector<std::string> touch_links;
           trajectory_msgs::JointTrajectory emptyDetach;

           std::string link_name = "base_link";

           thisRobotState.attachBody(id,shapes, poses, touch_links, link_name,emptyDetach);

           moveit_msgs::RobotState thisRobotStateMsg;
           moveit::core::robotStateToRobotStateMsg(thisRobotState,thisRobotStateMsg);

           planning_scene_msg.robot_state = thisRobotStateMsg;
           planning_scene_msg.robot_state.is_diff=true;

           planning_scene_msg.world.collision_objects.clear();
           planning_scene_msg.is_diff=true;
           planning_scene_diff_publisher.publish(planning_scene_msg);
       }

         ros::Duration(0.1).sleep();

         //Comment out
         visual_tools.prompt("Press next to continue");
      }



      // UNCOMMENT FOLLOWING CODE IF YOU WISH TO SAVE THE FINAL TRAJECTORY TO A FILE. DON't
      // FORGET TO CHANGE THE PATH DIRECTORY
      // outputFile.open(path,std::ios::app);
      // outputFile << "Shortcut{" +std::to_string(main_loop_iter) + "} = [";
      // numJointsTariq = response.trajectory.joint_trajectory.points[0].positions.size();
      // size1 = response.trajectory.joint_trajectory.points.size();
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

      double shortcutCost = determineCost(&(response.trajectory.joint_trajectory));
      avgShortcutCost += shortcutCost;
      ROS_INFO_STREAM("Final Plan Cost :: " + std::to_string(shortcutCost));

      //Save the best plan
      if(shortcutCost < bestCost)
      {
        bestResponse.trajectory_start = response.trajectory_start;
        bestResponse.trajectory = response.trajectory;
        bestCost = shortcutCost;
      }

    }
    ROS_INFO_STREAM("Current iteration :: "+std::to_string(main_loop_iter));

    //TODO: REMOVE THIS SECTION
    // std::cout<<"Planning Scene transformation frame :: "<< planning_scene->getPlanningFrame();

    // std::cout<<"has Transformation from base_link :: " << planning_scene->knowsFrameTransform("/base_link")<<std::endl;
    // Eigen::Affine3d transform_From_BaseLink2 = planning_scene->getFrameTransform("/base_link");
    // std::cout<<"translation from base_link :: \n"<<transform_From_BaseLink2.translation().format(OctaveFmt)<<std::endl;
    // std::cout<<"translation from base_link :: \n"<<transform_From_BaseLink2.rotation().format(OctaveFmt)<<std::endl;
    
    // std::cout<<"has Transformation from tool0 :: " << planning_scene->knowsFrameTransform("tool0")<<std::endl;
    // Eigen::Affine3d transform_From_EE2 = planning_scene->getFrameTransform("tool0");
    // std::cout<<"translation from tool0 :: \n"<<transform_From_EE2.translation().format(OctaveFmt)<<std::endl;
    // std::cout<<"rotation from tool0 :: \n"<<transform_From_EE2.rotation().format(OctaveFmt)<<std::endl;

    //UNCOMMENT following line if you wish to make code stop here until a button press is received.
    visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");

  }

  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = bestResponse.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
  display_trajectory.trajectory.clear();
  display_trajectory.trajectory.push_back(bestResponse.trajectory);
  display_publisher.publish(display_trajectory);

  avgTime = avgTime/(max_Iter-(numSeedFails+numWeirdSeeds));
  avgSeedTime = avgSeedTime/(max_Iter-(numSeedFails+numWeirdSeeds));
  avgShortcutCost = avgShortcutCost/(max_Iter-(numSeedFails+numWeirdSeeds));
  avgSeedCost = avgSeedCost/(max_Iter-(numSeedFails+numWeirdSeeds));
  ROS_INFO_STREAM("Average BFMT* Time :: " +std::to_string(avgSeedTime));
  ROS_INFO_STREAM("Average BFMT*+Shortcut Time :: " +std::to_string(avgTime));
  ROS_INFO_STREAM("Number of SEED failures :: " +std::to_string(numSeedFails));
  ROS_INFO_STREAM("Number of Weird Seeds :: " +std::to_string(numWeirdSeeds));
  ROS_INFO_STREAM("Average BFMT* Cost :: " +std::to_string(avgSeedCost));
  ROS_INFO_STREAM("Average BFMT*+Shortcut Cost :: " + std::to_string(avgShortcutCost));



  // UNCOMMENT FOLLOWING CODE IF YOU WISH TO SAVE THE FMT TRAJECTORY TO A FILE. DON't
  // FORGET TO CHANGE THE PATH DIRECTORY. Time parameterization not saved.
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  std::string path = "/home/tariq/Documents/camera_navigation_records.txt";
  std::ofstream outputFile;
  outputFile.open(path,std::ios::app);

  outputFile << "*********************************\n";
  outputFile << shortcutMethod.c_str()<< std::endl;
  outputFile << "*********************************\n";

  outputFile << "Seed Planning Time (s) :: \n" <<avgSeedTime<<std::endl;
  outputFile << "Seed Cost :: \n" << avgSeedCost<<std::endl;
  outputFile << "Number of Failures :: \n"<<numSeedFails<<std::endl<<std::endl;

  outputFile << "Seed+Shortcut Planning Time (s) :: \n" <<avgTime<<std::endl;
  outputFile << "Seed+Shortcut Cost :: \n" << avgShortcutCost<<std::endl;
  outputFile << "Number of Failures :: \n"<<numSeedFails<<std::endl;

  outputFile << std::endl<<std::endl;
  outputFile.close();
  ROS_INFO_STREAM("Finished creating log file");


// } //end gtts

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


Purpose: Simple function that prints out the cost of a trajectory. Returns the total distance
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
      cost += (costOfCurrentMovement);
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
  
  Function: addObstacles


  Purpose: Adds preset obstacle environments to the planning scene
  */


static void addObstacles(planning_scene::PlanningScenePtr planning_scene, ros::Publisher *planning_scene_diff_publisher, moveit_msgs::PlanningScene *planning_scene_msg, std::string environment, ros::Publisher &vis_pub)
{
  if(environment == "satellite")
  {
    // double scale;
    // node_handle.getParam("scale",scale);
    // Eigen::Vector3d vector_scale;
    // vector_scale<<scale,scale,scale;


    /*
  

        NOTE: MARKER VISUALIZATION PROBLEMS BASED ON LONG START UP. MAYBE WAIT A BIT?
    */

    std::string file_loc = "package://motoman_sia20d_moveit_updated/src/satellite_collision.dae";//"package://motoman_sia20d_moveit_updated/src/satellite_mock_up_scaled.dae";

    moveit_msgs::CollisionObject satellite;
    satellite.id="satellite";
    shapes::Mesh* mesh = shapes::createMeshFromResource(file_loc);
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(mesh,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    satellite.meshes.resize(1);
    satellite.meshes[0] = co_mesh;
    satellite.mesh_poses.resize(1);
    // node_handle.getParam("satX",satellite.mesh_poses[0].position.x);
    // node_handle.getParam("satY",satellite.mesh_poses[0].position.y);
    // node_handle.getParam("satZ",satellite.mesh_poses[0].position.z);
    satellite.mesh_poses[0].position.x = 0.20;
    satellite.mesh_poses[0].position.y = 0.45;//0.38;
    satellite.mesh_poses[0].position.z = -0.2;
    // node_handle.getParam("orW",satellite.mesh_poses[0].orientation.w);
    // node_handle.getParam("orX",satellite.mesh_poses[0].orientation.x);
    // node_handle.getParam("orY",satellite.mesh_poses[0].orientation.y);
    // node_handle.getParam("orZ",satellite.mesh_poses[0].orientation.z);
    satellite.mesh_poses[0].orientation.w = 1.0;
    satellite.mesh_poses[0].orientation.x = 0.0;
    satellite.mesh_poses[0].orientation.y = 0.0;
    satellite.mesh_poses[0].orientation.z = 0.0;
    satellite.id="satellite";
    satellite.header.frame_id = "base_link";
    

    visualization_msgs::Marker marker;
    marker.header.frame_id = satellite.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns=satellite.id;
    marker.id = 20; //ARBITRARY NUMBER
    marker.action = visualization_msgs::Marker::ADD;
    marker.type=visualization_msgs::Marker::MESH_RESOURCE;
    marker.pose = satellite.mesh_poses[0];
    marker.scale.x=1;
    marker.scale.y=1;
    marker.scale.z=1;
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.b= 0.0;
    marker.color.g = 1.0;

    marker.mesh_resource = "package://motoman_sia20d_moveit_updated/src/satellite_mock_up_scaled.dae";

    planning_scene->processCollisionObjectMsg(satellite);
    

    //For some reason, RVIZ has trouble taking meshes from planning_scene_msg
    vis_pub.publish(marker);
    //planning_scene_msg->world.collision_objects.push_back(satellite);


  }


  if(environment == "floating")
  {
    std::string file_loc = "package://motoman_sia20d_moveit_updated/src/floating_scaled.dae";

    moveit_msgs::CollisionObject maze;
    maze.id="floating";
    shapes::Mesh* mesh = shapes::createMeshFromResource(file_loc);
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(mesh,co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    maze.meshes.resize(1);
    maze.meshes[0] = co_mesh;
    maze.mesh_poses.resize(1);

    maze.mesh_poses[0].position.x = 0.3;
    maze.mesh_poses[0].position.y = 0.0;
    maze.mesh_poses[0].position.z = 0.75;
    maze.mesh_poses[0].orientation.w = 1.0;
    maze.mesh_poses[0].orientation.x = 0.0;
    maze.mesh_poses[0].orientation.y = 0.0;
    maze.mesh_poses[0].orientation.z = 0.0;
    maze.id="floating";
    maze.header.frame_id = "base_link";

    visualization_msgs::Marker marker;
    marker.header.frame_id = maze.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns=maze.id;
    marker.id = 20; //ARBITRARY NUMBER
    marker.action = visualization_msgs::Marker::ADD;
    marker.type=visualization_msgs::Marker::MESH_RESOURCE;
    marker.pose = maze.mesh_poses[0];
    marker.scale.x=1;
    marker.scale.y=1;
    marker.scale.z=1;
    marker.color.a = 1.0;
    marker.color.r = 0.5;
    marker.color.b= 0.0;
    marker.color.g = 1.0;

    marker.mesh_resource = "package://motoman_sia20d_moveit_updated/src/floating_visual.dae";
    // ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker",0);
    // while(vis_pub.getNumSubscribers() < 1)
    // {
    //   std::cout<<"Waiting for Marker to connect to /camera_navigation/visualization_marker"<<std::endl;
    //   ros::Duration(0.5).sleep();
    // }

    planning_scene->processCollisionObjectMsg(maze);
    

    //For some reason, RVIZ has trouble taking meshes from planning_scene_msg
    vis_pub.publish(marker);
    //planning_scene_msg->world.collision_objects.push_back(maze);

  }


  if(environment == "box")
  {
    //SQUARE BOX
    //^^^^^^^^^^^
    moveit_msgs::CollisionObject collision_object;
    collision_object.id="box1";
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
    collision_object.header.frame_id = "base_link";
    planning_scene->processCollisionObjectMsg(collision_object);
    planning_scene_msg->world.collision_objects.push_back(collision_object);



    moveit_msgs::CollisionObject sphere_object;
    sphere_object.id="sphere_object";
    shape_msgs::SolidPrimitive sphere_primitive;
    sphere_primitive.type = sphere_primitive.SPHERE;
    sphere_primitive.dimensions.resize(1);
    sphere_primitive.dimensions[0] = 0.3;

    geometry_msgs::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = 0.8;
    sphere_pose.position.y = 0.4;
    sphere_pose.position.z = 1.1;

    sphere_object.primitives.push_back(sphere_primitive);
    sphere_object.primitive_poses.push_back(sphere_pose);
    sphere_object.operation = sphere_object.ADD;
    sphere_object.header.frame_id = "base_link";
    planning_scene_msg->world.collision_objects.push_back(sphere_object);
    planning_scene->processCollisionObjectMsg(sphere_object);

    // moveit_msgs::CollisionObject collision_object2;
    // collision_object2.id="box2";
    // shape_msgs::SolidPrimitive primitive2;
    // primitive2.type = primitive.BOX;
    // primitive2.dimensions.resize(3);
    // primitive2.dimensions[0] = 0.2;
    // primitive2.dimensions[1] = 0.4;
    // primitive2.dimensions[2] = 0.4;

    // geometry_msgs::Pose box_pose2;
    // box_pose2.orientation.w = 1.0;
    // box_pose2.position.x = 0.8;
    // box_pose2.position.y = 0.6;
    // box_pose2.position.z = 1.1;

    // collision_object2.primitives.push_back(primitive2);
    // collision_object2.primitive_poses.push_back(box_pose2);
    // collision_object2.operation = collision_object2.ADD;
    // collision_object.header.frame_id = "base_link";

    // planning_scene_msg->world.collision_objects.push_back(collision_object2);
    // planning_scene->processPlanningSceneWorldMsg  (planning_scene_msg->world);
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
    box_pose.position.y = 0.30;//0.15;
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

