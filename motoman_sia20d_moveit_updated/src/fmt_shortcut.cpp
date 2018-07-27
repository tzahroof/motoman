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
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_detection/collision_tools.h>

//for time parameterization
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

//for writing files
#include <iostream>
#include <fstream>


/*
  GLOBAL VARIABLES
*/

const std::string PLANNING_GROUP = "manipulator";
double max_EdgeLength_Discretization; //determines how big the maxEdgeLength should be for checking discretization
double max_EdgeLength_Waypoint_Injection; //determines how big the maxEdgeLength should be for WayPoint injection during populate()
/*
  Prototype Functions
*/
static double determineCost(trajectory_msgs::JointTrajectory *joint_trajectory);
static void visualizePlot(trajectory_msgs::JointTrajectory *joint_trajectory, ros::Publisher *rqt_publisher);
static void populatePath(trajectory_msgs::JointTrajectory *joint_trajectory);
static bool checkIfPathHasCollisions(moveit_msgs::MotionPlanResponse *response, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state);
static bool clearPath(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int a, int b, int defaultNumPoints);
static double Shortcut(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr *planning_scene, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int numShortcutLoops, int defaultNumPoints);
static double timeParameterize(moveit_msgs::MotionPlanResponse *response, robot_model::RobotModelPtr robot_model, robot_state::RobotState *start_state);


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
  int numSeedFails = 0;
  double avgTime = 0.0;

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
    if(!node_handle.getParam("max_EdgeLength_Discretization",max_EdgeLength_Discretization))
    {
      ROS_ERROR_STREAM("Could not load the max_EdgeLength_Discretization Parameter");
    }

    if(!node_handle.getParam("max_EdgeLength_Waypoint_Injection", max_EdgeLength_Waypoint_Injection))
    {
      ROS_ERROR_STREAM("Could not load the max_EdgeLength_Waypoint_Injection Parameter");
    }

    if(!node_handle.getParam("max_Iter", max_Iter))
    {
      ROS_INFO_STREAM("No max_Iter value provided. Defaulting to 50 iterations instead");
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

   // UNCOMMENT the following line  to only start the program upon button press
   // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo. Make sure to add the button via the Panels menu in the top bar.");

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

  ros::Duration(1).sleep();

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
    if(!node_handle.getParam("allowed_planning_time", req.allowed_planning_time))
    {
      ROS_ERROR_STREAM("Could not find the allowed_planning_time parameter");
    }


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
    start_state.setToDefaultValues(start_state.getJointModelGroup(PLANNING_GROUP),"home");
    moveit::core::robotStateToRobotStateMsg(start_state,req.start_state);
   
    //Displays the starting state in MoveIt! via display_robot_state topic    
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.robot_state = req.start_state;
    planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);



  	//Explicitly sets goal state
    //^^^^^^^^^^^^^^^^^^^^^^^^^^

    // UNCOMMENT Code if you want to set goal_state by joint values instead of from SRDF file
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
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


    bool seedSuccess = true;

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      numSeedFails++;
      seedSuccess = false;
      ROS_ERROR("Could not compute plan successfully");
    }


    if(seedSuccess)
    {

	    moveit_msgs::MotionPlanResponse response;
	    res.getMessage(response);
      moveit_msgs::DisplayTrajectory display_trajectory;

      avgTime += response.planning_time;


      //UNCOMMENT FOLLOWING CODE IF YOU WANT THE FMT TRAJECTORY BROADCASTED to MoveIt! without smoothing
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      /* Visualize the trajectory */
      ////^^^^^^^^^^^^^^^^^^^^^^^^^^^^

     //  ROS_INFO("Visualizing the trajectory");
	    // display_trajectory.trajectory_start = response.trajectory_start;
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
	    ROS_INFO_STREAM("BFMT Cost :: " + std::to_string(determineCost(&(response.trajectory.joint_trajectory))));



	    //Uncomment below to Publish JointTrajectory message for rqt plot visualization
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	    //visualizePlot(&(response.trajectory.joint_trajectory));


      //Uncomment the following code to make the simulation require an input before continuing
      //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      //visual_tools.prompt("Please press next in the RvizVisualToolsGui for Shortcut Algorithm. Make sure to add the button via the Panels menu in the top bar.");


    /////////////////////////
    //SECOND PART of the Code 
    /////////////////////////


      // Send the trajectory to the Shortcut Method to prune unnecessary movements
      ROS_INFO_STREAM("Pre-processing size :: " + std::to_string(response.trajectory.joint_trajectory.points.size()));
      avgTime += Shortcut(&(response.trajectory.joint_trajectory),&planning_scene,joint_model_group,&(*robot_state),30,3);


      //Time parameterize said trajectory again (waypoints were removed and added from previous method)
      double tempTime = timeParameterize(&response, robot_model, &start_state);
      if(tempTime !=-1) //reprsents that time parameterization found a solution
      {
        avgTime += tempTime;
      }

      //Send the trajectory to RViz for Visualization
      display_trajectory.trajectory_start = response.trajectory_start;  //this might suggest why it starts off the wrong way sometimes?
      display_trajectory.trajectory.clear();
      display_trajectory.trajectory.push_back(response.trajectory);
      display_publisher.publish(display_trajectory);

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

      ROS_INFO_STREAM("Final Plan Cost :: " + std::to_string(determineCost(&(response.trajectory.joint_trajectory))));

    }
    ROS_INFO_STREAM("Current iteration :: "+std::to_string(main_loop_iter));


    //UNCOMMENT following line if you wish to make code stop here until a button press is received.
    //visual_tools.prompt("Please press next in the RVizVisualToolsGui to continue. Make sure to add the button via the Panels menu in the top bar.");

  }

  avgTime = avgTime/(max_Iter-numSeedFails);
  ROS_INFO_STREAM("Average Time :: " +std::to_string(avgTime));
  ROS_INFO_STREAM("Number of SEED failures :: " +std::to_string(numSeedFails));

}

/*

Function: Shortcut


Purpose: Uses the shortcut method to prune the tree. Essentially, it tries to connect to random nodes in a trajectory with a straight line. If the two nodes have a clear path, then we remove all nodes
         in between those two random nodes, as a straight line path is the most efficient way to connect two nodes.


*/


static double Shortcut(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr *planning_scene, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int numShortcutLoops, int defaultNumPoints)
{

  double time = ros::Time::now().toSec();

  int startingNumPoints = joint_trajectory->points.size();

  for(int loop_iter = 0; loop_iter < numShortcutLoops; loop_iter++) 
  {
    std::vector<int>::size_type trajSize = joint_trajectory->points.size();

    int a;
    int b;

    if(trajSize <= 2)
    {
      ROS_INFO_STREAM("Trajectory has "+std::to_string(trajSize)+" points\n");
      break; //trajectory has been reduced to a straightline between two points, is just a single point, or doesn't have any points
    }
    else if(trajSize == 3)
    {
      a = 0; //special case to prevent modulus by 0
      b = 2;
    }
    else 
    {
      a = rand()%(trajSize-3); //returns a value between 0 and the third-to-last element (b can be from 0 -> last element)
      b = rand()%(trajSize);

      while(!(b > a && (b-a) > 1)) { 
      b = rand()%(trajSize); //ensures that index b is after a
    }
    }

    if(clearPath(joint_trajectory, *planning_scene, joint_model_group, robot_state, a, b, defaultNumPoints)) //straightline path is clear; therefore remove all nodes in between
    {
      joint_trajectory->points.erase(joint_trajectory->points.begin()+a+1, joint_trajectory->points.begin()+b);
    }
  }
  int postShortcutNumPoints = joint_trajectory->points.size();

  populatePath(joint_trajectory);
  int postPopulatePoints = joint_trajectory->points.size();

  ROS_INFO("Shortcut method: %d -> %d -> %d points",startingNumPoints, postShortcutNumPoints, postPopulatePoints );
  return ros::Time::now().toSec()-time;
}


/*

Function: clearPath


Purpose: Determines if the path between two nodes is clear. It begins by discretizing the the path between two nodes
         until the edgelength is between them is 1.0. Then, the collision checker checks each node (effectively, a robot
         state) to see if said state is in collision. Returns true if the path is collision-free.
*/
    


static bool clearPath(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int a, int b, int defaultNumPoints)
{
  int numPoints = defaultNumPoints;
  int a_size = joint_trajectory->points[a].positions.size();
  int b_size = joint_trajectory->points[b].positions.size();
  if(a_size != b_size)
  {
    ROS_ERROR_STREAM("The number of joints of a and b in Clear Path are not equal ");
    return false;
  }
  
  // Use Eigen::VectorXd for easy vector math
  Eigen::VectorXd APoints(joint_trajectory->points[a].positions.size());
  
  for(int i = 0; i < a_size; i++)
  {
    APoints(i) = (joint_trajectory->points[a].positions[i]);
  }

  Eigen::VectorXd BPoints(joint_trajectory->points[b].positions.size());
  for(int i = 0; i < b_size; i++)
  {
    BPoints(i) = joint_trajectory->points[b].positions[i];
  }



  bool notDoneFlag = true;
  double oldnorm = 0.0; //REMOVE when debug code is removed

  // If the edgelength between the discretized nodes is less than 1.0, then we need more points betweeen the waypoints.
  while(notDoneFlag) 
  {

    if(  ((BPoints - APoints) /(numPoints-1)).norm() <= max_EdgeLength_Discretization   )
    {
      notDoneFlag = false;
    } 
    else
    {
      numPoints = numPoints * 2;
    }
  }

  // Sets up a collision check for each of the nodes of the dicretized path. Then uses the 
  // collision_detection::CollisionRequest object to check if the state is in collision.
  for(int i = 0; i <= numPoints-1; i++) 
  {
    Eigen::VectorXd substateVector = APoints + (BPoints - APoints)/(numPoints-1) * i;


    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = PLANNING_GROUP; //replace this for improved modularity later
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;
    std::vector<double> substate_joint_vals(&substateVector[0], substateVector.data()+substateVector.cols()*substateVector.rows());

    robot_state->setJointGroupPositions(joint_model_group,substate_joint_vals);

    planning_scene ->checkCollision(c_req,c_res, *robot_state);

    if(c_res.collision)
    {
      return false;
    }

  }
  return true;
}

/*

Function: populatePath


Purpose: After a path is generated by Shortcut, the number of waypoints may potentially be way too few to be useful
         for generating useful velocity/acceleration parameters from start to finish. As such, this method adds
         waypoint markers in a straight-line between the nodes of the original trajectory to allow for better
         post-plan time parameterization


*/

static void populatePath(trajectory_msgs::JointTrajectory *joint_trajectory)
{
  std::vector<trajectory_msgs::JointTrajectoryPoint> *points;
  points = &(joint_trajectory->points);
  std::vector<int>::size_type trajSize = points->size();
  std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator firstIndex = points->begin();

  for(int i = 0; i < trajSize-1; i++)
  {
     Eigen::VectorXd firstVec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((*points)[i].positions.data(),(*points)[i].positions.size());
     Eigen::VectorXd secondVec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>((*points)[i+1].positions.data(),(*points)[i+1].positions.size());
     
     if((secondVec-firstVec).norm() > max_EdgeLength_Waypoint_Injection)
     {
       Eigen::VectorXd unitVec = firstVec + ((secondVec-firstVec)/(secondVec-firstVec).norm()) * 0.1;
       points->insert(firstIndex+i+1,trajectory_msgs::JointTrajectoryPoint());
       std::vector<double> intermediatePositions(&unitVec[0], unitVec.data()+unitVec.cols()*unitVec.rows());
       (*points)[i+1].positions = intermediatePositions;

       trajSize =points->size();
       firstIndex = points->begin();
     }
  }
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

Function: checkIfPathHasCollisions


Purpose: Uses both the created Collision Checker (created by Tariq, not the MoveIt! one) and also
         double-checks the waypoints to see if the generated path creates collisions. Returns true if
         there is a collision
*/

static bool checkIfPathHasCollisions(moveit_msgs::MotionPlanResponse *response, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state)
{
  bool pass = false;

  //Goes through the path and checks if there are collisions between the waypoints (inclusive)
  for(int i = 0; i < response->trajectory.joint_trajectory.points.size()-1;i++)
  {
    if(clearPath(&(response->trajectory.joint_trajectory), planning_scene, joint_model_group, robot_state, i, i+1, 3) == false)
    {
      ROS_INFO_STREAM("Found a collision by standard collision checking");
      pass = true;
    }
  }


  // Checks if only the waypoints are in collision. Useful if one is unsure if the MoveIt!-generated trajectory
  // is clean.
  for(int i = 0; i < response->trajectory.joint_trajectory.points.size(); i++)
  {

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = PLANNING_GROUP; //replace this for improved modularity later
    c_req.contacts = true;
    c_req.max_contacts = 100;
    c_req.max_contacts_per_pair = 5;
    c_req.verbose = false;

    robot_state->setJointGroupPositions(joint_model_group,response->trajectory.joint_trajectory.points[i].positions);

    planning_scene ->checkCollision(c_req,c_res, *robot_state);

    if(c_res.collision)
    {
      ROS_INFO_STREAM("WayPoint Checker also found a collision...");
    }

  }

  return pass;
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
