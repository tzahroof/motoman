#ifndef FMT_SHORTCUT_
#define FMT_SHORTCUT_

#include <chrono>
#include <complex>
#include <math.h> 
#include <limits>

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

//for writing files
#include <iostream>
#include <fstream>

namespace shortcut{

class Shortcut_Planner{
	private:
		std::string PLANNING_GROUP;
		double max_EdgeLength_Discretization; //determines how big the maxEdgeLength should be for checking discretization
		double max_EdgeLength_Waypoint_Injection; //determines how big the maxEdgeLength should be for WayPoint injection during populate()
		double allowed_planning_time;
		int adaptive_repetitions;
		std::string planner_type;

	public:

	void setPlannerType(std::string shortcut_method)
	{
		planner_type = shortcut_method;
	}


	void initialize(std::string shortcut_method,std::string manipulator_group, double planning_time)
	{
		PLANNING_GROUP = manipulator_group;
		allowed_planning_time = planning_time;
		planner_type = shortcut_method;
	}

	void setAdaptiveRepetitions(int ar)
	{
		adaptive_repetitions = ar;
	}

	void setRegularShortcutSettings(double edgelength_discretization, double waypoint_injection)
	{
		max_EdgeLength_Discretization = edgelength_discretization;
		max_EdgeLength_Waypoint_Injection = waypoint_injection;
	}

	Shortcut_Planner(std::string shortcut_method,std::string manipulator_group, double planning_time, double edgelength_discretization, double waypoint_injection, int numberOfAdaptiveRepetitions)
	{
		initialize(shortcut_method,manipulator_group,planning_time);
		setRegularShortcutSettings(edgelength_discretization,waypoint_injection);
		setAdaptiveRepetitions(numberOfAdaptiveRepetitions);
	}

	Shortcut_Planner(std::string shortcut_method,std::string manipulator_group, double planning_time, double edgelength_discretization, double waypoint_injection)
	{
		if(shortcut_method == "Adaptive")
		{
			Shortcut_Planner(shortcut_method, manipulator_group,planning_time, edgelength_discretization,waypoint_injection, 5);
		} else
		{
			initialize(shortcut_method,manipulator_group,planning_time);
			setRegularShortcutSettings(edgelength_discretization,waypoint_injection);
		}
	}

	Shortcut_Planner(std::string shortcut_method,std::string manipulator_group, double planning_time)
	{
		if(shortcut_method == "Regular" || shortcut_method == "Adaptive")
		{
			Shortcut_Planner(shortcut_method,manipulator_group,planning_time,0.1,0.2);
		}
		else 
		{
			initialize(shortcut_method, manipulator_group, planning_time);
		}

	}


	Eigen::VectorXd determineCostVector(trajectory_msgs::JointTrajectory *joint_trajectory)
	{
	    std::vector<int>::size_type numJointsTariq = joint_trajectory->points[0].positions.size();
	    std::vector<int>::size_type size1 = joint_trajectory->points.size();
	    Eigen::VectorXd returnCostVector(joint_trajectory -> points[0].positions.size());
	    for(unsigned j = 0; j < numJointsTariq; j++) {
	      double costOfCurrentMovement = 0;
	      for(unsigned iter = 0; iter < size1-1; iter++) {
	            costOfCurrentMovement += 
	              fabs((joint_trajectory->points[iter+1].positions[j] - joint_trajectory->points[iter].positions[j]));
	      }
	      returnCostVector(j) = (costOfCurrentMovement);
	    }
	    return returnCostVector;
	}

/*

Function: selectDof()

Purpose: Selects a degree-of-freedom to perform partial shortcut method based on AWD (Weight Distribution of Joints). Currently selects only 1 joint.
         Sums the weights of AWD to randomly select a degree-

*/
	 

	int selectDOF(Eigen::VectorXd AWD)
	{
	  double sumOfWeights = 0;
	  for(int i = 0; i < AWD.size(); i++)
	  {
	    sumOfWeights += AWD(i);
	  }
	  double rnd = randomNumber(0,sumOfWeights);
	  for(int i = 0; i < AWD.size(); i++)
	  {
	    if(rnd < AWD(i))
	    {
	      return i; 
	    } else
	    {
	      rnd -= AWD(i);
	    }
	  }

	  return 0; //we should never get here

	}

	Eigen::VectorXd genAWD(Eigen::VectorXd refCost, trajectory_msgs::JointTrajectory *joint_trajectory)
	{
	  Eigen::VectorXd currentCost = determineCostVector(joint_trajectory);
	  return (currentCost-refCost).cwiseAbs();
	}


/*
Function: randomNumber()

Purpose: Selects a random number between two bounds

*/

	double randomNumber(double lower_bound, double upper_bound)
	{
	  return (double)rand()/RAND_MAX*(upper_bound-lower_bound)+lower_bound;
	}



/*
Function: clearPathPartial


Purpose: Checks whether a path is collision-free by interpolating a specific degree of freedom between two specified waypoints


*/

	bool clearPathPartial(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int a, int b, int dof)
	{
	  int a_size = joint_trajectory->points[a].positions.size();
	  int b_size = joint_trajectory->points[b].positions.size();

	  std::vector<double>::size_type numberDOFs = joint_trajectory->points[0].positions.size();

	  if(a_size != b_size)
	  {
	    ROS_ERROR_STREAM("The number of joints of a and b in Clear Path are not equal ");
	    return false;
	  }
	  

	  double increment = (joint_trajectory->points[b].positions[dof] - joint_trajectory->points[a].positions[dof])/(b-a);


	  // Sets up a collision check for each of the nodes of the discretized path. Then uses the 
	  // collision_detection::CollisionRequest object to check if the state is in collision.
	  for(int i = 0; i <= b-a; i++) 
	  {
	    Eigen::VectorXd substateVector(numberDOFs);
	    //Eigen::VectorXd substateVector = APoints + (BPoints - APoints)/(numPoints-1) * i;
	    for(int j = 0; j < numberDOFs;j++)
	    {
	      substateVector(j) = joint_trajectory->points[a+i].positions[j];
	    }
	    //interpolate the selected dof over the waypoints from a -> b
	    substateVector(dof) = joint_trajectory->points[a].positions[dof] + increment*i;

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

	Function: determineCostVector


	Purpose: Simple function that prints out the cost of a trajectory. Returns a Vector of the distance each joint traveled


	*/


    double PartialShortcut(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr *planning_scene, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int numShortcutLoops)
	{
	  //Ensures that we have a valid Partial Shortcut Method name entered into the .yaml
	  if(planner_type != "Partial")
	  {
	    planner_type = "AdaptivePartial";
	  }
	  ROS_INFO("Using (shortcut namespace) %s Shortcut to clean up the trajectory",planner_type.c_str());
	  double time = ros::Time::now().toSec();
	  std::vector<trajectory_msgs::JointTrajectoryPoint>::size_type trajSize = joint_trajectory->points.size();
	  std::vector<double>::size_type numberDOFs = joint_trajectory->points[0].positions.size();;

	  //Ensure that the trajectory is valid.
	  if(joint_trajectory== nullptr || joint_trajectory->points.size() == 0)
	  {
	    return -1;
	  }

	  //Determine reference cost. This is cost from start to goal assuming no collisions. It is a vector.
	  Eigen::VectorXd refCostVector(joint_trajectory->points[0].positions.size());
	  for(int i = 0; i < refCostVector.size(); i++)
	  {
	    refCostVector(i) = joint_trajectory->points.back().positions[i] - joint_trajectory->points[0].positions[i];
	  }

	  //begin main algorithm loop
	  for(int loop_iter = 0; loop_iter < numShortcutLoops; loop_iter++) //TODO: Change 1 to numShortcutLoops
	  {
	    int dof;
	    if(planner_type == "Partial")
	    {
	      dof = (int)randomNumber(0,numberDOFs);
	    } else 
	    {
	      Eigen::VectorXd AWD = genAWD(refCostVector, joint_trajectory);
	      dof = selectDOF(AWD);
	    }
	    


	    int a; //variable marking random waypoint (beginning)
	    int b; //variable marking random waypoint (end) -> attempt to connect a to b

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

	      while(!(b > a && (b-a) > 1)) 
	      { 
	       b = rand()%(trajSize); //ensures that index b is after a
	      }

	    }

	    //The following calculates the cost of improvement of the degree of freedom
	    //Ensures that there is improvement to prevent re-doing an already improved part (checking for collisions is intensive)

	    double refCostab = 0.0;
	    for(unsigned iter = a; iter < b; iter++) {
	        refCostab += 
	          fabs((joint_trajectory->points[iter+1].positions[dof] - joint_trajectory->points[iter].positions[dof]));
	    }        

	    double costOfImprovement = fabs(joint_trajectory->points[b].positions[dof] - joint_trajectory->points[a].positions[dof]);


	    //the straightline cost improves the trajectory, so we are going to check if the path is valid
	    if(costOfImprovement < refCostab)
	    {
	      if(clearPathPartial(joint_trajectory, *planning_scene,joint_model_group, robot_state, a, b, dof)) {
	        //Officially change the joint

	        //check to see whether we actually replaced the points

	        double increment = (joint_trajectory->points[b].positions[dof] - joint_trajectory->points[a].positions[dof])/(b-a);
	        for(int i = 0; i <= b-a; i++)
	        {
	          joint_trajectory->points[a+i].positions[dof] = joint_trajectory->points[a].positions[dof] + increment*i;
	        }

	      }
	    }
	  }
	  return ros::Time::now().toSec() - time;

	}


/*

Function: clearPath


Purpose: Determines if the path between two nodes is clear. It begins by discretizing the the path between two nodes
         until the edgelength is between them is 1.0. Then, the collision checker checks each node (effectively, a robot
         state) to see if said state is in collision. Returns true if the path is collision-free.
*/
    


	bool clearPath(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int a, int b, int defaultNumPoints)
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

	void populatePath(trajectory_msgs::JointTrajectory *joint_trajectory)
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

	double Shortcut(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr *planning_scene, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int numShortcutLoops, int defaultNumPoints)
	{
	  double time = ros::Time::now().toSec();

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

	  return ros::Time::now().toSec()-time;
	}

	double RegularShortcut(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr *planning_scene, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int numShortcutLoops, int defaultNumPoints)
	{
	    ROS_INFO_STREAM("Using Regular Shortcut to clean up the Trajectory");
		double time = ros::Time::now().toSec();
		Shortcut(joint_trajectory, planning_scene, joint_model_group, robot_state, numShortcutLoops,defaultNumPoints);
		int startingNumPoints = joint_trajectory->points.size();
		int postShortcutNumPoints = joint_trajectory->points.size();
		populatePath(joint_trajectory);
		int postPopulatePoints = joint_trajectory->points.size();

		ROS_INFO("%s Shortcut method: %d -> %d -> %d points",planner_type.c_str(), startingNumPoints, postShortcutNumPoints, postPopulatePoints );

		return ros::Time::now().toSec()-time;
	}

	void Oracle(trajectory_msgs::JointTrajectory *joint_trajectory)
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
	       Eigen::VectorXd unitVec = firstVec + ((secondVec-firstVec)/2);
	       points->insert(firstIndex+i+1,trajectory_msgs::JointTrajectoryPoint());
	       std::vector<double> intermediatePositions(&unitVec[0], unitVec.data()+unitVec.cols()*unitVec.rows());
	       (*points)[i+1].positions = intermediatePositions;

	       trajSize =points->size();
	       firstIndex = points->begin();
	       i+=1;
	     }
	  }
	}

	double AdaptiveShortcut(trajectory_msgs::JointTrajectory *joint_trajectory, planning_scene::PlanningScenePtr *planning_scene, const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state, int numShortcutLoops, int defaultNumPoints)
	{
	    ROS_INFO_STREAM("Using Adaptive Regular Shortcut to clean up the Trajectory");
		double time = ros::Time::now().toSec();
		int startingNumPoints = joint_trajectory->points.size();

		for(int i = 0; i < adaptive_repetitions; i++) 
		{
			Shortcut(joint_trajectory, planning_scene, joint_model_group, robot_state, numShortcutLoops,defaultNumPoints);
			if(i != adaptive_repetitions-1)
			{
				Oracle(joint_trajectory);
			}
		}

		int postShortcutNumPoints = joint_trajectory->points.size();
		//populatePath(joint_trajectory);
		int postPopulatePoints = joint_trajectory->points.size();
		ROS_INFO("Shortcut method: %d -> %d -> %d points",startingNumPoints, postShortcutNumPoints, postPopulatePoints );

		return ros::Time::now().toSec()-time;
	}

	/*

	Function: checkIfPathHasCollisions


	Purpose: Uses both the created Collision Checker (created by Tariq, not the MoveIt! one) and also
	         double-checks the waypoints to see if the generated path creates collisions. Returns true if
	         there is a collision
	*/

	bool checkIfPathHasCollisions(moveit_msgs::MotionPlanResponse *response, planning_scene::PlanningScenePtr planning_scene,const robot_state::JointModelGroup *joint_model_group, robot_state::RobotState *robot_state)
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


}; //end class Shortcut_Planner
}

#endif
