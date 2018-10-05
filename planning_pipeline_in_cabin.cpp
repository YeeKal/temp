/************************
*move group for ur5
*Author: yee
*Date: 2018-09-29
*************************/

//collision object from stl
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit/move_group/capability_names.h>


//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <actionlib/client/simple_action_client.h>

int main(int argc,char **argv){
    ros::init(argc,argv,"planning_in_cabin");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration sleep_time(2.0);
    

    static std::string PLANNING_GROUP="manipulator";
    // moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // bool success;
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

	ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  	moveit_msgs::DisplayTrajectory display_trajectory;


    	// We will get the name of planning plugin we want to load
	// from the ROS param server, and then load the planner
	// making sure to catch all exceptions.

	// if (!node_handle.getParam("planning_plugin", planner_plugin_name))
	// 	ROS_FATAL_STREAM("Could not find planner plugin name");
    planner_plugin_name="ompl_interface/OMPLPlanner";
    //ROS_INFO("name space %s",node_handle.getNamespace().c_str());
	try
	{
		planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
			"moveit_core", "planning_interface::PlannerManager"));
	}
    //planner_plugin_name
    
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

    namespace rvt=rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose=Eigen::Affine3d::Identity();
    text_pose.translation().z()=0.75;
    visual_tools.publishText(text_pose," motion planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    //visual_tools.prompt("next step");

    //define pose goal
	planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
	geometry_msgs::PoseStamped pose;
	pose.header.frame_id="world";
	pose.pose.orientation.w=1.0;
    pose.pose.position.x=0.3;
    pose.pose.position.y=0;
    pose.pose.position.z=0.2;
	std::vector<double> tolerance_pose(3,0.01);
	std::vector<double> tolerance_angle(3,0.01);
    visual_tools.publishAxisLabeled(pose.pose, "pose1");
    visual_tools.trigger();

	req.group_name=PLANNING_GROUP;
	moveit_msgs::Constraints pose_goal=kinematic_constraints::constructGoalConstraints("ee_link",pose,tolerance_pose,tolerance_angle);
	req.goal_constraints.push_back(pose_goal);

	planning_interface::PlanningContextPtr context=planner_instance->getPlanningContext(planning_scene,req,res.error_code_);
	context->solve(res);
	if (res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
		return 0;
	}

    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    display_publisher.publish(display_trajectory);

    //publish path
    
    //ros::Duration(1.0).sleep();
    //execute
    sleep_time.sleep();
    ROS_INFO("start execute.");
    std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> > execute_action_client;
    execute_action_client.reset(new actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>(node_handle,move_group::EXECUTE_ACTION_NAME, false));
    moveit_msgs::ExecuteTrajectoryGoal goal;
    goal.trajectory=response.trajectory;
    ROS_INFO("start execute.");
    if (!execute_action_client->isServerConnected()){
        execute_action_client->sendGoal(goal);
    }
    if (execute_action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("success to execute.");
    }
    else{
        ROS_INFO("FAILED TO EXECUTE.");
    }

    //release memory
    visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}