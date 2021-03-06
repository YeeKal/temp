#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
//#include <geometry_msgs/Pose.h>


//moveit
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>




#include "constraint_torus.h"





bool isValid(const ob::State *state) {
    const ob::TorusStateSpace::StateType *state_torus=state->as<ob::TorusStateSpace::StateType>();
    std::vector<double> thetas;
    state_torus->getJoints(thetas);
    return true;
}

int main(int argc,char **argv){
    ros::init(argc,argv,"ur5_planning_ompl_constraint");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(50);

    static std::string PLANNING_GROUP="manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    const robot_state::JointModelGroup *joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names=joint_model_group->getVariableNames();
    const std::string &end_effector_name=move_group.getEndEffectorLink();
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    

    namespace rvt=rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose=Eigen::Affine3d::Identity();
    text_pose.translation().z()=0.75;
    visual_tools.publishText(text_pose," ur5 ompl planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    //visual_tools.prompt("next step");

    // ros::Publisher pub_jointstates = nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);
    
    // sensor_msgs::JointState ur5_jointstates;

    //start joint values
    std::vector<double> joint_values_start;
    std::vector<double> joint_values_target;
    move_group.getCurrentState()->copyJointGroupPositions(joint_model_group,joint_values_start);
    const Eigen::Affine3d &end_effector_position=kinematic_state->getGlobalLinkTransform(end_effector_name);
   // ROS_INFO_STREAM("start transform:\n"<<end_effector_position.matrix()<<"\n");
    //test
    

    //target joint values
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x= 0.707;
    target_pose.orientation.y=0.707;
  //target_pose.orientation.w=1.0;
    target_pose.position.x = 0.434;
    target_pose.position.y = 0.357;
    target_pose.position.z = 0.4066;
    bool found_ik=false;
    found_ik=kinematic_state->setFromIK(joint_model_group,target_pose);
    if(found_ik){
        ROS_INFO("Success to solve ik.");
        kinematic_state->copyJointGroupPositions(joint_model_group,joint_values_target);
        visual_tools.publishAxisLabeled(target_pose, "target");//coordinates with 3 axis
        visual_tools.trigger();
    }
    else
    {
        ROS_INFO("Failed to solve ik.");
        visual_tools.publishText(text_pose,"end planning", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();
        visual_tools.deleteAllMarkers();
        ros::shutdown();
        return 0;
    }
/*
    //construct constraint problem
    Eigen::VectorXd constraint_vector(6);
    // contraint_vector(0)=0;
    // contraint_vector(1)=0;
    // contraint_vector(2)=0;
    // contraint_vector(3)=1;
    // contraint_vector(4)=1;
    // contraint_vector(5)=0;
    constraint_vector << 0,0,0,1,1,0;
    auto space=std::make_shared<ob::TorusStateSpace>(6);

    //test
    auto *state=space->allocState();
    //double* 
    auto constraint=std::make_shared<ob::AxisConstraint>(end_effector_position,constraint_vector,kinematic_state,joint_model_group);
    //auto constraint=std::make_shared<ob::AxisConstraint>(constraint_vector,robot_state,joint_model_group);
    auto constraint_space=std::make_shared<ob::ProjectedStateSpace>(space,constraint);
    auto constraint_si=std::make_shared<ompl::base::ConstrainedSpaceInformation>(constraint_space);
    auto ss=std::make_shared<ompl::geometric::SimpleSetup>(constraint_si);
    ss->setStateValidityChecker(isValid);


    ompl::base::ScopedState<> start(constraint_space);
    ompl::base::ScopedState<> goal(constraint_space);
    Eigen::VectorXd start_joints,target_joints;
    start_joints=Eigen::VectorXd::Map(&joint_values_start[0],joint_values_start.size());
    target_joints=Eigen::VectorXd::Map(&joint_values_target[0],joint_values_target.size());
    start->as<ob::ConstrainedStateSpace::StateType>()->copy(start_joints);
    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(target_joints);
    ss->setStartAndGoalStates(start,goal);
    auto pp=std::make_shared<ompl::geometric::RRTstar>(constraint_si);
    ss->setPlanner(pp);
    ss->setup();
    ompl::base::PlannerStatus solution=ss->solve(5.0);
    if(solution){
        //ss->simplifySolution(5.);//optimize the path
        ompl::geometric::PathGeometric path=ss->getSolutionPath();
        path.interpolate();
       // path.print(std::cout);

        moveit_msgs::DisplayTrajectory display_trajectory;
        display_trajectory.model_id="ur5";
        robot_state::RobotStatePtr start_state=move_group.getCurrentState();//new robot_state::RobotState(kinematic_model);
        robot_state::robotStateToRobotStateMsg(*start_state, display_trajectory.trajectory_start);

        moveit_msgs::RobotTrajectory robot_trajectory;
        robot_trajectory.multi_dof_joint_trajectory.header.frame_id="/world";
        robot_trajectory.joint_trajectory.header.frame_id="/world";
        robot_trajectory.joint_trajectory.header.stamp=ros::Time::now();
        robot_trajectory.joint_trajectory.joint_names=joint_names;

        
        for(std::size_t i=0;i<path.getStateCount();i++){
            std::vector<double> joint_values_path;
            geometry_msgs::Pose path_pose;
            path.getState(i)->as<ob::TorusStateSpace::StateType>()->getJoints(joint_values_path);

            kinematic_state->setJointGroupPositions(joint_model_group,joint_values_path);
            const Eigen::Affine3d &end_effector_position=kinematic_state->getGlobalLinkTransform(end_effector_name);
            tf::poseEigenToMsg(end_effector_position,path_pose);
            visual_tools.publishAxisLabeled(path_pose, "path"+std::to_string(i));
            visual_tools.trigger();

            trajectory_msgs::JointTrajectoryPoint point;
            point.time_from_start=ros::Duration(0.01);
            point.positions=joint_values_path;
            robot_trajectory.joint_trajectory.points.push_back(point);
            i++;
        }
        display_trajectory.trajectory.push_back(robot_trajectory);
        display_publisher.publish(display_trajectory);
    }
    else
    {
        ROS_INFO("Failed to find solution");
    }
    
    sleep(5.0);
*/
    

    visual_tools.deleteAllMarkers();
    ros::shutdown();
    return 0;
}