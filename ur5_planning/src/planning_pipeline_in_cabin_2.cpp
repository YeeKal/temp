/************************
*move group for ur5
*Author: yee
*Date: 2018-10-13
*************************/
#include "constrained_sampler_robot.h" 


int main(int argc,char **argv){
    ros::init(argc,argv,"planning_in_cabin");
    ros::NodeHandle node_handle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration sleep_time(10.0);
    
    
    //robot model 
    static std::string PLANNING_GROUP="manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
    const std::string &end_effector_name=joint_model_group->getLinkModelNames().back();
    const std::vector<std::string> &joint_names=joint_model_group->getVariableNames();
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

    sleep_time.sleep();
    namespace rvt=rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.loadRemoteControl();
    Eigen::Affine3d text_pose=Eigen::Affine3d::Identity();
    text_pose.translation().z()=0.75;
    visual_tools.publishText(text_pose," motion planning", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("next step");
    //ros 通信
    ros::Publisher display_publisher 
        =node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher execute_trajectory
        =node_handle.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 5, true);
  	//moveit_msgs::DisplayTrajectory display_trajectory;

	//ompl state space
    ompl_interface::ModelBasedStateSpaceSpecification model_ss_spec(robot_model, joint_model_group);
    const ompl_interface::JointModelStateSpaceFactory factory;
    ompl_interface::ModelBasedStateSpacePtr model_state_space = factory.getNewStateSpace(model_ss_spec);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(model_state_space));
    ob::ScopedState<> start(model_state_space),goal(model_state_space);
    double *start_values=new double[model_state_space->getDimension()];
    double *goal_values=new double[model_state_space->getDimension()];
    for(unsigned i=0;i<model_state_space->getDimension();i++){
        start_values[i]=0.0;
    }
    start->as<ompl_interface::ModelBasedStateSpace::StateType>()->values=start_values;
    std::vector<double> joint_values_target;
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x=0.707;
    target_pose.orientation.y=0.707;
    target_pose.position.x=0.3;
    target_pose.position.y=0;
    target_pose.position.z=0.2;

    //inverse and set goal values
    bool found_ik=false;
    for(int i=0;i<5;i++){
        found_ik=robot_state->setFromIK(joint_model_group,target_pose);
        if(found_ik){
            ROS_INFO("Success to solve ik.");
            robot_state->copyJointGroupPositions(joint_model_group,joint_values_target);
            visual_tools.publishAxisLabeled(target_pose, "target");//coordinates with 3 axis
            visual_tools.trigger();
            break;
        }
    }
    if(!found_ik)
    {
        ROS_INFO("Failed to solve ik.");
        visual_tools.publishText(text_pose,"end planning", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();
        visual_tools.deleteAllMarkers();
        ros::shutdown();
        return 0;
    }
    for(unsigned i=0;i<model_state_space->getDimension();i++){
        goal_values[i]=joint_values_target[i];
    }
    goal->as<ompl_interface::ModelBasedStateSpace::StateType>()->values=goal_values;
    //state sampler
    GeneralSampler *constraint_sampler=new GeneralSampler(joint_model_group,robot_state);
    int v[6]={0,0,0,1,1,1};
    Eigen::VectorXi invalid_vector;
    Eigen::VectorXd reference_pos(6);
    invalid_vector=Eigen::Map<Eigen::VectorXi>(v,6);
    Eigen::Quaterniond q(0,0.707,0.707,0);
    Eigen::Vector3d rpy;
    rpy=q.normalized().toRotationMatrix().eulerAngles(0,1,2);
    reference_pos[0]=0;
    reference_pos[1]=0;
    reference_pos[2]=0;
    //reference_pos.head(3)<<0.0,0.0,0;
    reference_pos[3]=rpy[0];
    reference_pos[4]=rpy[1];
    reference_pos[5]=rpy[2];
    constraint_sampler->setInvalidVector(invalid_vector);
    constraint_sampler->setRefVector(reference_pos);
    //end state sampler

    auto ss=std::make_shared<ompl::geometric::SimpleSetup>(si);
    ss->getStateSpace()->setStateSamplerAllocator(boost::bind(&allocConstrainedRobotSampler,model_state_space,constraint_sampler));
    ss->setStateValidityChecker(isValid);
    ss->setStartAndGoalStates(start,goal);
    auto pp=std::make_shared<ompl::geometric::RRTstar>(si);
    ss->setPlanner(pp);
    ss->setup();

    ob::PlannerStatus solution=ss->solve(5.0);
    if(!solution){
        ROS_INFO("Failed to find solution");
    }
    ss->simplifySolution(5.);//optimize the path
    ompl::geometric::PathGeometric path=ss->getSolutionPath();
    path.interpolate();
    //path.print(std::cout);

    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.model_id="ur5";
    robot_state->setJointGroupPositions(joint_model_group,start_values);//new robot_state::RobotState(kinematic_model);
    robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);

    moveit_msgs::RobotTrajectory robot_trajectory;
    robot_trajectory.multi_dof_joint_trajectory.header.frame_id="/world";
    robot_trajectory.joint_trajectory.header.frame_id="/world";
    robot_trajectory.joint_trajectory.header.stamp=ros::Time::now();
    robot_trajectory.joint_trajectory.joint_names=joint_names;

    
    for(std::size_t i=0;i<path.getStateCount();i++){
        std::vector<double> joint_values_path;
        geometry_msgs::Pose path_pose;
        for(int k=0;k<6;k++){
            joint_values_path.push_back(path.getState(i)->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[k]);
        }
        //publish axis
        robot_state->setJointGroupPositions(joint_model_group,joint_values_path);
        const Eigen::Affine3d &end_effector_position=robot_state->getGlobalLinkTransform(end_effector_name);
        tf::poseEigenToMsg(end_effector_position,path_pose);
        visual_tools.publishAxisLabeled(path_pose, std::to_string(i));
        visual_tools.trigger();

        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start=ros::Duration(0.01);
        point.positions=joint_values_path;
        robot_trajectory.joint_trajectory.points.push_back(point);
        //error
        Eigen::Vector3d rpy=end_effector_position.rotation().eulerAngles(0, 1, 2);
        rpy=rpy-reference_pos.tail(3);
        //satisfy bounds
        for(int i=0;i<3;i++){
            if(rpy[i]<-boost::math::constants::pi<double>())
                rpy[i] +=2*boost::math::constants::pi<double>();
            if(rpy[i]>boost::math::constants::pi<double>())
                rpy[i] -=2*boost::math::constants::pi<double>();
        }
        ROS_INFO("%d: %f %f %f",(int)i,rpy[0],rpy[1],rpy[2]);
    }
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    //move
    // ros::Duration d(0.5);
    // for(std::size_t i=0;i<path.getStateCount();i++){
    //     std::vector<double> joint_values_path;
    //     for(int k=0;k<6;k++){
    //         joint_values_path.push_back(path.getState(i)->as<ompl_interface::ModelBasedStateSpace::StateType>()->values[k]);
    //     }
    //     sensor_msgs::JointState joint_state;
    //     joint_state.header.frame_id="/world";
    //     joint_state.header.stamp=ros::Time::now();
    //     for(int i=0;i<joint_names.size();i++){
    //         joint_state.name.push_back(joint_names[i]);
    //         joint_state.position.push_back(joint_values_path[i]);
    //     }
    //     execute_trajectory.publish(joint_state);
    //     d.sleep();
    //     ROS_INFO("PUBLISH %d",(int)i);
    // }
    
    


    //release memory
    sleep(5.0);
    visual_tools.deleteAllMarkers();

    ros::shutdown();
    return 0;
}