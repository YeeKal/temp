#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/tools/config/MagicConstants.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/base/StateSampler.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
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
#include <moveit/robot_state/conversions.h>

#include <moveit/ompl_interface/parameterization/model_based_state_space.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space_factory.h>

//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
//#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>
#include <boost/scoped_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <Eigen/Dense>

using namespace std;
namespace ob=ompl::base;
class GeneralSampler;

bool isValid(const ob::State *state) {
    const ompl_interface::ModelBasedStateSpace::StateType *state_torus=state->as<ompl_interface::ModelBasedStateSpace::StateType>();
    std::vector<double> thetas;
    return true;
}



namespace
{
void ikCallbackFnAdapter(robot_state::RobotState* state, const robot_model::JointModelGroup* jmg,
                                 const robot_state::GroupStateValidityCallbackFn& constraint,
                                 const geometry_msgs::Pose& /*unused*/, const std::vector<double>& ik_sol,
                                 moveit_msgs::MoveItErrorCodes& error_code)
{
  const std::vector<unsigned int>& bij = jmg->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[i] = ik_sol[bij[i]];
  if (constraint(state, jmg, &solution[0]))
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
}
}

class GeneralSampler{
public:
    /*
    PlanningSceneConstPtr scene_;
    JointModelGroup* jmg_;
    robot_state::GroupStateValidityCallbackFn group_state_validity_callback_;
    */
    const robot_state::JointModelGroup* jmg_;
    robot_state::GroupStateValidityCallbackFn group_state_validity_callback_;
    kinematics::KinematicsBaseConstPtr kb_;
    double ik_timeout_;
    random_numbers::RandomNumberGenerator random_number_generator_;
    robot_state::RobotStatePtr robot_state_;
    int dim_;
    Eigen::VectorXi invalid_vector_;
    Eigen::VectorXd reference_pos_;
    Eigen::VectorXd tolerance_;
    int max_iterations_;

    GeneralSampler(const robot_state::JointModelGroup* joint_model_group,robot_state::RobotStatePtr robot_state)
    :jmg_(joint_model_group),robot_state_(robot_state),max_iterations_(50)
    {
        kb_=jmg_->getSolverInstance();
        dim_=jmg_->getVariableCount();
        double a[6]={0.1,0.1,0.1,0.1,0.1,0.1};
        tolerance_=Eigen::Map<Eigen::VectorXd>(a,6);
    }

    //project a state to the constrained space
    bool sample(ompl::base::State *state) const{
        Eigen::VectorXd values;
        values=Eigen::Map<Eigen::VectorXd>(state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values,dim_);
        //ROS_INFO("%f %f",values[0],values[1]);
        if(project(values)){
            //ROS_INFO("%f %f",values[0],values[1]);
            Eigen::Map<Eigen::VectorXd>(state->as<ompl_interface::ModelBasedStateSpace::StateType>()->values,dim_)=values;
            return true;
        }
        return false;
    }
    /*
    get jacobian from joints
    */
    void jacobian(Eigen::VectorXd &x, Eigen::MatrixXd jacobian)const{
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        robot_state_->getJacobian(jmg_,
            robot_state_->getLinkModel(jmg_->getLinkModelNames().back()),
            reference_point_position, jacobian);
    }
    //constraint function: error
    void function(Eigen::VectorXd &x, Eigen::VectorXd out)const{
        std::vector<double> values;
        for(int i=0;i<dim_;i++){
            values.push_back(x[i]);
        }
        robot_state_->copyJointGroupPositions(jmg_, values);
        const Eigen::Affine3d &end_effector_state = robot_state_->getGlobalLinkTransform(jmg_->getLinkModelNames().back());
        Eigen::Vector3d pos=end_effector_state.translation();
        Eigen::Vector3d rpy=end_effector_state.rotation().eulerAngles(0, 1, 2);
        Eigen::VectorXd new_pose(6);
        new_pose << pos,rpy;
        new_pose=new_pose-reference_pos_;
        //satisfy bounds
        for(int i=3;i<6;i++){
            if(new_pose[i]<-boost::math::constants::pi<double>())
                new_pose[i] +=2*boost::math::constants::pi<double>();
            if(new_pose[i]>boost::math::constants::pi<double>())
                new_pose[i] -=2*boost::math::constants::pi<double>();
        }
        for(int i=0;i<6;i++){
            out[i]=new_pose[i]*invalid_vector_[i];
        }
    }
    /*
    project a state to the constrained space
    */
    bool project(Eigen::VectorXd &x)const {
        int iter=0;
        Eigen::VectorXd error(dim_);
        Eigen::MatrixXd jac(6,dim_);
        function(x,error);
        while(!satisfyTolerance(error) && iter++<max_iterations_){
            jacobian(x,jac);
            x -=jac.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(error);
            function(x,error);
        }
        return satisfyTolerance(error);
    }
    /*
    check whether the error satisfy the tolerance
    */
    bool satisfyTolerance(Eigen::VectorXd &error)const{
        for(int i=0;i<error.size();i++){
            if(error[i]<-tolerance_[i] || error[i]>tolerance_[i]){
                return false;
            }
        }
        return true;
    }
    bool callIK(const geometry_msgs::Pose& pose,std::vector<double> joint_values,int max_attempts) const
    {
        // kinematics::KinematicsBase::IKCallbackFn adapted_ik_validity_callback;
        // if (group_state_validity_callback_){
        //     adapted_ik_validity_callback =boost::bind(&ikCallbackFnAdapter, robot_state_.get(), jmg_, group_state_validity_callback_, _1, _2, _3);
        //     ROS_INFO("IT IS TRUE ");
        // }
        if(robot_state_->setFromIK(jmg_,pose)){
            robot_state_->copyJointGroupPositions(jmg_,joint_values);
            return true;
        }
        return false;
    }
    void setInvalidVector(Eigen::VectorXi &x){
        invalid_vector_=x;
    }
    void setRefVector(Eigen::VectorXd &x){
        reference_pos_=x;
    }

};

namespace ompl{
    namespace base{
class ConstrainedSamplerRobot : public StateSampler{
public:
    const robot_state::JointModelGroup* jmg_;
    GeneralSampler* constraint_sampler_;
    ompl::base::StateSamplerPtr                       default_;
    unsigned int                                      constrained_success_;
    unsigned int                                      constrained_failure_;
    int sample_attempts_;
    double                                            inv_dim_;

    ConstrainedSamplerRobot(const ompl_interface::ModelBasedStateSpacePtr &state_space,GeneralSampler* cs);
    void sampleUniform(ompl::base::State *state);
    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, const double distance);
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, const double stdDev);
    bool sampleC(ompl::base::State *state);
};
}}
ompl::base::ConstrainedSamplerRobot::ConstrainedSamplerRobot(const ompl_interface::ModelBasedStateSpacePtr &state_space,GeneralSampler* cs)
    : ompl::base::StateSampler(state_space.get()),
    jmg_(space_->as<ompl_interface::ModelBasedStateSpace>()->getJointModelGroup()),
    constraint_sampler_(cs),
    sample_attempts_(5),
    constrained_failure_(0),
    constrained_success_(0)
{   
    default_=space_->allocDefaultStateSampler();
    inv_dim_=1.0/(double)space_->getDimension();
}

bool ompl::base::ConstrainedSamplerRobot::sampleC(ob::State *state){
        
    int iter=0;
    while(iter++<sample_attempts_){
        default_->sampleUniform(state);
        if(constraint_sampler_->sample(state)){
            if(space_->satisfiesBounds(state)){
                ++constrained_success_;
                return  true;
            }
            else{
                continue;
            }
        }

    }
    ++constrained_failure_;
    return false;
}

void ompl::base::ConstrainedSamplerRobot::sampleUniform(ompl::base::State* state){
    if (!sampleC(state) && !sampleC(state) && !sampleC(state)){
        //default_->sampleUniform(state);
        //ROS_INFO("+1");
    }
}

void ompl::base::ConstrainedSamplerRobot::sampleUniformNear(ob::State *state, const ob::State *near, const double distance){
    if (sampleC(state) || sampleC(state) || sampleC(state))
    {
        double total_d = space_->distance(state, near);
        if (total_d > distance)
        {
            double dist = pow(rng_.uniform01(), inv_dim_) * distance;
            space_->interpolate(near, state, dist / total_d, state);
        }
    }
    else
    default_->sampleUniformNear(state, near, distance);
}

void ompl::base::ConstrainedSamplerRobot::sampleGaussian(ob::State *state, const ob::State *mean, const double stdDev)
{
    if (sampleC(state) || sampleC(state) || sampleC(state))
    {
        double total_d = space_->distance(state, mean);
        double distance = rng_.gaussian(0.0, stdDev);
        if (total_d > distance)
        {
            double dist = pow(rng_.uniform01(), inv_dim_) * distance;
            space_->interpolate(mean, state, dist / total_d, state);
        }
    }
    else
        default_->sampleGaussian(state, mean, stdDev);
}

ompl::base::StateSamplerPtr allocConstrainedRobotSampler(ompl_interface::ModelBasedStateSpacePtr &state_space,GeneralSampler* cs){
    return ompl::base::StateSamplerPtr(new ompl::base::ConstrainedSamplerRobot(state_space,cs));
}