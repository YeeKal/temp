#include "yeebot_core/kine_kdl.h"
#include <kdl/solveri.hpp>


namespace yeebot{
    KineKdl::KineKdl(const std::string& urdf_param,const std::string& base_name,const std::string& tip_name)
    :KineBase(),
    base_name_(base_name),tip_name_(tip_name),
    max_iter_(100),eps_(1e-6),project_error_(1e-4){
        robot_model_.initParam(urdf_param);
        if(!kdl_parser::treeFromUrdfModel(robot_model_,tree_)){
            std::cout<<"error!failed to initialize kdl tree from urdf model."<<std::endl;
        }
        tree_.getChain(base_name_,tip_name_,chain_);
        link_num_=chain_.getNrOfSegments();
        joint_num_=chain_.getNrOfJoints();
        link_names_.resize(link_num_);
        joint_names_.resize(joint_num_);
        joint_limits_.resize(joint_num_,2);

        for(int i=0,j=0;i<link_num_;i++){
            const KDL::Segment &seg=chain_.getSegment(i);
            const KDL::Joint& jnt = seg.getJoint();
            link_names_[i]=seg.getName();
            if(jnt.getType()==KDL::Joint::None) 
                continue;

            joint_names_[j]=jnt.getName();
            urdf::JointConstSharedPtr joint = robot_model_.getJoint(jnt.getName());
            joint_limits_(j,0)=joint->limits->lower;
            joint_limits_(j,1)=joint->limits->upper;
            j++;
        }
        KDL::JntArray joints_min,joints_max;
        Eigen2KDL(joint_limits_.col(0),joints_min);
        Eigen2KDL(joint_limits_.col(1),joints_max);

        fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
        iksolver_vel_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
        iksolver_nr_jl_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_,joints_min,joints_max,*fksolver_,*iksolver_vel_,max_iter_,eps_));
    }
    bool KineKdl::solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values)const{
        return solveFK(pose,joint_values,-1);
    }
    bool KineKdl::solveFK(Eigen::Isometry3d & pose, const std::string& link_name, const Eigen::Ref<const Eigen::VectorXd> &joint_values)const{
        if(link_name==base_name_){
            pose.setIdentity();
            return true;
        }
        //find the link name index
        for(unsigned int i=0;i<link_num_;i++){
            if(link_name==link_names_[i]){
                return solveFK(pose,joint_values,i+1);
            }
        }
    }

    bool KineKdl::solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values,int link_num)const{
        KDL::JntArray kdl_joints;
        Eigen2KDL(joint_values,kdl_joints);
        KDL::Frame kdl_pose;

        int error_status=fksolver_->JntToCart(kdl_joints,kdl_pose,link_num);
        if(getError(error_status)){
            KDL2Eigen(kdl_pose,pose);
            return true;
        }
        return false;
    }
    
    bool KineKdl::solveIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &joint_values,const Eigen::Isometry3d &pose)const{
        KDL::JntArray kdl_joints_in;//ik initial joint values
        Eigen2KDL(joint_in,kdl_joints_in);
        KDL::Frame kdl_pose;
        Eigen2KDL(pose,kdl_pose);
        KDL::JntArray kdl_joints=KDL::JntArray(joint_num_);//ik result

        int error_status=iksolver_nr_jl_->CartToJnt(kdl_joints_in,kdl_pose,kdl_joints);
        if(getError(error_status)){
            KDL2Eigen(kdl_joints,joint_values);
            return true;
        }
        return false;
    }

    bool KineKdl::calcJac(Eigen::Ref<Eigen::MatrixXd> &jacobian, const Eigen::Ref<const Eigen::VectorXd> &joint_values )const{
        KDL::JntArray kdl_joints;
        Eigen2KDL(joint_values,kdl_joints);
        KDL::Jacobian kdl_jac(joint_num_);

        int error_status=jac_solver_->JntToJac(kdl_joints,kdl_jac);
        if(getError(error_status)){
            KDL2Eigen(kdl_jac,jacobian);
            return true;
        }
        return false;
    }
   

    bool KineKdl::axisProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        //convert eigen to kdl
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        
        //pre-declaration frequently used variables 
        KDL::Frame kdl_new_pose;
        KDL::Twist delta_twist;
        KDL::JntArray delta_jnt(joint_num_);

        KDL::JntArray kdl_jnt_out(joint_num_);
        kdl_jnt_out=kdl_jnt_in;
        int error_status;
        double twist_error;
        for(unsigned int i=0;i<max_iter_;i++){
            //fk to get new pose
            if(error_status=fksolver_->JntToCart(kdl_jnt_out,kdl_new_pose)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::Twist(eleMulti(KDL::diff(kdl_ref_pose.p,kdl_new_pose.p),invalid_xyz),  
                                   eleMulti(KDL::diff(kdl_ref_pose.M,kdl_new_pose.M),invalid_rpy));
            
            //presudoinverse of jacobian to get delta joint values 
            if(error_status=iksolver_vel_->CartToJnt(kdl_jnt_out,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                std::cout<<"vel"<<std::endl;
                return getError(error_status);
            }

            KDL::Subtract(kdl_jnt_out,delta_jnt,kdl_jnt_out);

            //calc error  
            // twist_error=0;
            // for(int i=0;i<6;i++){
            //     twist_error +=delta_twist[i]*delta_twist[i];
            // }
            //std::cout<<"error:"<<twist_error<<std::endl;

            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),project_error_)){
                KDL2Eigen(kdl_jnt_out,jnt_out);
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
        
    }

     //for function in constraint space
    void KineKdl::function(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> err_out) const{
        //err_out should have the correct size
        //the validity of size won't be verified here
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);

        KDL::Twist delta_twist;
        KDL::Frame kdl_new_pose;

        //fk to get new pose
        fksolver_->JntToCart(kdl_jnt_in,kdl_new_pose);//get new frame

        //differential of the frame 
        delta_twist=KDL::Twist(KDL::diff(kdl_ref_pose.p,kdl_new_pose.p),  
                               KDL::diff(kdl_ref_pose.M,kdl_new_pose.M));

        //get the invalid velocity according to the invalid vector
        int invalid_num=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis(i)>0){//TODO:change 0 to epsilon
                err_out(invalid_num)=delta_twist(i);
                invalid_num ++;
            }
        }//end for
    }
    //for jacobian in constraint space
    void KineKdl::jacobian(const Eigen::Ref<const Eigen::VectorXd>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::MatrixXd> jac_out)const {
        Eigen::MatrixXd jac(6,joint_num_);
        //calcJac(jac,jnt_in);
        KDL::JntArray kdl_joints;
        Eigen2KDL(jnt_in,kdl_joints);
        KDL::Jacobian kdl_jac(joint_num_);
        jac_solver_->JntToJac(kdl_joints,kdl_jac);
        KDL2Eigen(kdl_jac,jac);

        int invalid_num=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis(i)>0){//TODO:change 0 to epsilon
                jac_out.row(invalid_num)=jac.row(i);
                invalid_num ++;
            }
        }//end for
    }

    /*
    comparation between KDL-SVD and EIgen-SVD.
    kdl is faster(1000,0.4s) but more times to fail.
    eigen is slower(1000,1.6s) but less times to fail.
    */
    bool KineKdl::project(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd>  jnt_out) const{
         //convert eigen to kdl
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        
        //pre-declaration frequently used variables 
        KDL::Frame kdl_new_pose;
        KDL::Twist delta_twist;
        //KDL::JntArray delta_jnt(joint_num_);
        Eigen::VectorXd delta_jnt(joint_num_);

        KDL::JntArray kdl_jnt_out(joint_num_);
        kdl_jnt_out=kdl_jnt_in;
        jnt_out=jnt_in;

        int invalid_count=0;
        for(int i=0;i<6;i++){
            if(invalid_axis(i)>0){
                invalid_count++;
            }
        }
        Eigen::MatrixXd jac(invalid_count,joint_num_);
        Eigen::MatrixXd jac_inv(joint_num_,invalid_count);
        Eigen::VectorXd err_vel(invalid_count);
        KDL::Jacobian kdl_jac(joint_num_);
        int error_status;
        double twist_error;
        for(unsigned int i=0;i<max_iter_;i++){
            function(ref_pose,invalid_axis,jnt_out,err_vel);
            jacobian(invalid_axis,jnt_out,jac);
            //inverse jac
            dampedPInv(jac,jac_inv);
            delta_jnt=jac_inv*err_vel;
            jnt_out=jnt_out-delta_jnt;  

            //calc error  
            // twist_error=0;
            // for(int i=0;i<invalid_count;i++){
            //     twist_error +=err_vel[i]*err_vel[i];
            // }
            //std::cout<<"error:"<<twist_error<<std::endl;

            bool complete=true;
            for(int k=0;k<invalid_count;k++){
                if(fabs(err_vel[k])>project_error_){
                    complete=false;
                    break;
                }
            }
            if(complete){
                return getError(KDL::SolverI::E_NOERROR);
            }
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
    }

    KDL::Vector KineKdl::eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const{
        return KDL::Vector(vec1.data[0]*vec2.data[0],vec1.data[1]*vec2.data[1],vec1.data[2]*vec2.data[2]);
    }

    bool KineKdl::getError(const int error_status)const{
        //TODO:process error
        if(error_status<0){
            //std::cout<<"error:kin"<<error_status<<std::endl;
            return false;
        }
        else if(error_status>0){
            //std::cout<<"warning:"<<std::endl;
            return true;
        }
        return true;
    }

}//end namesapce yeebot