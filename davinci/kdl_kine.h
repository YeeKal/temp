//#include <kdl_parser/kdl_parser.hpp>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <stdio.h>

Eigen::Affine3d transXYZRPY(double x,double y,double z,double roll,double pitch,double yaw ){
    Eigen::Affine3d trans_all;
    Eigen::Matrix3d rot;
    rot=(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()))* (Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))*(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::Vector3d pos;
    pos<<x,y,z;
    Eigen::Affine3d frame(Eigen::Translation3d(pos) * rot);
    return frame;
}
//homogeneous coordinates
class HomoKinematics{
private:
    unsigned int dim_;
    unsigned int link_num_;
    Eigen::MatrixXd xyz_rpy_;
    Eigen::MatrixXd xyz_rpy_change_;
public:

    HomoKinematics(double dim=4):dim_(dim),link_num_(9){
        ROS_INFO("homo kinematics start.");
        xyz_rpy_.resize(link_num_,6);
        xyz_rpy_change_=Eigen::MatrixXd::Zero(link_num_,6);
        xyz_rpy_<<0, 0.0569, 0.6348,-0.52333, 0, 0,//roll1
                 0, 0.11338, -0.17756,0.261799388, 0, 0,//roll2
                0.005, 0.39069, -0.05209,0,0,0,//link1
                0.04, 0.0, 0.200,0.17453, 0, 0,//link2
                0, 0.3, 0,-0.17453, 0, 0,//link3
                -0.045, -0.025, 0.200,0,0,0,//slide
                0,0.025,0,-1.5708, 0, 0,//pole
                0, 0.4, 0,0,0,0,//remote center
                0, 0.1, 0,0, 0, 0;//end_effector
        ROS_INFO("homo kinematics completed.");
    }
    void setJointValues(Eigen::VectorXd &joint_values){
        xyz_rpy_change_(0,4) =joint_values[0];//roll1-pitch
        xyz_rpy_change_(1,4) =joint_values[1];//roll2-pitch
        xyz_rpy_change_(2,3) =joint_values[2];//link1-roll
        xyz_rpy_change_(3,3) =-joint_values[2];//link2-roll
        xyz_rpy_change_(4,3) =joint_values[2];//link3-roll
        xyz_rpy_change_(5,2) =-joint_values[3];//slide-z
    }
    void getTransformAtIndex(int index, Eigen::Affine3d &frame){
        if(index>link_num_ || index<1){
            printf("invalid index.[1-9]");
        }
        frame=Eigen::Affine3d::Identity();
        for(int i=0;i<index;i++){
            frame=frame*
                  transXYZRPY(xyz_rpy_(i,0),xyz_rpy_(i,1),xyz_rpy_(i,2),xyz_rpy_(i,3),xyz_rpy_(i,4),xyz_rpy_(i,5))*
                  transXYZRPY(xyz_rpy_change_(i,0),xyz_rpy_change_(i,1),xyz_rpy_change_(i,2),xyz_rpy_change_(i,3),xyz_rpy_change_(i,4),xyz_rpy_change_(i,5));
        }
    }
    void getTransformAtRemoteCenter(Eigen::Affine3d &frame){
        getTransformAtIndex(8,frame);
    }
    void getTransformAtEnd(Eigen::Affine3d &frame){
        getTransformAtIndex(9,frame);
    }
    void getIk(double roll1_theta,Eigen::Affine3d frame_end,Eigen::VectorXd& joint_values){
        joint_values.resize(dim_);
        joint_values<<roll1_theta,0,0,0;
        setJointValues(joint_values);//set roll2 with 0
        /*roll2 transformation with theta=0,
        whith will be set to the reference transformation*/
        Eigen::Affine3d frame_rc;
        getTransformAtRemoteCenter(frame_rc);
        /*transformation of end transformed to roll2 coordinates at 0 */
        Eigen::Affine3d frame_to_rc;
        frame_to_rc=frame_rc.inverse()*frame_end;
        Eigen::Vector3d translation_relative=frame_to_rc.translation();
        double theta_roll2=std::atan2(-translation_relative(0),translation_relative(1));
        double translation_slide=translation_relative.norm();//cant equal to 0
        double theta_link1=std::asin(translation_relative(2)/translation_slide);
        joint_values[1]=theta_roll2;
        joint_values[2]=theta_link1;
        joint_values[3]=translation_slide-0.1;
    }
};
