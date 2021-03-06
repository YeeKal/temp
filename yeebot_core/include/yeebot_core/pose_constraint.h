#ifndef YEEBOT_POSE_CONSTRAINT_H
#define YEEBOT_POSE_CONSTRAINT_H

#include <ompl/base/Constraint.h>

#include "yeebot_core/kine_kdl.h"

namespace yeebot{
    
class PoseConstraint: public ompl::base::Constraint{
protected:
    Eigen::VectorXd invalid_vector_;    //invalid axis for xyz-rpy
    Eigen::Isometry3d ref_pose_;    //reference pose, default to Isometry3d::Identity()

    KineKdlConstPtr kine_kdl_;
public:
    PoseConstraint(Eigen::VectorXd invalid_vector,KineKdlPtr kine_kdl);

    /**
     * update manifold dimension according to invalid_vector_
     */ 
    void updateManifold();
    /**
     * reset invalid_vector_
     */ 
    void setInvalidVector(Eigen::VectorXd &x);
    void setRefPose(Eigen::Isometry3d &ref_pose);
    void getRefPose(Eigen::Isometry3d &ref_pose) const;

    void function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const override;
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::MatrixXd> out) const override;
    bool project(Eigen::Ref<Eigen::VectorXd> x)const override;


};//end class PoseConstraint
typedef std::shared_ptr<PoseConstraint> PoseConstraintPtr;
typedef std::shared_ptr<const PoseConstraint> PoseConstraintConstPtr;

}//end namespace yeebot

#endif