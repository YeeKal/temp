//ompl
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

#include <boost/math/constants/constants.hpp>
#include <boost/shared_ptr.hpp>

#include <ompl/config.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>

using namespace std;
namespace ob=ompl::base;

namespace ompl{
    namespace base{
		class TorusStateSampler: public StateSampler{
		public:
			TorusStateSampler(const StateSpace *space): StateSampler(space){}
			void sampleUniform(State *state) override;
			void sampleUniformNear(State *state, const State *near, double distance) override;
			void sampleGaussian(State *state, const State *mean, double stdDev) override;
		};
        class TorusStateSpace : public StateSpace{
        protected:
            unsigned int dim_;
            RealVectorBounds bounds_;
            std::size_t stateBytes_;
            bool continuous_;
        public:
            class StateType : public State{
                public:
                    double *values;
                    StateType()=default;
                    void setJoints(std::vector<double> &thetas){
                        values=new double[thetas.size()];
                        for(int i=0;i<thetas.size();i++){
                            values[i]=thetas[i];
                        }
                    }
                    
                    void getJoints(std::vector<double> &thetas) const{
                        if(values==NULL){
                            cout<<"values are not assigned.";
                            return;
                        }
                        int dim=sizeof(values)/sizeof(values[0]);
                        for(int i=0;i<dim;i++){
                            thetas.push_back(values[i]);
                        }                            
                    }
                    double operator[](unsigned int i) const
                    {
                        return values[i];
                    }
                    double &operator[](unsigned int i)
                    {
                        return values[i];
                    }
            };
            TorusStateSpace(unsigned int dim):dim_(dim),bounds_(dim){
                setName("Torus"+getName());
                //type_ = STATE_SPACE_SO2;
                continuous_=true;
                setBounds(-boost::math::constants::pi<double>(),boost::math::constants::pi<double>());
                stateBytes_=dim_*sizeof(double);
            }
            ~TorusStateSpace() override=default;
            void setContinuous(bool conti){
                continuous_=conti;
            }
            State* allocState() const{
                auto *state = new StateType();
                state->values=new double[dim_];
                return state;
            }
            void freeState(State* state) const override{
                auto *rstate = static_cast<StateType *>(state);
                delete[] rstate->values;
                delete rstate;
            }
            bool equalStates(const State *state1,const State* state2) const override{
                const double *s1 = static_cast<const StateType *>(state1)->values;
                const double *s2 = static_cast<const StateType *>(state2)->values;
                for (unsigned int i = 0; i < dim_; ++i)
                {
                    double diff = (*s1++) - (*s2++);
                    if (fabs(diff) > std::numeric_limits<double>::epsilon() * 2.0)
                        return false;
                }
                return true;
            }
            void setBounds(const RealVectorBounds &bounds) {
                bounds.check();
                if (bounds.low.size() != dim_)
                    throw Exception("Bounds do not match dimension of state space: expected dimension " +
                                    std::to_string(dim_) + " but got dimension " + std::to_string(bounds.low.size()));
                bounds_ = bounds;
                //continuous_=false;
            }
            void setBounds(double low, double high)
            {
                RealVectorBounds bounds(dim_);
                bounds.setLow(low);
                bounds.setHigh(high);
                setBounds(bounds);
            }

            void enforceBounds(State *state) const override{
                //double v = fmod(state->as<StateType>()->value, 2.0 * boost::math::constants::pi<double>());
                double *values=state->as<StateType>()->values;
                if(continuous_){
                    for(int i=0;i<dim_;i++){
                        if(values[i]<-boost::math::constants::pi<double>())
                            values[i] +=2*boost::math::constants::pi<double>();
                        if(values[i]>boost::math::constants::pi<double>())
                            values[i] -=2*boost::math::constants::pi<double>();
                    }
                }
                else{
                    for(int i=0;i<dim_;i++){
                        if(values[i]<bounds_.low[i])
                            values[i]=bounds_.low[i];
                        if(values[i]>bounds_.high[i])
                            values[i]=bounds_.high[i];
                    }
                }

            }

            bool satisfiesBounds(const State *state) const override{
                for(int i=0;i<dim_;i++){
                    if(state->as<StateType>()->values[i]<bounds_.low[i] || state->as<StateType>()->values[i]>bounds_.high[i])
                        return false;
                }
                return true;
            }
            void copyState(State *destination,const State* source)const {
                //memcpy(static_cast<StateType *>(destination)->values,static_cast<StateType *>(source)->values,stateBytes_);
                memcpy(destination->as<StateType>()->values,source->as<StateType>()->values,stateBytes_);
                //void *memcpy(void*dest, const void *src, size_t n);
                //由src指向地址为起始地址的连续n个字节的数据复制到以destin指向地址为起始地址的空间内
            }
            unsigned int getDimension() const override{
                return dim_;
            }
            double getMeasure() const override{
                double m=1.0;
                for(int i=0;i<dim_;i++){
                    m*=bounds_.high[i]-bounds_.low[i];
                }
                return m;
            }
            double getMaximumExtent()const override{
                double e=0.0;
                for(int i=0;i<dim_;i++){
                    double d=bounds_.high[i]-bounds_.low[i];
                    e +=d;
                }
                return e;
            }

            const RealVectorBounds & getBounds() const{
                return bounds_;
            }

            void serialize(void *serialization,const State *state) const override{
                memcpy(serialization,state->as<StateType>()->values,stateBytes_);
            }
            void deserialize(State* state,const void *serialization) const override{
                memcpy(state->as<StateType>()->values,serialization,stateBytes_);
            }
            double distance(const State *state1,const State* state2) const override{
                const double *s1 = static_cast<const StateType *>(state1)->values;
                const double *s2 = static_cast<const StateType *>(state2)->values;
                double dist=0;
                if(continuous_){
                    for (unsigned int i = 0; i < dim_; ++i)
                    {	
                        double diff = fmod(fabs(s1[i]-s2[i]),2.0*boost::math::constants::pi<double>());
                        diff=((diff > boost::math::constants::pi<double>()) ? 2.0 * boost::math::constants::pi<double>() - diff : diff);
                        dist += diff;
                    }
                }
                else{
                    for (unsigned int i = 0; i < dim_; ++i)
                    {
                        dist +=fabs(s1[i]-s2[i]);
                    }
                }
                return dist;
            }
            void interpolate(const State *from,const State *to,const double t,State *state)const override{
                for(int i=0;i<dim_;i++){
                    double diff=to->as<StateType>()->values[i]-from->as<StateType>()->values[i];
                    if(continuous_){
                        if (fabs(diff) <= boost::math::constants::pi<double>())
                            state->as<StateType>()->values[i] = from->as<StateType>()->values[i] + diff * t;
                        else
                        {
                            double &v = state->as<StateType>()->values[i];
                            if (diff > 0.0)
                                diff = 2.0 * boost::math::constants::pi<double>() - diff;
                            else
                                diff = -2.0 * boost::math::constants::pi<double>() - diff;
                            v = from->as<StateType>()->values[i] - diff * t;
                            // input states are within bounds, so the following check is sufficient
                            if (v > boost::math::constants::pi<double>())
                                v -= 2.0 * boost::math::constants::pi<double>();
                            else if (v < -boost::math::constants::pi<double>())
                                v += 2.0 * boost::math::constants::pi<double>();
                        }
                    }//end if continuous
                    else{
                        state->as<StateType>()->values[i]=from->as<StateType>()->values[i]+diff*t;
                    }
                }//end for
                
            }

            void setup() override{
                bounds_.check();
                StateSpace::setup();//调用父类函数
            }
            void printState(const State *state,std::ostream &out) const override{
                out<<"TorusState [";
                if(state !=nullptr)//null 被解释为0，而nullptr不会被解释为0
                {
                    for(int i=0;i<dim_;i++){
                        out<<" "<<state->as<StateType>()->values[i];
                    }
                }
                else{
                    out<<"nullptr";
                }
                out<<"]"<<std::endl;
            }
            void printSettings(std::ostream &out)const override{
                out<<"Torus State Space in "<<dim_<<" dimension."<<endl;
            }
            StateSamplerPtr allocDefaultStateSampler() const{
                // std::shared_ptr<TorusStateSampler> sampler(new TorusStateSampler(this));
                // return sampler;
                return std::make_shared<TorusStateSampler>(this);
            }

            void registerProjections() override{
                class TorusDefaultProjection: public ProjectionEvaluator{
                public:
                    TorusDefaultProjection(const StateSpace *space): ProjectionEvaluator(space){}
                    unsigned int getDimension() const override
                    {
                        return space_->getDimension();
                    }

                    void defaultCellSizes() override
                    {   
                        unsigned int dim=getDimension();
                        cellSizes_.resize(dim);
                        bounds_.resize(dim);
                        for(int i=0;i<dim;i++)
                        {
                            cellSizes_[i]=boost::math::constants::pi<double>()/magic::PROJECTION_DIMENSION_SPLITS;
                            bounds_.low[i] = -boost::math::constants::pi<double>();
                            bounds_.high[i] = boost::math::constants::pi<double>();
                        }
                    }

                    void project(const State* state,Eigen::Ref<Eigen::VectorXd> projection) const override
                    {   
                        unsigned int dim=getDimension();
                        for(int i=0;i<dim;i++)
                        {
                            projection(i)=state->as<TorusStateSpace::StateType>()->values[i];
                        }   
                    }
                };
                registerDefaultProjection(std::make_shared<TorusDefaultProjection>(this));
            }   
         };
		

        // class AxisConstraint : public Constraint{
		// 	public:
		// 		Eigen::Affine3d end_effector_transform_;
        //         Eigen::VectorXd constraint_vector_;
        //         robot_state::RobotStatePtr kinematic_state_;
        //         moveit::planning_interface::MoveGroupInterface *move_group_;
        //         const robot_state::JointModelGroup *joint_model_group_;
                
        //         AxisConstraint(const Eigen::Affine3d end_effector_transform,
        //                         Eigen::VectorXd constraint_vector,
        //                         robot_state::RobotStatePtr kinematic_state,
        //                         const robot_state::JointModelGroup *joint_model_group)
        //         :Constraint(6,2,1e-3),
        //         constraint_vector_(constraint_vector),
        //         kinematic_state_(kinematic_state),
        //         joint_model_group_(joint_model_group)
        //         {
        //             end_effector_transform_=end_effector_transform.inverse();
        //         }

        //         void function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const override{
        //             std::vector<double> joint_values;
        //             for(int i=0;i<joint_model_group_->getVariableCount();i++)
        //             {
        //                 joint_values.push_back(x(i));
        //             }
        //             kinematic_state_->setJointGroupPositions(joint_model_group_,joint_values);
        //             const Eigen::Affine3d &end_transform=kinematic_state_->getGlobalLinkTransform(joint_model_group_->getLinkModelNames().back());
        //             const Eigen::Affine3d transform=end_effector_transform_*end_transform;
        //             Eigen::VectorXd delta_x;
        //             delta_x.head(3)=transform.translation().block<3,1>(0,0);
        //             delta_x.tail(3)=transform.rotation().eulerAngles(0,1,2);
                    
        //             for(int i=0;i<6;i++)
        //             {
        //                 if(constraint_vector_(i)!=0){
        //                     out(i)=delta_x(i);
        //                 }
        //                 else{
        //                     out(i)=0;
        //                 }
        //             }
        //         }

        //         void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::MatrixXd out) 
        //         {
        //             Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

        //             kinematic_state_->getJacobian(joint_model_group_,
        //                             kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
        //                             reference_point_position, out);
        //         }
        // };
    }//namespace base
}//namespace ompl

void ompl::base::TorusStateSampler::sampleUniform(State *state){
	const unsigned int dim=space_->getDimension();
	const RealVectorBounds &bounds=static_cast<const TorusStateSpace *>(space_)->getBounds();
	auto *rstate = static_cast<TorusStateSpace::StateType *>(state);
	for (unsigned int i = 0; i < dim; ++i)
		rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);

}
void ompl::base::TorusStateSampler::sampleUniformNear(State *state,const State *near,const double distance){
	const double *vnear=near->as<TorusStateSpace::StateType>()->values;
	double *vstate=state->as<TorusStateSpace::StateType>()->values;
	const unsigned int dim=space_->getDimension();
	for(int i=0;i<dim;i++){
		vstate[i]=rng_.uniformReal(vnear[i]-distance,vnear[i]+distance);
	}
	space_->enforceBounds(state);
}
void ompl::base::TorusStateSampler::sampleGaussian(State* state,const State* mean,const double stdDev){
	const double *vmean=mean->as<TorusStateSpace::StateType>()->values;
	double *vstate=state->as<TorusStateSpace::StateType>()->values;
	const unsigned int dim=space_->getDimension();
	for(int i=0;i<dim;i++){
		vstate[i]=rng_.gaussian(vmean[i],stdDev);
	}
	space_->enforceBounds(state);
}