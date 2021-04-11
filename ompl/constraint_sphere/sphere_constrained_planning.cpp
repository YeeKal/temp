#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
 
#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <cmath>
 
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

//constraints must inherit from the constraint base class
class Sphere: public ompl::base::Constraint{
public:
    Sphere():ompl::base::Constraint(3,1){}
    //define the constraint function
    void function(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::VectorXd> out) const override{
        out[0]=x.norm()-1;
    }
    //provide an analytic Jacobian for a constrained planning problem
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,Eigen::Ref<Eigen::MatrixXd> out) const override{
        out=x.transpose().normalized();
    }
};
//define obstacle
bool isValid(const ompl::base::State *state){
    //define a narrow band obstacle with a small hole on the side
    const Eigen::Map<Eigen::VectorXd> &x=*state->as<ob::ConstrainedStateSpace::StateType>();
    if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    return true;
}

int main(){
    //R^3 space
    auto space=std::make_shared<ompl::base::RealVectorStateSpace>(3);
    ompl::base::RealVectorBounds bounds(3);
    bounds.setLow(-2);
    bounds.setHigh(2);
    space->setBounds(bounds);

    auto constraint=std::make_shared<Sphere>();

    auto constraint_space=std::make_shared<ompl::base::ProjectedStateSpace>(space,constraint);

    cout<<"constraint space name:"<<constraint_space->getName().c_str()<<endl;

    auto constraint_si=std::make_shared<ompl::base::ConstrainedSpaceInformation>(constraint_space);

    auto ss=std::make_shared<ompl::geometric::SimpleSetup>(constraint_si);
    ss->setStateValidityChecker(isValid);

    Eigen::VectorXd sv(3),gv(3);
    sv<<0,0,1;
    gv<<0,0,-1;

    ompl::base::ScopedState<> start(constraint_space);
    ompl::base::ScopedState<> goal(constraint_space);

    start->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(sv);
    goal->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(gv);

    ss->setStartAndGoalStates(start,goal);

    auto pp=std::make_shared<ompl::geometric::PRM>(constraint_si);
    ss->setPlanner(pp);
    ss->setup();

    ompl::base::PlannerStatus solution=ss->solve(5.0);
    if(solution){
        //ss->simplifySolution(5.);//optimize the path
        ompl::geometric::PathGeometric path=ss->getSolutionPath();
        path.interpolate();
        //path.print(std::cout);

        std::ofstream pathFile("c_path.txt");

        path.as<ompl::geometric::PathGeometric>()->printAsMatrix(pathFile);
        pathFile.close();

    }
    else{
        std::cout<<"no solution"<<std::endl;
    }

//test
    ompl::base::StateSamplerPtr sampler;
    sampler=constraint_space->allocStateSampler();
    ompl::base::State *state=constraint_space->allocState();

    //adress comparation
    ompl::base::StateSpacePtr space_s=constraint_space;
    std::cout<<"adress as projected:"<<space_s->as<ompl::base::ProjectedStateSpace>()->getValueAddressAtIndex(state,0)<<std::endl;
    std::cout<<"adress as state space:"<<space_s->getValueAddressAtIndex(state,0)<<std::endl;
    std::cout<<"adress as constrained:"<<space_s->as<ompl::base::ConstrainedStateSpace>()->getValueAddressAtIndex(state,0)<<std::endl;

    
    std::cout<<"space:"<<space_s->getName().c_str()<<std::endl;
    std::cout<<"adress as state space:"<<space_s->getValueAddressAtIndex(state,0)<<std::endl;



    return 0;
}