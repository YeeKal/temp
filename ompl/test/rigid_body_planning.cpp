#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>


#include <ompl/config.h>
#include <iostream>
#include <fstream>
#include <cmath>
 
//namespace ob = ompl::base;
namespace og = ompl::geometric;

//[0.1]x[0,1]
//the obstacle is a dick with radious 0.25 int he center
class ValidityChecker: public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si): ompl::base::StateValidityChecker(si){}
    bool isValid(const ompl::base::State* state) const{
        const ompl::base::RealVectorStateSpace::StateType * state2D=state->as<ompl::base::RealVectorStateSpace::StateType>();
        double x=state2D->values[0];
        double y=state2D->values[1];
        double error;
        error=sqrt(pow(x-0.5,2)+pow(y-0.5,2))-0.25;
        if (error>0)
            return true;
        else
            return false;
    }
};

int main(){
    ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2));
    space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0,1.0);
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space),goal(space);
    start->values[0]=0;
    start->values[1]=0;
    goal->values[0]=1;
    goal->values[1]=1;

    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start,goal);
    
    ompl::base::PlannerPtr planner(new ompl::geometric::RRT(si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    ompl::base::PlannerStatus solved=planner->solve(1.0);

    if (solved)
     {
        std::cout << "Found solution:" << std::endl;
        ompl::base::PathPtr path=pdef->getSolutionPath();
        path->print(std::cout);

        std::ofstream pathFile("path.txt");

        path->as<ompl::geometric::PathGeometric>()->printAsMatrix(pathFile);
        pathFile.close();
     }
     else
         std::cout << "No solution found" << std::endl;


    return 0;
}
/*
两种构造方式
- ScopedState state
- State *state
赋值方式:
state->setX();
state->as<>()-.setX(); 
 */