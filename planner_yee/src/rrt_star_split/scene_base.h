/**
 * scene:
 * 0. provide goal and origin
 * 1. check collision
 * 2. draw Line
 * 3. draw path
 * 4. random node
 **/
#ifndef SCENE_BASE
#define SCENE_BASE

#include "planning_util.h"


//rows-cols

class SceneBase{
protected:
	Status goal;
	Status origin;
public:
    SceneBase(){}
    virtual ~SceneBase(){}
	SceneBase(Status p1, Status p2){
        origin=p2;
        goal=p1;
    }

	virtual void drawLine(Status p1, Status p2) const=0;
	virtual void delLine(Status p1, Status p2) const=0;
    virtual Status randomNode() const=0;
	virtual bool obstacleFree(Status node_near, Status node_new) const=0;


    Status getGoal(){return goal;}
    Status getOrigin(){return origin;}

};

#endif

