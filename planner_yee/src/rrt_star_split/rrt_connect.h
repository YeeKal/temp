

#ifndef RRT_CONNECT
#define RRT_CONNECT

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

#include "planning_util.h"
#include "scene_base.h"

#define NEAR_NODES 5


// status returned when one tree extendes
enum ExtendStatus{
    Trapped=0,      // not step forward
    Connected=1,    // connected
    Extended=2,     // step forward
};


class RrtConnect {
private:
	Status goal;
	Status origin;
	std::vector<Status> path;
	SceneBase *scene;
	Tree start_tree;
    Tree goal_tree;
	float step=10; //length in each step
	int dim;
public:
	RrtConnect(SceneBase *scene);

	Status randomNode() const;
    static bool cmp(const NodePtr &a, const NodePtr &b);
	static float distanceCost(Status a, Status b) ;
	std::vector<Status> getPath(){return path;}

	std::vector<int> nearestNode(Tree any_tree, Status node_rand,int num) const;
	Status steerNode(Status node_near,Status node_rand) const;
    ExtendStatus extendTree(Tree *anyTree, Status node_rand);
	bool extend();
	void extractPath();


};

#endif
