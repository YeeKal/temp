#ifndef RRT
#define RRT

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>

#include "planning_util.h"
#include "scene_base.h"

#define NEAR_NODES 5




class Rrt {
private:
	Status goal;
	Status origin;
	std::vector<Status> path;
	SceneBase *scene;
	Tree tree;
	float step=10; //length in each step
	int dim;
public:
	Rrt(SceneBase *scene);

	Status randomNode() const;
	std::vector<int> nearestNode(Status node_rand,int num) const;
	Status steerNode(Status node_near,Status node_rand) const;
	bool CheckCollision(Status node_near, Status node_new) const;
	bool extend();
	void extractPath();

	static bool cmp(const NodePtr &a, const NodePtr &b);
	static float distanceCost(Status a, Status b) ;
	std::vector<Status> getPath(){return path;}

};

#endif
