#ifndef PLANNING_UTIL
#define PLANNING_UTIL

#include <iostream>
#include <cstdlib>
#include <vector>

#define NEAR_NODES 5

typedef struct Node* NodePtr;
typedef std::vector<float> Status;
typedef std::vector<NodePtr> Tree;


struct  Node{
	int parent_index;
	int self_index;
	float cost;
	float distance;
	Status status;
};

static int obstacleNodes[3][4] = { 0,300,200,250 , 350, 400, 900, 1200 , 400, 700, 600, 700 };

#endif
