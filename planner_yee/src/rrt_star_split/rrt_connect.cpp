#include <algorithm>
#include "rrt_connect.h"


RrtConnect::RrtConnect(SceneBase *scene){
	this->scene=scene;
	goal=scene->getGoal();
	origin=scene->getOrigin();

	dim=goal.size();

	NodePtr m = new struct Node();
	m->status = origin;
	m->parent_index = -1;
	m->self_index = 0;
	m->distance = 0;
	m->cost = 0;
	start_tree.push_back(m);

    NodePtr mg = new struct Node();
	mg->status = goal;
	mg->parent_index = -1;
	mg->self_index = 0;
	mg->distance = 0;
	mg->cost = 0;
	goal_tree.push_back(mg);
}

/*find neighbor nodes*/
std::vector<int> RrtConnect::nearestNode(Tree any_tree, Status node_rand, int num) const {
	Tree tree_copy(any_tree);
	size_t num_visited = tree_copy.size();
	std::vector<int> index_list;
	//if not enough nodes, all will be viewed as the neighbors
	if (num_visited <= num) {
		for (int i = 0; i < num_visited; i++) {
			index_list.push_back(i);
		}
		return index_list;
	}

	for (int i = 0; i < num_visited; i++) {
		tree_copy.at(i)->distance = distanceCost(node_rand, tree_copy.at(i)->status);
	}
	sort(tree_copy.begin(), tree_copy.end(), cmp);
	for (int i = 0; i < num; i++) {
		index_list.push_back(tree_copy.at(i)->self_index);
	}
	return index_list;
}

/*generate node_new in the direction of node_rand*/
Status RrtConnect::steerNode(Status node_near, Status node_rand) const {
	Status s;
	float dis,coeff=10;
	dis=distanceCost(node_near,node_rand);
	if(dis > 0.1){
		coeff =step / dis;
	}

	for(int i=0;i<dim;i++){
		s.push_back(coeff*(node_rand[i]-node_near[i])+node_near[i]);
	}

	return s;
}

ExtendStatus RrtConnect::extendTree(Tree *anyTree, Status node_rand){
    Status node_near, node_new;
    std::vector<int> nearest_index_list=nearestNode(*anyTree, node_rand,1);
    int near_index = nearest_index_list.at(0);

    node_near=anyTree->at(near_index)->status;
    float dis_near=distanceCost(node_near, node_rand);
    if(dis_near < step){
        if(! scene->obstacleFree(node_near,node_rand)){
            return Trapped;
        }else{
            NodePtr m = new struct Node();
			size_t num_node = anyTree->size();
			m->parent_index = near_index;
			m->status = node_rand;
			m->self_index = num_node;
			m->cost = dis_near + anyTree->at(near_index)->cost;
			m->distance = 0;
			anyTree->push_back(m);
		    RrtConnect::scene->drawLine(node_near, node_rand);
            return Connected;
        }
    }

    node_new = RrtConnect::steerNode(node_near, node_rand);
    if (scene->obstacleFree(node_near, node_new)) {
        NodePtr m = new struct Node();
        size_t num_node = anyTree->size();
        m->parent_index = near_index;
        m->status = node_new;
        m->self_index = num_node;
        m->cost = distanceCost(node_near, node_new) + anyTree->at(near_index)->cost;
        m->distance = 0;
        anyTree->push_back(m);
		RrtConnect::scene->drawLine(node_near, node_new);
        return Extended;
    }

    return Trapped;
}

//expand RrtConnect tree
bool RrtConnect::extend() {
	bool success = false;
    bool is_start=false;
    Tree *t1,*t2;
    ExtendStatus extend_status;

    int max_iter=1000;
	while (!success && max_iter>0) {
        is_start= !is_start;
        max_iter--;

        t1=is_start? &start_tree:&goal_tree;
        t2=is_start? &goal_tree:&start_tree;

		Status node_rand, node_added;

		node_rand = RrtConnect::randomNode();
        extend_status=extendTree(t1,node_rand);
        if(extend_status == Trapped){
            continue;
        }

        node_added=t1->back()->status;

        extend_status=extendTree(t2,node_added);
        while(extend_status==Extended){
            extend_status=extendTree(t2,node_added);
        }

        if(extend_status == Connected){
            success=true;
        }
    }

	if(success){
		extractPath();
	}

	return success;
}

void RrtConnect::extractPath(){
    // add start tree path
	Tree::iterator ptr = start_tree.end() - 1;
	while ((*ptr)->parent_index >= 0) {
		path.push_back((*ptr)->status);
		ptr = start_tree.begin() + (*ptr)->parent_index;
	}
	path.push_back((*ptr)->status);
	std::reverse(path.begin(), path.end());

    // add goal tree path
    ptr = goal_tree.begin();
    while((*ptr)->parent_index >=0){
        path.push_back((*ptr)->status);
		ptr = goal_tree.begin() + (*ptr)->parent_index;
    }
}


Status RrtConnect::randomNode() const{
	return scene->randomNode();
}

/*sort vector<RrtConnect_node> by distance*/
bool RrtConnect::cmp(const NodePtr &a, const NodePtr &b) {
	return a->distance < b->distance;
}

/*calculate distance between two points*/
float RrtConnect::distanceCost(Status a, Status b)  {
    float cost_square=0.0;
    for(int i=0;i<a.size();i++){
        cost_square += (float)pow(a[i] - b[i], 2);
    }
	return sqrt(cost_square);
}






