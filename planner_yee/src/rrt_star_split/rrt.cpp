#include <algorithm>
#include"rrt.h"


Rrt::Rrt(SceneBase *scene){
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
	tree.push_back(m);
}

Status Rrt::randomNode() const{
	return scene->randomNode();
}

/*find neighbor nodes*/
std::vector<int> Rrt::nearestNode(Status node_rand, int num) const {
	Tree tree_copy(tree);
	size_t num_visited = tree.size();
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
Status Rrt::steerNode(Status node_near, Status node_rand) const {
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

//expand rrt tree
bool Rrt::extend() {
	bool success = false;

	while (!success) {
		Status node_rand, node_near, node_new;
		std::vector<int> near_index_list;
		int near_index;
		node_rand = Rrt::randomNode();
		near_index_list = Rrt::nearestNode(node_rand, 1);
		near_index = near_index_list.at(0);
		node_near = tree.at(near_index)->status;
		node_new = Rrt::steerNode(node_near, node_rand);
		if (scene->obstacleFree(node_near, node_new)) {
			//update near node index
			near_index_list = Rrt::nearestNode(node_new, NEAR_NODES);
			NodePtr m = new struct Node();
			size_t num_node = tree.size();
			m->parent_index = near_index;
			m->status = node_new;
			m->self_index = num_node;
			m->cost = distanceCost(node_new, node_near) + tree.at(near_index)->cost;
			m->distance = 0;
			tree.push_back(m);

			int min_index = near_index;
			float newCost;
			float min_cost = m->cost;
			//find the min_cost near node connected to node_new
			for (size_t i = 0; i < near_index_list.size(); i++) {
				if (scene->obstacleFree(tree.at(near_index_list.at(i))->status, node_new)) {
					newCost = distanceCost(tree.at(near_index_list.at(i))->status, node_new) +
						tree.at(near_index_list.at(i))->cost;
					if (newCost < min_cost) {
						min_index = near_index_list.at(i);
						min_cost = newCost;
					}
				}
			}
			//update the connect to node_new
			tree.at(num_node)->parent_index = min_index;
			tree.at(num_node)->cost = min_cost;
			//update node_new's child node
			for (size_t i = 0; i < near_index_list.size(); i++) {
				if (scene->obstacleFree(tree.at(near_index_list.at(i))->status, node_new)) {
					if (tree.at(near_index_list.at(i))->cost>(tree.at(num_node)->cost + distanceCost(tree.at(near_index_list.at(i))->status, node_new))) {
						//delete line
						Status p1 = tree.at(near_index_list.at(i))->status;
						Status p2 = tree.at(tree.at(near_index_list.at(i))->parent_index)->status;
						scene->delLine(p1,p2);
						//change line
						tree.at(near_index_list.at(i))->parent_index = num_node;
						Rrt::scene->drawLine(tree.at(near_index_list.at(i))->status, node_new);
					}
				}
			}
			Rrt::scene->drawLine(tree.at(min_index)->status, node_new);
		}

		//check whether it is free from node_new to goal
		if (distanceCost(node_new, goal) <= (step + 20) && scene->obstacleFree(node_new, goal)) {
			NodePtr node_last = new struct Node();
			size_t num_node = tree.size();
			node_last->parent_index = num_node - 1;
			node_last->status = goal;
			node_last->self_index = num_node;
			node_last->cost = distanceCost(node_new, goal) + tree.at(num_node - 1)->cost;
			node_last->distance = 0;
            tree.push_back(node_last);
			scene->drawLine(goal, node_new);
			success = true;
		}
	}

	if(success){
		extractPath();
	}

	return success;
}

void Rrt::extractPath(){
	Tree::iterator ptr = tree.end() - 1;
	//Point current = (*(--ptr))->point;
	while ((*ptr)->parent_index >= 0) {
		path.push_back((*ptr)->status);
		ptr = tree.begin() + (*ptr)->parent_index;
	}
	path.push_back((*ptr)->status);
	
	std::reverse(path.begin(), path.end());
}

/*sort vector<rrt_node> by distance*/
bool Rrt::cmp(const NodePtr &a, const NodePtr &b) {
	return a->distance < b->distance;
}

/*calculate distance between two points*/
float Rrt::distanceCost(Status a, Status b)  {
    float cost_square=0.0;
    for(int i=0;i<a.size();i++){
        cost_square += (float)pow(a[i] - b[i], 2);
    }
	return sqrt(cost_square);
}



