#include"RrtGraph.h"

bool cmp(const rrt_node &a, const rrt_node &b);
vecPoint vectorPoint(Point a, Point b);
float vectorProduct(vecPoint a, vecPoint b);
bool isIntersected(Point a, Point b, Point c, Point d);
float distanceCost(Point a, Point b);

RrtGraph::RrtGraph() {
	srand((unsigned)time(NULL));
	//700*1200 gray map
	Mat map = Mat::ones(800, 1200, CV_8UC1) * 255;
	buildObstacle(map);
	Mat coords[3] = { map,map.clone(),map.clone() };
	merge(coords, 3, graph);//color map
	RrtGraph::graphDraw();
	this->goal = Point(map.cols - 5, map.rows - 5);
	this->origin = Point(300, 600);
	rows = map.rows;
	cols = map.cols;

	rrt_node m = new struct node();
	m->point = origin;
	m->parent_index = -1;
	m->self_index = 0;
	m->distance = 0;
	m->cost = 0;
	rrtTree.push_back(m);
}

/*
add obstacle in the map
map - single channel
*/
void RrtGraph::buildObstacle(Mat &map) {
	for (int i = 0; i < 3; i++) {
		obstacle.push_back(obstacleNodes[i]);
	}
	//obstacle num
	for (size_t sz = 0; sz < obstacle.size(); sz++) {
		//rows num
		for (int i = obstacle.at(sz)[0]; i < obstacle.at(sz)[1]; i++) {
			uchar *data = map.ptr<uchar>(i);
			//cols nnum
			for (int j = obstacle.at(sz)[2]; j < obstacle.at(sz)[3]; j++) {
				data[j] = 0;
			}
		}
	}
}

/*draw the graph*/
void RrtGraph::graphDraw() const {
	imshow("map", graph);
	waitKey(5);
}

/*return map*/
Mat RrtGraph::getMap() const {
	return graph;
}

/*draw the path in each step*/
void RrtGraph::drawLine(Point p1, Point p2) const {
	line(graph, p1, p2, Scalar(200, 0, 0), 1);
}

/*generate node_rand*/
Point RrtGraph::randomNode() const {
	//heuristic random
	//uncommon this, time will be reduced greatly
	// if (rand() % 2 == 0)
	// 	return goal;
	int x = rand() % cols;
	int y = rand() % rows;
	return Point(x, y);
}

/*find neighbor nodes*/
vector<int> RrtGraph::nearestNode(Point node_rand, int num) const {
	vector<rrt_node> tree_copy(rrtTree);
	size_t num_visited = rrtTree.size();
	vector<int> index_list;
	//if not enough nodes, all will be viewed as the neighbors
	if (num_visited <= num) {
		for (int i = 0; i < num_visited; i++) {
			index_list.push_back(i);
		}
		return index_list;
	}

	for (int i = 0; i < num_visited; i++) {
		tree_copy.at(i)->distance = distanceCost(node_rand, tree_copy.at(i)->point);
	}
	sort(tree_copy.begin(), tree_copy.end(), cmp);
	for (int i = 0; i < num; i++) {
		index_list.push_back(tree_copy.at(i)->self_index);
	}
	return index_list;
}

/*generate node_new in the direction of node_rand*/
Point RrtGraph::steerNode(Point node_near, Point node_rand) const {
	float theta = atan2((float)(node_rand.y - node_near.y), (float)(node_rand.x - node_near.x));
	int x = int(node_near.x + step*cos(theta));
	int y = int(node_near.y + step*sin(theta));
	return Point(x, y);
}
// if there is an obstacle free path
//check two lines intersect or not
//reference to https://segmentfault.com/a/1190000004457595
bool RrtGraph::obstacleFree(Point node_near, Point node_new) const {
	for (size_t sz = 0; sz < obstacle.size(); sz++) {
		//rows num
		Point x1, x2, x3, x4;
		int a, b, c, d;
		a = obstacle.at(sz)[0];
		b = obstacle.at(sz)[1];
		c = obstacle.at(sz)[2];
		d = obstacle.at(sz)[3];
		x1 = Point(c, a);
		x2 = Point(d, a);
		x3 = Point(d, b);
		x4 = Point(c, b);

		if (isIntersected(node_near, node_new, x1, x2) || isIntersected(node_near, node_new, x2, x3))
			return false;
		else if (isIntersected(node_near, node_new, x3, x4) || isIntersected(node_near, node_new, x4, x1))
			return false;
	}
	return true;
}
//expand rrt tree
void RrtGraph::extend() {
	bool success = false;
    circle(graph,origin,4,Scalar(0,255,0),-1);
    circle(graph,goal,4,Scalar(100,100,255),-1);
    RrtGraph::graphDraw();
	while (!success) {
		Point node_rand, node_near, node_new;
		vector<int> near_index_list;
		int near_index;
		node_rand = RrtGraph::randomNode();
		near_index_list = RrtGraph::nearestNode(node_rand, 1);
		near_index = near_index_list.at(0);
		node_near = rrtTree.at(near_index)->point;
		node_new = RrtGraph::steerNode(node_near, node_rand);
		if (obstacleFree(node_near, node_new)) {
			//update near node index
			near_index_list = RrtGraph::nearestNode(node_new, NEAR_NODES);
			rrt_node m = new struct node();
			size_t num_node = rrtTree.size();
			m->parent_index = near_index;
			m->point = node_new;
			m->self_index = num_node;
			m->cost = distanceCost(node_new, node_near) + rrtTree.at(near_index)->cost;
			m->distance = 0;
			rrtTree.push_back(m);

			int min_index = near_index;
			float newCost;
			float min_cost = m->cost;
			//find the min_cost near node connected to node_new
			for (size_t i = 0; i < near_index_list.size(); i++) {
				if (obstacleFree(rrtTree.at(near_index_list.at(i))->point, node_new)) {
					newCost = distanceCost(rrtTree.at(near_index_list.at(i))->point, node_new) +
						rrtTree.at(near_index_list.at(i))->cost;
					if (newCost < min_cost) {
						min_index = near_index_list.at(i);
						min_cost = newCost;
					}
				}
			}
			//update the connect to node_new
			rrtTree.at(num_node)->parent_index = min_index;
			rrtTree.at(num_node)->cost = min_cost;
			//update node_new's child node
			for (size_t i = 0; i < near_index_list.size(); i++) {
				if (obstacleFree(rrtTree.at(near_index_list.at(i))->point, node_new)) {
					if (rrtTree.at(near_index_list.at(i))->cost>(rrtTree.at(num_node)->cost + distanceCost(rrtTree.at(near_index_list.at(i))->point, node_new))) {
						//delete line
						Point p1 = rrtTree.at(near_index_list.at(i))->point;
						Point p2 = rrtTree.at(rrtTree.at(near_index_list.at(i))->parent_index)->point;
						line(graph, p1, p2, Scalar(255, 255, 255), 1);
						//change line
						rrtTree.at(near_index_list.at(i))->parent_index = num_node;
						RrtGraph::drawLine(rrtTree.at(near_index_list.at(i))->point, node_new);
					}
				}
			}
			RrtGraph::drawLine(rrtTree.at(min_index)->point, node_new);
			RrtGraph::graphDraw();
		}

		//check whether it is free from node_new to goal
		if (distanceCost(node_new, goal) <= (step + 20) && obstacleFree(node_new, goal)) {
			rrt_node node_last = new struct node();
			size_t num_node = rrtTree.size();
			node_last->parent_index = num_node - 1;
			node_last->point = goal;
			node_last->self_index = num_node;
			node_last->cost = distanceCost(node_new, goal) + rrtTree.at(num_node - 1)->cost;
			node_last->distance = 0;
            rrtTree.push_back(node_last);
			RrtGraph::drawLine(goal, node_new);
			RrtGraph::graphDraw();
			success = true;
		}
	}
	//draw the path after the planning succeed
	RrtGraph::drawPath();
}

/*draw the path*/
void RrtGraph::drawPath() {
	vector<rrt_node>::iterator ptr = rrtTree.end() - 1;
	//Point current = (*(--ptr))->point;
	while ((*ptr)->parent_index >= 0) {
		line(graph, (*ptr)->point, rrtTree.at((*ptr)->parent_index)->point, Scalar(0, 0, 250), 2);
		RrtGraph::graphDraw();
		ptr = rrtTree.begin() + (*ptr)->parent_index;
	}
}



/*sort vector<rrt_node> by distance*/
bool cmp(const rrt_node &a, const rrt_node &b) {
	return a->distance < b->distance;
}

/*define a vector by two points*/
vecPoint vectorPoint(Point a, Point b) {
	vecPoint vec = new struct vectorTwoPoint();
	vec->x = b.x - a.x;
	vec->y = b.y - a.y;
	return vec;
}

/*cross product of two 2-d vector*/
float vectorProduct(vecPoint a, vecPoint b) {
	return a->x*b->y - a->y*b->x;
}

//return true if two lines intersected
bool isIntersected(Point a, Point b, Point c, Point d) {
	vecPoint ac, ad, bc, bd;
	vecPoint ca, cb, da, db;
	ac = vectorPoint(a, c);
	ad = vectorPoint(a, d);
	bc = vectorPoint(b, c);
	bd = vectorPoint(b, d);

	ca = vectorPoint(c, a);
	da = vectorPoint(d, a);
	cb = vectorPoint(c, b);
	db = vectorPoint(d, b);
	return (vectorProduct(ac, ad)*(vectorProduct(bc, bd)) <= 0 && vectorProduct(ca, cb)*(vectorProduct(da, db)) <= 0);
}

/*calculate distance between two points*/
float distanceCost(Point a, Point b) {
	return sqrt((float)pow(a.x - b.x, 2) + (float)pow(a.y - b.y, 2));
}