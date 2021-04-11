#include "scene.h"


Scene::Scene(Status p1, Status p2,int w, int h){
    cv::Mat map = cv::Mat::ones(h, w, CV_8UC1) * 255;
	buildObstacle(map);
	cv::Mat coords[3] = { map,map.clone(),map.clone() };
	cv::merge(coords, 3, graph);//color map

	this->goal = p2;
	this->origin = p1;
	rows = map.rows;
	cols = map.cols;

    initGraph();
}

/*
add obstacle in the map
map - single channel
*/
void Scene::buildObstacle(cv::Mat &map) {
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

Status Scene::randomNode() const {
	//heuristic random
	//uncommon this, time will be reduced greatly
	if (rand() % 3 == 0)
		return goal;
    Status s;
	s.push_back((float)(rand() % cols));
	s.push_back((float)(rand() % rows));
	return s;
}

void Scene::initGraph(){
    cv::circle(graph,toCVPoint(origin),4,cv::Scalar(0,255,0),-1);
    cv::circle(graph,toCVPoint(goal),4,cv::Scalar(100,100,255),-1);
}

/*draw the graph*/
void Scene::graphDraw() const {
	imshow("map", graph);
	cv::waitKey(5);
}

/*return map*/
cv::Mat Scene::getMap() const {
	return graph;
}

/*draw the path in each step*/
void Scene::drawLine(Status s1, Status s2) const {
    cv::Point2i p1=toCVPoint(s1);
    cv::Point2i p2=toCVPoint(s2);
	cv::line(graph, p1, p2, cv::Scalar(200, 0, 0), 1);
    graphDraw();
}

void Scene::delLine(Status s1, Status s2) const{
    cv::Point2i p1=toCVPoint(s1);
    cv::Point2i p2=toCVPoint(s2);
	cv::line(graph, p1, p2, cv::Scalar(255, 255, 255), 1);
    graphDraw();
}



// if there is an obstacle free path
//check two lines intersect or not
//reference to https://segmentfault.com/a/1190000004457595
bool Scene::obstacleFree(Status node_near, Status node_new) const {
    cv::Point2i p_near(node_near[0], node_near[1]);
    cv::Point2i p_new(node_new[0], node_new[1]);

	for (size_t sz = 0; sz < obstacle.size(); sz++) {
		//rows num
		cv::Point2i x1, x2, x3, x4;
		int a, b, c, d;
		a = obstacle.at(sz)[0];
		b = obstacle.at(sz)[1];
		c = obstacle.at(sz)[2];
		d = obstacle.at(sz)[3];
		x1 = cv::Point2i(c, a);
		x2 = cv::Point2i(d, a);
		x3 = cv::Point2i(d, b);
		x4 = cv::Point2i(c, b);

		if (isIntersected(p_near, p_new, x1, x2) || isIntersected(p_near, p_new, x2, x3))
			return false;
		else if (isIntersected(p_near, p_new, x3, x4) || isIntersected(p_near, p_new, x4, x1))
			return false;
	}
	return true;
}

/*draw the path*/
void Scene::drawPath(std::vector<Status> path){
	for(std::vector<Status>::iterator iter=path.begin();iter!=path.end()-1;iter++){
		cv::line(graph, toCVPoint(*iter), toCVPoint(*(iter+1)), cv::Scalar(0, 0, 250), 2);
		Scene::graphDraw();
	}
}


/*define a vector by two points*/
cv::Point2i Scene::vectorPoint(cv::Point2i a, cv::Point2i b) {
	cv::Point2i vec;
	vec.x = b.x - a.x;
	vec.y = b.y - a.y;
	return vec;
}

/*cross product of two 2-d vector*/
float Scene::vectorProduct(cv::Point2i a, cv::Point2i b) {
	return a.x*b.y - a.y*b.x;
}

//return true if two lines intersected
bool Scene::isIntersected(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d) {
	cv::Point2i ac, ad, bc, bd;
	cv::Point2i ca, cb, da, db;
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
