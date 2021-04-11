#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <cstdlib>

#define NEAR_NODES 5

using namespace cv;
using namespace std;
typedef struct node* rrt_node;
typedef struct vectorTwoPoint* vecPoint;



//rows-cols
static int obstacleNodes[3][4] = { 0,300,200,250 , 350, 400, 900, 1200 , 400, 700, 600, 700 };

struct  node{
	Point point;
	int parent_index;
	int self_index;
	float cost;
	float distance;
};
struct vectorTwoPoint {
	float x;
	float y;
};


class RrtGraph {
private:
	Mat graph; //m*n*3
	Point goal;
	Point origin;
	vector<rrt_node> rrtTree;
	int rows, cols;
	float step=10; //length in each step
	vector<int *> obstacle;//record obstacle
public:
	RrtGraph();
	void buildObstacle(Mat &map);
	void graphDraw() const;

	Mat getMap() const;
	void drawLine(Point p1, Point p2) const;
	Point randomNode() const;
	vector<int> nearestNode(Point node_rand,int num) const;
	Point steerNode(Point node_near,Point node_rand) const;
	bool obstacleFree(Point node_near, Point node_new) const;
	void extend();
	void drawPath();
};
