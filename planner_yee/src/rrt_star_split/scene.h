/**
 * scene:
 * 0. provide goal and origin
 * 1. check collision
 * 2. draw Line
 * 3. draw path
 * 4. random node
 **/
#ifndef SCENE
#define SCENE

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "scene_base.h"


//rows-cols

class Scene : public SceneBase{
protected:
	cv::Mat graph; //m*n*3
	// Status goal;
	// Status origin;
	int rows, cols;
	std::vector<int *> obstacle;//record obstacle
public:
	Scene(Status p1, Status p2,int w, int h);
	void buildObstacle(cv::Mat &map);

	void graphDraw() const;
	cv::Mat getMap() const;

    void initGraph();
	void drawLine(Status p1, Status p2) const;
	void delLine(Status p1, Status p2) const;
    void drawPath(std::vector<Status> path);
	bool obstacleFree(Status node_near, Status node_new) const;
    Status randomNode() const;


    // Status getGoal(){return goal;}
    // Status getOrigin(){return origin;}

    static cv::Point2i vectorPoint(cv::Point2i a, cv::Point2i b);
    static float vectorProduct(cv::Point2i a, cv::Point2i b);
    static bool isIntersected(cv::Point2i a, cv::Point2i b, cv::Point2i c, cv::Point2i d);

    static cv::Point2i toCVPoint(Status s){
        if(s.size()<2){
            return cv::Point2i(0, 0);
        }
        return cv::Point2i(s[0], s[1]);
    }
};

#endif

