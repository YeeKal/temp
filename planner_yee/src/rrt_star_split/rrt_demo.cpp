/*************
support: opencv 3.2.0-vs2015 / opencv 3.4.1-ubuntu
author: yee
update: 2018-4-21
**************/
#include <iostream>
#include <ctime>
#include <opencv2/core.hpp>
#include "scene.h"
#include "rrt.h"




int main() {
	srand((unsigned)time(NULL));

	int w, h;
	w=1200;
	h=800;

	Status goal = {(float)(w - 5), (float)(h - 5)};
	Status origin = {300, 600};

	Scene scene=Scene(origin,goal,w,h);
	Rrt rrt=Rrt(&scene);
	if(rrt.extend()){
		scene.drawPath(rrt.getPath());
	}else{
		std::cout<<"planning failed\n";
	}

	//enter <q> to quit
	while(char(cv::waitKey(5))!='q'){}
	
	return 0;
}