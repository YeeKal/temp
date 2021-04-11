/*************
support: opencv 3.2.0-vs2015 / opencv 3.4.1-ubuntu
author: yee
update: 2018-4-21
**************/
#include"RrtGraph.h"

int main() {
	
	RrtGraph *Graph=new RrtGraph();
	//expand the tree
	Graph->extend();

	//enter <q> to quit
	while(char(waitKey(5))!='q'){}
	
	return 0;
}