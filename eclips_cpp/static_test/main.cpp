/*
 * main.cpp
 *
 *  Created on: Dec 5, 2019
 *      Author: tanakasan
 */

#include "main.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
using std::cout;
using std::endl;
using std::vector;


float occg::x=(float)2.0;
void test(){

	printf("%f",occg::x);
}
extern void test2();

//Point Pose::point={};
//vector<vector3f> PoseArray::pose={};

Point Pose::point={};
float Pose::angle={};
vector<Pose> PoseArray::pose={};




int main(){
	Point state0 = {0.f,0.f, 2.f};
	Point state = {2.f,2.f, 2.f};
	Pose::point = state;
	Pose::angle=0.f;
	Pose p={};

	p.point=state;
	p.angle=2.f;
	PoseArray::pose.emplace_back(p);
	PoseArray::pose.emplace_back(p);


	vector<int> j={1,2};
	vector<vector<int>> i={};
	i.emplace_back(j);


	printf(" %d \n %d \n",i[0][0],i[0][1]);

	//PoseArray pose;

	//printf(" %f \n %f \n",PoseArray::pose);

	//vector3f p=;
	//Pose::vector3f=p;
	printf(" %f \n %f \n",Pose::angle,p.angle);

	//std::cout<Pose::vector3f.X<<std:endl;
	//constc cons={};
	occg::x=3.f;
	printf("%f",occg::x);
	occg::x=5.f;

	test();
	test2();
	return 0;
}



