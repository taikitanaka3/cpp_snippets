//============================================================================
// Name        : astar.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <vector>
using std::vector;
using namespace std;

struct Node{
	int x;
	int y;
	int cost;
	int pind;

	Node(int x,int y,int cost,int pind){
		this->x=x;
		this->y=y;
		this->cost=cost;
		this->pind=pind;
	}
	void print(){
		printf("x:%d,y:%d,cost:%d,pind:%d\n",x,y,cost,pind);
	}
};
struct xy{
	float x;
	float y;
	xy(float x,float y){
		this->x=x;
		this->y=y;
	}
};

class DikstraSearch{

public:
	void Initialize(){
		_sx=10;
		_sy=10;
		_gx=50;
		_gy=50;
		_grid_size=2;
		_robot_radius=1;
	}
private:
	float reso;
	vector<float> oxy;
	int open_node;
	int closed_nodes;
	float _sx;
	float _sy;
	float _gx;
	float _gy;
	float _grid_size;
	float _robot_radius;
	void make_map(){

	}
	void calc_obstacle_map(xy oxy){
		//sint minx=roud
	}
	/*input
	 * sx:start position of x
	 * sy:start position of y
	 * gx goal position of x
	 * gy goal position of y
	 *output
	 *rx:position listx of final path
	 *ry:position listy of  final path
	 */
	void planing(float sx,float sy,float gx,float gy){

	}
	void calc_fial_map(){
	}
	void calc_heuristic(){
	}

	void calc_grid_position(){
	}
	void calc_xy_index(){
	}
	void calc_grid_index(){
	}
	void verify_node(Node node){
	}
	void get_motion_model(){

	}



};

int main() {
	Node node;
	node.print();
	uint a=3;
	cout<<a;
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
