#include <iostream>
using namespace std;
struct vector3f{
	float X;
	float Y;
	float Z;
	vector3f(float x,float y,float z){
		X=x;
		Y=y;
		Z=z;
	}
};

int main(int argc, char **argv) {
	cout << "Hello world";
	vector3f a={1,2,3};
	return 0;
}
