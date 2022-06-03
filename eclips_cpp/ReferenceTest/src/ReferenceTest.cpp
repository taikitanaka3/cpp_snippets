//============================================================================
// Name        : ReferenceTest.cpp
// Author      : taikitanaka
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;

int main() {

	int a=1;
	int &b=a;

	a++;
	cout<<"a=2 "<<a<<"bはaの参照だから2 "<<b<<endl;

	b++;
	cout<<"aはbの参照元なだけだから2 "<<a<<"bは3 "<<b<<endl;


	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
