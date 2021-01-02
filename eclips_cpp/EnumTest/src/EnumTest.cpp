//============================================================================
// Name        : EnumTest.cpp
// Author      : taikitanaka
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;


enum Eday{
	sunday,
	monday
};

void enumfunc(enum Eday day){
	if(day==sunday){
		cout<<"sunday";
	}
}



class TestC{
public:
	enum Etest{
		right,
		ledft
	};
};


enum enumfuncC(enum TestC::Etest test){
	cout<<test;

	cout<<test.right;

	return test;
}


int main() {

	int day=Eday::sunday;
	//enum newEnum=Eday::monday;
	Eday::sunday;



	enumfunc(Eday::sunday);

	enumfuncC(TestC::Etest::right);



	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
	return 0;
}
