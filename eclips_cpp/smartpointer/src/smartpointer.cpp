//============================================================================
// Name        : smartpointer.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include<memory>
#include <vector>

using namespace std;



class hoge{
private:
   std::unique_ptr<int[]> ptr;
public:
   std::unique_ptr<int[]> ptr2;
   hoge(int val_):ptr(new int(val_)){};
//   int getValue()const{return *ptr;}
};

//std::unique_ptr<int> *pInt2;


int *pInt;

#define v 1000000

class Smart{
public:
	int k[v];
	std::shared_ptr<int[]> l;
};


int main(){

	Smart sm;
	//int v=10000;
	sm.k[v-1]=1;
	sm.l.reset(new int [v*100]);
	sm.l[v*100-1]=2;
	//vector<int> n(sm.l, std::end(sm.l));
	vector <int> *n=sm.l;
	cout<<"stack"<<sm.k[v-1]<<" heap"<<sm.l[v*100-1]<<n[v*100-1];

	sm.l[2]=3;

	//int *smartI;

	//std::unique_ptr<int> ptr2;
	//ptr2.reset(new int(10));

   //hogeのコンストラクタでint型を動的に確保しunique_ptrに委ねる
  //hoge Hoge(10);


   //std::unique_ptr<int[]> pInt(new int(120));
   //pInt[2]=100;
   //cout<<pInt[2];

   //cout<<Hoge.ptr2[3];



   return 0;
}//auto_ptr同様、ディストラクタで自動的にメモリ解放
