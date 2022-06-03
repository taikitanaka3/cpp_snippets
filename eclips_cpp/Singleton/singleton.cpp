//============================================================================
// Name        : ClassTemplete.cpp
// Author      : taikitanaka
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "singleton.h"
#include <iostream>
#include <string>

using namespace std;

//----------------




//--  when defining template func in template class,
//--  2 template declaraton required
template<class U> template<typename T>
void
SingletonParent<U>::print2Times(T v)
{
  cout << v << endl;
}


//----------------


int
main(int argc, char const* argv[])
{
	  SingletonChild::getInstance()->child_value=1;
	  //SingletonChild *inst2 = SingletonChild::getInstance();
	  SingletonChild::getInstance()->print();

	  //SingletonChild inst2;
	  //inst2.child_value=3;
	  //SingletonChild::getInstance()->print();
	  //inst2.print();

	  //SingletonChildA::getInstance().print();
	  //SingletonChildA::getInstance().a=4;

	  //SingletonChildA singA;
	  //singA.print();
	  //singA.a=0;
	  //SingletonChildA::getInstance().Act();
	  //--  both 0
	  //cout << inst1->get() << endl;

  return 0;
}
