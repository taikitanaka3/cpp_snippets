//============================================================================
// Name        : NamespaceTest.cpp
// Author      : taikitanaka
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;
#include <map>

using std::map;

namespace map_ {
int a;
}

int main() {
  map_::a = 0;

  map<int, int> b;
  cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!
  return 0;
}
