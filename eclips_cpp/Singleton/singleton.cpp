#include "singleton.h"
#include <iostream>
#include <string>

using namespace std;

//----------------

//--  when defining template func in template class,
//--  2 template declaraton required
template <class U>
template <typename T>
void SingletonParent<U>::print2Times(T v) {
  cout << v << endl;
}

//----------------

int main(int argc, char const *argv[]) {
  SingletonChild::getInstance()->child_value = 1;
  // SingletonChild *inst2 = SingletonChild::getInstance();
  SingletonChild::getInstance()->print();
  return 0;
}
