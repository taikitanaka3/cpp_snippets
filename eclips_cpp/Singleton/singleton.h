/*
 * singleton.h
 *
 *  Created on: Jan 22, 2020
 *      Author: tanakasan
 */

#ifndef SINGLETON_H_
#define SINGLETON_H_
#include <iostream>
#include <string>

using namespace std;

template <class U>
class SingletonParent
{
  public:
    static U* getInstance() {
      static U instance;
      return &instance;
    }

  protected:
    //--  template func in template class available
    template<typename T> void print2Times(T v);

    SingletonParent() {
      base_str = string("BASE");
    }
    virtual ~SingletonParent() {}
    string base_str;
};


//--  use template succession
class SingletonChild : public SingletonParent<SingletonChild> {
  public:
    SingletonChild() {};
    ~SingletonChild() {};

    //void set(int v) { child_value = v; }
    //int  get()      { return child_value; }

    void print()    {
      print2Times(child_value);
    }
  public:
    int child_value=10;
};


template <class U>
class SingletonParentA
{
  public:
    static U& getInstance() {
      static U instance;
      return instance;
    }

  protected:
    //--  template func in template class available
    template<typename T> void print2Times(T v);
    void Act(){
    	cout<<"Act";
    }

    SingletonParentA() {
      base_str = string("BASE");
    }
    virtual ~SingletonParentA() {}
    string base_str;
};

//--  use template succession
class SingletonChildA : public SingletonParentA<SingletonChildA> {
public:
    SingletonChildA() {
      child_value = 0;
    }
    ~SingletonChildA() {}
public:
    void set(int v) { child_value = v; }
    int  get()      { return child_value; }

    void print()    {
    	cout<<a;
      //print2Times(base_str);
      //print2Times(child_value);
    }
    int a=2;
  private:
    int child_value;
};

#endif /* SINGLETON_H_ */
