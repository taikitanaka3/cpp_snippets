#pragma once
#include "SingletonPattern.h"
#include <iostream>
class StatePattern{
public:
	virtual void Action(int time)=0;
private:
};

class OnEnter:public StatePattern,public Singleton<OnEnter>{
	friend class Singleton<OnEnter>;
public:
	void Action(int time){
		std::cout<<"PM"<<(time-12)<<":00"<<std::endl;
	}
};

class OnStay:public StatePattern,public Singleton<OnStay>{
	friend class Singleton<OnStay>;
public:
	void Action(int time){
		std::cout<<"Stay"<<(time)<<":00"<<std::endl;
	}
};

class OnExit:public StatePattern,public Singleton<OnExit>{
	friend class Singleton<OnExit>;
public:
	void Action(int time){
		std::cout<<"AM"<<(time)<<":00"<<std::endl;
	}
};


class Clock{
public:
	Clock(void){
		state_=new OnExit();
	}

	void Action(int time){
		if(time<10){
			state_=OnEnter::instance();
		}
		else if(10<time&&time<14){
			state_=OnStay::instance();
		}
		else{
			state_=OnExit::instance();
		}

		state_->Action(time);
	}
private:
	StatePattern *state_;
};