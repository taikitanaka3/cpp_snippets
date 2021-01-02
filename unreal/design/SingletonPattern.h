#pragma once
#include <iostream>

template <class U>
class Singleton
{
public:
	static U *instance()
	{
		static U Instance;
		return &Instance;
	}

protected:
	Singleton()
	{
		std::cout << "singleton parent called" << std::endl;
	}
	virtual ~Singleton() {}
};