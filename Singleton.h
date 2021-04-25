// by Mark Featherstone (C) 2019 All Rights Reserved.
#ifndef SINGLETON_H
#define SINGLETON_H

#include <cassert>

template<class T> class Singleton
{
public:
	Singleton()
	{
		assert( spSingleton == nullptr);
		spSingleton = static_cast<T*>(this);
	}

	virtual ~Singleton()
	{
		assert( spSingleton != nullptr);
		spSingleton = nullptr;
	}

	static T& Get()	{ 
		assert(spSingleton!=nullptr);
		return *spSingleton; 
	}

private:
	static T* spSingleton;
	Singleton(Singleton const&) = delete;
	void operator=(Singleton const&) = delete;
};

template<typename T> T* Singleton<T>::spSingleton = 0;

#endif
