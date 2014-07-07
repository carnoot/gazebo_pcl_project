#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <mutex>
#include "ros/ros.h"

class thread_class {

public:
	std::string name;
	int shared_variable;

public:

	boost::thread *my_thread;
	std::mutex mut;
	thread_class();
	~thread_class();
	void setName(std::string str);
	std::string getName();

	void doNothing();

};

thread_class::thread_class() {
	std::cout << "Constructor of thread class!" << std::endl;
	this->setName("Default");
	this->my_thread = new boost::thread(
			boost::bind(&thread_class::doNothing, this));
}

thread_class::~thread_class() {
	std::cout << "Destructor of thread class!" << std::endl;
}

std::string thread_class::getName() {
	return this->name;
}

void thread_class::setName(std::string str) {
	this->name = str;
}

void thread_class::doNothing() {
	while (1) {
		mut.lock();
		std::cout << "THREAD Acquired mutex!" << std::endl;
		this->shared_variable++;
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
		std::cout << "THREAD Released mutex!" << std::endl;
		mut.unlock();
	}
}

int main(int argc, char** argv) {

	thread_class *tr1 = new thread_class();
	while (1) {
		tr1->mut.lock();
		std::cout << "MAIN Acquired mutex!" << std::endl;
		std::cout << tr1->shared_variable << std::endl;
		std::cout << "MAIN Released mutex!" << std::endl;
		tr1->mut.unlock();
		boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	}

}
