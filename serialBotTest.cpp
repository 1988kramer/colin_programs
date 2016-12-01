// serialBotTest.cpp
// by Andrew Kramer
// 11/30/2016

// testing for the SerialBot class

#include "SerialBot.h"
#include <pthread.h>

using namespace std;

SerialBot colin;
pthread_t commThread;

void* threadFunction(void* args)
{
	colin.commThreadFunction();
}

int main()
{
	pthread_create(&commThread, NULL, threadFunction, NULL);
}