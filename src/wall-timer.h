#ifndef __WALL_TIMER_H__ 
#define __WALL_TIMER_H__ 

#include <sys/time.h>
#include <unistd.h>

struct wall_time_timer {
	timeval tv;
	double start_time, stopped_at;
	wall_time_timer() { start(); }
	void start() {
		gettimeofday(&tv, 0);
		start_time = (double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0;
	}
	void restart() { start(); }
	double look() {
		gettimeofday(&tv, 0);
		stopped_at = (double)tv.tv_sec*1000.0 + (double)tv.tv_usec/1000.0;
		return stopped_at - start_time;
	}
	static double msec(int ms) { return (double)ms; }
	static double sec(int s) { return s*1000.0; }
};

#endif

