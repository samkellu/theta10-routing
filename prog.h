#ifndef GUARD_H
#define GUARD_H

#include <SDL2/SDL.h>
#include <stdlib.h>
#include <time.h>

#define CONE_LENGTH 1000
#define NUM_CONES 10
#define NUM_POINTS 25
#define NUM_OBSTACLES 8
#define PI 3.14159

struct vec2 {
	double x;
	double y;
};

struct edge {
	vec2 points[2];
};

struct cone {
	vec2 v;
	vec2 closest_pt;
	double dist;
	double cone_left_angle;
	double cone_right_angle;
	edge bisect;
	bool initialized;
	bool is_subcone;
};

struct theta_graph {
	cone* cones;
	int n;
};

struct color {
	int r;
	int g;
	int b;
	int a;
};

#endif