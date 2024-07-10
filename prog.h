#ifndef GUARD_H
#define GUARD_H

#include <SDL2/SDL.h>
#include <stdlib.h>
#include <time.h>

#define CONE_LENGTH 2000
#define NUM_CONES 10
#define NUM_POINTS 25
#define NUM_OBSTACLES 8
#define PI 3.14159
#define RANDOM_START 1

struct canonical_triangle;

struct vec2 {
	double x;
	double y;
};

struct point {
    double x;
	double y;
    canonical_triangle** neighbours;
    int num_neighbours;
	point* obstacle_endpoint;
};

struct canonical_triangle {
	point* p;
	double al;
	double ar;
	double bisect_distance;
};

struct edge {
	point* points[2];
};

struct cone {
	point v;
	point* closest_pt;
	double dist;
	double cone_left_angle;
	double cone_right_angle;
	edge bisect;
	bool initialized;
	bool is_subcone;
};

struct color {
	int r;
	int g;
	int b;
	int a;
};

#endif