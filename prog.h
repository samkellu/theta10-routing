#ifndef GUARD_H
#define GUARD_H

#include <SDL2/SDL.h>
#include <stdlib.h>
#include <time.h>

#define PI 3.14159
#define CONE_LENGTH 2000

#ifndef NUM_CONES
#define NUM_CONES 10
#endif

#ifdef RANDOM_START
#define NUM_OBSTACLES 8
#else
#define NUM_OBSTACLES 0
#endif

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
	int p_idx[2];
};

struct pl_edge {
	point points[2];
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