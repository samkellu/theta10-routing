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

struct routing_data {
	vec2 v;
	double dist;
	double cone_left_angle;
	double cone_right_angle;
};

struct color {
	int r;
	int g;
	int b;
	int a;
};

struct edge {
	vec2 points[2];
};

vec2 points[NUM_POINTS + NUM_OBSTACLES * 2 + 2];
edge obstacles[NUM_OBSTACLES];
const int s = 0;
const int t = 1;
const int p_start = 2 + NUM_OBSTACLES * 2;
const int p_end = NUM_POINTS + p_start;
int cur_point = 2;
int num_points = p_start;

void draw_point(SDL_Renderer* renderer, vec2 pos, color c) {

	SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
	SDL_RenderDrawPoint(renderer, pos.x, pos.y);
	SDL_RenderDrawPoint(renderer, pos.x-1, pos.y);
	SDL_RenderDrawPoint(renderer, pos.x+1, pos.y);
	SDL_RenderDrawPoint(renderer, pos.x, pos.y-1);
	SDL_RenderDrawPoint(renderer, pos.x, pos.y+1);
	SDL_RenderDrawPoint(renderer, pos.x-1, pos.y-1);
	SDL_RenderDrawPoint(renderer, pos.x+1, pos.y-1);
	SDL_RenderDrawPoint(renderer, pos.x+1, pos.y+1);
	SDL_RenderDrawPoint(renderer, pos.x-1, pos.y+1);
}

void draw_tri(SDL_Renderer* renderer, vec2 v, color c, double theta, double thetaN, double a) {

	double o = a * tanf(PI / NUM_CONES);
	double h = sqrt(pow(a, 2) + pow(o, 2));
	// Draw triangle
	int blx = v.x + h * cosf(theta);
	int bly = v.y + h * sinf(theta);
	int brx = v.x + h * cosf(thetaN);
	int bry = v.y + h * sinf(thetaN);

	SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
	SDL_RenderDrawLine(renderer, blx, bly, brx, bry);
	SDL_RenderDrawLine(renderer, v.x, v.y, brx, bry);
	SDL_RenderDrawLine(renderer, v.x, v.y, blx, bly);
}

void draw_line(SDL_Renderer* renderer, vec2 v1, vec2 v2, color c) {
	SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
	SDL_RenderDrawLine(renderer, v1.x, v1.y, v2.x, v2.y);
}

void draw(SDL_Renderer* renderer) {
	for (int i = 2; i < num_points; i++) {
		draw_point(renderer, points[i], {255, 255, 255, 50});
	}

	draw_point(renderer, points[s], {255, 0, 0, 255});
	draw_point(renderer, points[t], {0, 0, 255, 255});

	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 50);
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		vec2 p0 = obstacles[i].points[0];
		vec2 p1 = obstacles[i].points[1];
		SDL_RenderDrawLine(renderer, p0.x, p0.y, p1.x, p1.y);
	}

	SDL_SetRenderDrawColor(renderer, 255, 255, 0, 50);
	SDL_RenderDrawLine(renderer, points[s].x, points[s].y, points[t].x, points[t].y);
	SDL_RenderPresent(renderer);
}

vec2 orth_project(vec2 v, edge edge) {
	double recip_m = -(edge.points[1].x - edge.points[0].x) / (edge.points[1].y - edge.points[0].y);
	double m = (edge.points[1].y - edge.points[0].y) / (edge.points[1].x - edge.points[0].x);

	double recip_b = v.y - recip_m * v.x;
	double b = edge.points[0].y - m * edge.points[0].x;

	double x = recip_b / (m - recip_m) - b / (m - recip_m);
	double y = m * x + b;

	return {x, y};
}

double get_orth_distance(vec2 v, edge edge) {
	
	vec2 projected = orth_project(v, edge);
	return sqrt(pow(edge.points[0].y - projected.y, 2) + pow(edge.points[0].x - projected.x, 2));
}

double get_distance_from_edge(vec2 v, edge edge) {
	
	vec2 projected = orth_project(v, edge);
	return sqrt(pow(v.y - projected.y, 2) + pow(v.x - projected.x, 2));
}

vec2 get_intersect(edge l1, edge l2) {

	double x[] = {l2.points[0].x,
				  l2.points[1].x,
				  l1.points[0].x,
				  l1.points[1].x};

	double y[] = {l2.points[0].y,
				  l2.points[1].y,
				  l1.points[0].y,
				  l1.points[1].y};

	double denominator = (x[0]-x[1])*(y[2]-y[3])-(y[0]-y[1])*(x[2]-x[3]);

	// vectors do not ever intersect
	if (denominator == 0) return {-1, -1};

	double t = ((x[0]-x[2])*(y[2]-y[3])-(y[0]-y[2])*(x[2]-x[3]))/denominator;
	double u = -((x[0]-x[1])*(y[0]-y[2])-(y[0]-y[1])*(x[0]-x[2]))/denominator;

	// Vectors do not intersect
	if (t <= 0 || t >= 1 || u <= 0) return {-1, -1};

	return { x[0] + t * (x[1] - x[0]), y[0] + t * (y[1] - y[0]) };
}

bool is_visible(vec2 observer, vec2 pt, edge obstacles[], int n) {

	edge vis_edge = {observer.x, observer.y, pt.x, pt.y};
	double distance = sqrt(pow(observer.x - pt.x, 2) + pow(observer.y - pt.y, 2));

	for (int i = 0; i < n; i++) {
		vec2 hit = get_intersect(vis_edge, obstacles[i]);
		double hit_dist = sqrt(pow(hit.y - observer.y, 2) + pow(hit.x - observer.x, 2));
		if (hit_dist <= distance) return false;
	}

	return true;
}

routing_data* generate_cones(SDL_Renderer* renderer, vec2 v) {

	routing_data* visible_pts = (routing_data*) malloc(sizeof(routing_data) * NUM_CONES);
	for (int i = 0; i < NUM_CONES; i++) {

		vec2 min_pt = {-1, -1};
		double min_bi_dist = pow(2, 31);

		// Find nearest visible point (bisec distance) in cone
		double theta = -PI + (i / (double) NUM_CONES) * 2 * PI;
		double thetaN = -PI + ((i+1) / (double) NUM_CONES) * 2 * PI;
		visible_pts[i] = {-1, -1, -1, theta, thetaN};

		for (int j = 0; j < num_points; j++) {

			vec2 pt = points[j];
			double alpha = atan2(pt.y - v.y, pt.x - v.x);
			// Point is not in current cone
			if (alpha >= thetaN || alpha < theta) continue;

			edge bisect = {
				v.x,
				v.y,
				v.x + cosf(theta + (thetaN - theta) / 2) * CONE_LENGTH,
				v.y + sinf(theta + (thetaN - theta) / 2) * CONE_LENGTH
			};

			double bisect_distance = get_orth_distance(points[j], bisect);
			if (bisect_distance >= min_bi_dist) continue;

			// Checks if the vector from the mouse to the point intersects any walls
			if (!is_visible(v, points[j], obstacles, NUM_OBSTACLES)) continue;

			min_bi_dist = bisect_distance;
			min_pt = points[j];
		}

		// Draw line to nearest point in cone if it exists
		if (min_bi_dist == pow(2, 31)) continue;

		visible_pts[i] = {min_pt.x, min_pt.y, min_bi_dist, theta, thetaN};
	}

	return visible_pts;
}

void route(SDL_Renderer* renderer) {

	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
	SDL_RenderClear(renderer);
	draw(renderer);

	double cone_angles[NUM_CONES-1];
	for (int i = 0; i < NUM_CONES; i++)  {
		-PI + (i / (double) NUM_CONES) * 2 * PI;
	} 

	vec2 cur_point = points[s];
	while (1) {
		routing_data* visible_points = generate_cones(renderer, cur_point);

		int cone = -1; 
		double t_angle = atan2(points[t].y - cur_point.y, points[t].x - cur_point.x);
		for (int i = 0; i < NUM_CONES-1; i++) {
			if (t_angle > cone_angles[i] && t_angle <= cone_angles[i+1]) {
				cone = i;
				break;
			}
		}

		double closest_dist = pow(2, 31);
		routing_data closest_point = {-1, -1, -1, -1, -1};
		edge e = {points[s], points[t]};

		for (int i = cone - 2; i <= cone + 2; i++) {
			routing_data r = visible_points[i % NUM_CONES];
			if (r.dist <= 0) continue;
			double distance = get_distance_from_edge(r.v, e);
			if (distance < closest_dist) {
				closest_dist = distance;
				closest_point = r;
			}
		}

		if (closest_point.dist == -1) break;

		printf("%d %d\n", cur_point.x, cur_point.y);
		draw_tri(renderer, cur_point, {255, 0, 0, 150}, closest_point.cone_left_angle, closest_point.cone_right_angle, closest_point.dist);
		draw_line(renderer, cur_point, closest_point.v, {255, 0, 0, 150});

		if (cur_point.x == closest_point.v.x && cur_point.y == closest_point.v.y) break;
		cur_point = closest_point.v;
		if (cur_point.x == points[t].x && cur_point.y == points[t].y) break;

		SDL_RenderPresent(renderer);
		SDL_Delay(1000);
		free(visible_points);
	}

	SDL_Delay(2000);
	return;
}

int main() {

    if(SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "Could not init SDL: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow("THETA-10 DEMO", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 1000, 1000, 0);
    if(!win) {
        fprintf(stderr, "Could not create window\n");
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(win, -1, SDL_RENDERER_SOFTWARE);
    if(!renderer) {
        fprintf(stderr, "Could not create renderer\n");
        return 1;
    }

	SDL_Delay(1000);

	srand(time(NULL));
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		bool valid = true;
		do {
			obstacles[i] = {(double) (rand() % 1000),
							(double) (rand() % 1000),
							(double) (rand() % 1000),
							(double) (rand() % 1000)};

			for (int j = 0; j < i; j++) {
				valid = get_intersect(obstacles[i],  obstacles[j]).x == -1;
				if (!valid) break;
			}
		} while (!valid);

		points[++cur_point] = obstacles[i].points[0];
		points[++cur_point] = obstacles[i].points[1];
	}

	edge st;
	do {

		points[s] = {(double) (rand() % 1000),
				 	 (double) (rand() % 1000)};

		points[t] = {(double) (rand() % 1000),
				 	 (double) (rand() % 1000)};

		st = {points[s], points[t]};

	} while (!is_visible(points[s], points[t], obstacles, NUM_OBSTACLES));

	// mouse coords
	int mx, my;
	bool running = true;
	while (running) {

		SDL_Delay(10);
		SDL_Event e;
		while (SDL_PollEvent(&e)) {
			switch (e.type) {
				case SDL_QUIT:
					running = false;
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&mx, &my);
					break;

				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&mx, &my);
					points[cur_point].x = mx;
					points[cur_point].y = my;
					cur_point = cur_point + 1 > p_end ? p_start : cur_point + 1;
					num_points += num_points == p_end ? 0 : 1;
					break;

				case SDL_KEYDOWN:
					switch (e.key.keysym.sym) {
						case SDLK_s:
							points[s] = {(double) mx, (double) my};
							st.points[0] = points[s];
							break;

						case SDLK_t:
							points[t] = {(double) mx, (double) my};
							st.points[1] = points[t];
							break;

						case SDLK_SPACE:
							route(renderer);
							break;
					}
					break;
			}
		}

		vec2 m = {(double) mx, (double) my};

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);
		routing_data* visible_points = generate_cones(renderer, m);
		for (int i = 0; i < NUM_CONES; i++) {

			routing_data r = visible_points[i];
			// Draw cone lines
			int cx = m.x + CONE_LENGTH * cos(r.cone_left_angle);
			int cy = m.y + CONE_LENGTH * sin(r.cone_left_angle);
			SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
			SDL_RenderDrawLine(renderer, m.x, m.y, cx, cy);

			draw_tri(renderer, m, {255, 0, 0, 150}, r.cone_left_angle, r.cone_right_angle, r.dist);
			draw_line(renderer, m, r.v, {255, 0, 0, 150});
		}
		free(visible_points);

		draw(renderer);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

