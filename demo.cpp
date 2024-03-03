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

struct color {
	int r;
	int g;
	int b;
	int a;
};

struct edge {
	vec2 points[2];
};

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

double get_bisect_distance(vec2 v, edge bisect) {
	
	double recip_m = -(bisect.points[1].x - bisect.points[0].x) / (bisect.points[1].y - bisect.points[0].y);
	double m = (bisect.points[1].y - bisect.points[0].y) / (bisect.points[1].x - bisect.points[0].x);

	double recip_b = v.y - recip_m * v.x;
	double b = bisect.points[0].y - m * bisect.points[0].x;

	double x = recip_b / (m - recip_m) - b / (m - recip_m);
	double y = m * x + b;
	
	return sqrt(pow(bisect.points[0].y - y, 2) + pow(bisect.points[0].x - x, 2));
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

	const int s = 0;
	const int t = 1;
	const int p_start = 2;
	const int p_end = NUM_POINTS + p_start;
	int cur_point = p_start;
	int num_points = 2;
	vec2 points[NUM_POINTS + 2];
	edge obstacles[NUM_OBSTACLES];

	srand(time(NULL));
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		obstacles[i] = {(double) (rand() % 1000),
						(double) (rand() % 1000),
						(double) (rand() % 1000),
						(double) (rand() % 1000)};
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
					}
					break;
			}
		}

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);
		for (int i = 0; i < NUM_CONES; i++) {

			vec2 min_pt = {-1, -1};
			double min_bi_dist = pow(2, 31);

			// Find nearest visible point (bisect distance) in cone
			double theta = -PI + (i / (double) NUM_CONES) * 2 * PI;
			double thetaN = -PI + ((i+1) / (double) NUM_CONES) * 2 * PI;

			edge bisect = {
				(double) mx,
				(double) my,
				mx + cosf(theta + (thetaN - theta) / 2) * CONE_LENGTH,
				my + sinf(theta + (thetaN - theta) / 2) * CONE_LENGTH
			};

			for (int j = 0; j < num_points; j++) {

				vec2 pt = points[j];
				double alpha = atan2(pt.y - my, pt.x - mx);
				// Point is not in current cone
				if (alpha >= thetaN || alpha < theta) continue;

				double bisect_distance = get_bisect_distance(points[j], bisect);
				if (bisect_distance >= min_bi_dist) continue;

				// Checks if the vector from the mouse to the point intersects any walls
				if (!is_visible({(double) mx, (double) my}, points[j], obstacles, NUM_OBSTACLES)) continue;

				min_bi_dist = bisect_distance;
				min_pt = points[j];
			}

			// Draw cone lines
			int cx = mx + CONE_LENGTH * cos(theta);
			int cy = my + CONE_LENGTH * sin(theta);
			SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
			SDL_RenderDrawLine(renderer, mx, my, cx, cy);

			// Draw line to nearest point in cone if it exists
			if (min_bi_dist == pow(2, 31)) continue;

			double tri_width = min_bi_dist * tanf(PI / NUM_CONES);
			double tri_length = sqrt(pow(min_bi_dist, 2) + pow(tri_width, 2));
			// Draw triangle
			int blx = mx + tri_length * cosf(theta);
			int bly = my + tri_length * sinf(theta);
			int brx = mx + tri_length * cosf(thetaN);
			int bry = my + tri_length * sinf(thetaN);

			SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
			SDL_RenderDrawLine(renderer, mx, my, min_pt.x, min_pt.y);
			SDL_RenderDrawLine(renderer, blx, bly, brx, bry);
			SDL_RenderDrawLine(renderer, mx, my, brx, bry);
			SDL_RenderDrawLine(renderer, mx, my, blx, bly);
		}

		for (int i = p_start; i < num_points; i++) {
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
		SDL_RenderDrawLine(renderer, st.points[0].x, st.points[0].y, st.points[1].x, st.points[1].y);
		SDL_RenderPresent(renderer);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

