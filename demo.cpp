#include "prog.h"
#include "graphics.h"
#include "algorithms.h"

point* points = NULL;
edge* obstacles = NULL;
const int s = 0;
const int t = 1;
int num_points = 2;
int num_obstacles = 0;

void dispose_graph() {
	for (int i = 0; i < num_points; i++) {
		for (int j = 0; j < points[i].num_neighbours; j++)
			free(points[i].neighbours[j]);

		free(points[i].neighbours);
		points[i].neighbours = NULL;
		points[i].num_neighbours = 0;
	}
}

bool point_equals(point a, point b) { return a.x == b.x && a.y == b.y; } 

canonical_triangle* get_canonical_tri(point v, double al, double ar) {
 
	point bisect_end = {
		v.x + cosf(al + (ar - al) / 2) * CONE_LENGTH,
		v.y + sinf(al + (ar - al) / 2) * CONE_LENGTH,
		NULL, 0, NULL
	};

	edge bisect = {&v, &bisect_end};
	canonical_triangle* best = (canonical_triangle*) malloc(sizeof(canonical_triangle));
	*best = {NULL, al, ar, 0};
	double min_bi_dist = pow(2, 31);

	for (int j = 0; j < num_points; j++) {
		if (point_equals(points[j], v)) continue;

		double alpha = atan2(points[j].y - v.y, points[j].x - v.x);
		// Point is not in current cone
		if (alpha >= ar || alpha < al) continue;
		
		double bisect_distance = get_orth_distance(points[j], bisect);
		if (bisect_distance >= min_bi_dist) continue;
		// Checks if the vector to the point intersects any walls
		if (!is_visible(v, points[j], obstacles, NUM_OBSTACLES)) continue;

		best->p = &points[j];
		best->bisect_distance = bisect_distance;
		min_bi_dist = bisect_distance;
	}

	if (best->p == NULL) {
		free(best);
		return NULL;
	}


	return best;
}

int get_neighbours(SDL_Renderer* renderer, point v, canonical_triangle*** neighbours) {

	int num_subcones = 0;
	double* subcone_bounds = NULL;
	int num_neighbours = 0;
	canonical_triangle** to_add = NULL;

	for (int i = 0; i < num_obstacles; i++) {
		point* p0 = obstacles[i].points[0];
		point* p1 = obstacles[i].points[1];
		point* end;
		if (point_equals(v, *p0)) end = p1;
		else if (point_equals(v, *p1)) end = p0;
		else continue;
		
		to_add = (canonical_triangle**) realloc(to_add, sizeof(canonical_triangle*) * ++num_neighbours);
		to_add[num_neighbours - 1] = (canonical_triangle*) malloc(sizeof(canonical_triangle));
		double alpha = atan2(end->y - v.y, end->x - v.x);
		*to_add[num_neighbours - 1] = {end, alpha, alpha, 0};

		subcone_bounds = (double*) realloc(subcone_bounds, sizeof(double) * ++num_subcones);
		subcone_bounds[num_subcones - 1] = alpha;
	}

	int subcone_curs = 0;
	for (int i = 0; i < NUM_CONES; i++) {

		// Find nearest visible point (bisect distance) in cone
		double al = -PI + (i / (double) NUM_CONES) * 2 * PI;
		double ar = -PI + ((i+1) / (double) NUM_CONES) * 2 * PI;
		while (subcone_curs < num_subcones && ar > subcone_bounds[subcone_curs] && al < subcone_bounds[subcone_curs]) {
			canonical_triangle* best = get_canonical_tri(v, al, subcone_bounds[subcone_curs]);
			al = subcone_bounds[subcone_curs++];
			if (best == NULL) continue;

			to_add = (canonical_triangle**) realloc(to_add, sizeof(canonical_triangle*) * ++num_neighbours);
			to_add[num_neighbours - 1] = best;
		}

		canonical_triangle* best = get_canonical_tri(v, al, ar);
		if (best == NULL) continue;

		to_add = (canonical_triangle**) realloc(to_add, sizeof(canonical_triangle*) * ++num_neighbours);
		to_add[num_neighbours - 1] = best;				
	}

	free(subcone_bounds);
	*neighbours = to_add;
	return num_neighbours;
}

void generate_graph(SDL_Renderer* renderer) {

	for (int i = 0; i < num_points; i++) {

		// 2x free??? TODO
		canonical_triangle** neighbours = NULL;
		int num_neighbours = get_neighbours(renderer, points[i], &neighbours);
		for (int j = 0; j < num_neighbours; j++) {

			// Add j to i if not exists
			bool valid = true;
			for (int u = 0; u < points[i].num_neighbours; u++) {
				if (point_equals(*points[i].neighbours[u]->p, *neighbours[j]->p)) {
					valid = false;
					break;
				}
			}

			if (valid) {
				points[i].neighbours = (canonical_triangle**) realloc(points[i].neighbours, sizeof(canonical_triangle*) * ++points[i].num_neighbours);
				points[i].neighbours[points[i].num_neighbours - 1] = neighbours[j];
			}

			// Add i to j if not exists
			valid = true;
			for (int u = 0; u < neighbours[j]->p->num_neighbours; u++) {
				if (point_equals(points[i], *neighbours[j]->p)) {
					valid = false;
					break;
				}
			}

			if (valid) {
				neighbours[j]->p->neighbours = (canonical_triangle**) realloc(neighbours[j]->p->neighbours, sizeof(canonical_triangle*) * ++neighbours[j]->p->num_neighbours);
				canonical_triangle* new_tri = (canonical_triangle*) malloc(sizeof(canonical_triangle));
				*new_tri = *neighbours[j];
				new_tri->p = &points[i];
				new_tri->al += PI;
				new_tri->ar += PI;
				neighbours[j]->p->neighbours[neighbours[j]->p->num_neighbours - 1] = new_tri;
			}
		}
	}
}

void route(SDL_Renderer* renderer) {

	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
	SDL_RenderClear(renderer);
	draw(renderer, points, num_points, obstacles, NUM_OBSTACLES, points[s], points[t]);

	point cur_point = points[s];
	int max_steps = 20;
	while (max_steps-- >= 0) {

		canonical_triangle* best = cone_alg(renderer, cur_point, points[s], points[t]);
		// canonical_triangle* best = low_angle_alg(renderer, cur_point, points[s], points[t]);
		
		if (best == NULL) {
			printf("Failed to find route\n");
			break;
		}

		draw_line(renderer, cur_point, *(best->p), {255, 0, 0, 150});

		draw_tri(renderer, cur_point, *best, {255, 0, 0, 150});

		SDL_RenderPresent(renderer);
		SDL_Delay(1000);
		if (best->p->x == points[t].x && best->p->y == points[t].y) break;

		// SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
		// for (int i = 0; i < n_cones; i++) {
		// 	cone c = cones[i];
		
		// 	int cx = cur_point.x + CONE_LENGTH * sin(c.cone_left_angle);
		// 	int cy = cur_point.y + CONE_LENGTH * cos(c.cone_left_angle);
		// 	SDL_RenderDrawLine(renderer, cur_point.x, cur_point.y, cx, cy);
		// 	int crx = cur_point.x + CONE_LENGTH * sin(c.cone_right_angle);
		// 	int cry = cur_point.y + CONE_LENGTH * cos(c.cone_right_angle);
		// 	SDL_RenderDrawLine(renderer, cur_point.x, cur_point.y, crx, cry);
		// }

		// draw_tri(renderer, cur_point, {255, 0, 0, 150}, best_cone.cone_left_angle, best_cone.cone_right_angle, best_cone.dist);
		// draw_line(renderer, cur_point, best_pt, {255, 0, 0, 150});

		cur_point = *best->p; 
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

	obstacles = (edge*) malloc(sizeof(edge) * NUM_OBSTACLES);
	points = (point*) malloc(sizeof(point) * NUM_OBSTACLES * 2 + 2);

	for (int i = 0; i < NUM_OBSTACLES; i++) {
		bool valid = true;
		do {
			point p1 = {(double) (rand() % 1000), (double) (rand() % 1000), NULL, 0, NULL};
			point p2 = {(double) (rand() % 1000), (double) (rand() % 1000), NULL, 0, NULL};
			obstacles[i] = {&p1, &p2};

			for (int j = 0; j < i; j++) {
				valid = get_intersect(obstacles[i],  obstacles[j]).x == -1;
				if (!valid) break;
			}
		} while (!valid);

		obstacles[i].points[0]->obstacle_endpoint = obstacles[i].points[1];
		obstacles[i].points[1]->obstacle_endpoint = obstacles[i].points[0];
		points[num_points++] = *obstacles[i].points[0];
		points[num_points++] = *obstacles[i].points[1];
	}

	num_obstacles = NUM_OBSTACLES;	
	edge st;
	do {

		points[s] = {(double) (rand() % 1000), (double) (rand() % 1000), NULL, 0, NULL};
		points[t] = {(double) (rand() % 1000), (double) (rand() % 1000), NULL, 0, NULL};
		st = {&points[s], &points[t]};

	} while (!is_visible(points[s], points[t], obstacles, NUM_OBSTACLES)
			 || sqrt(pow(points[s].x - points[t].x, 2) + pow(points[s].y - points[t].y, 2)) < 600);

	generate_graph(renderer);

	// mouse coords
	int mx, my;
	bool running = true;
	point* obstacle_prev = NULL;
	while (running) {

		SDL_Delay(10);
		SDL_Event e;
		while (SDL_PollEvent(&e)) {
			switch (e.type) {
				case SDL_QUIT:
					running = false;
					dispose_graph();
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&mx, &my);
					break;

				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&mx, &my);
					dispose_graph();

					printf("here %d\n",  sizeof(point) * (num_points + 1));
	fflush(stdout);
					points = (point*) realloc(points, sizeof(point) * (num_points + 1));
					points[num_points] = {(double) mx, (double) my, NULL, 0, NULL};
					if  (e.button.button == SDL_BUTTON_RIGHT) {
						
						if (obstacle_prev != NULL) {
							obstacle_prev->obstacle_endpoint = &points[num_points];
							points[num_points].obstacle_endpoint = obstacle_prev;
							obstacles = (edge*) realloc(obstacles, sizeof(edge) * (num_obstacles + 1));
							printf("herer %d\n",  num_obstacles);
	fflush(stdout);
							obstacles[num_obstacles++] = {obstacle_prev, &points[num_points]};
							obstacle_prev = NULL;

						} else {
							obstacle_prev = &points[num_points];
						}
					}

					num_points++;
					generate_graph(renderer);
					break;

				case SDL_KEYDOWN:
					switch (e.key.keysym.sym) {
						case SDLK_s:
							dispose_graph();
							points[s] = {(double) mx, (double) my, NULL, 0, NULL};

							st.points[0] = &points[s];
							generate_graph(renderer);
							break;

						case SDLK_t:
							dispose_graph();
							points[t] = {(double) mx, (double) my, NULL, 0, NULL};

							st.points[1] = &points[t];
							generate_graph(renderer);
							break;

						case SDLK_SPACE:
							route(renderer);
							break;
					}
					break;
			}
		}

		point p = {(double) mx, (double) my, NULL, 0, NULL};
		p.num_neighbours = get_neighbours(renderer, p, &p.neighbours);

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);

		draw(renderer, points, num_points, obstacles, num_obstacles, points[s], points[t]);
		for (int j = 0; j < p.num_neighbours; j++) {
            draw_line(renderer, p, *p.neighbours[j]->p, {100, 100, 100, 100});
        }
		SDL_RenderPresent(renderer);

		free(p.neighbours);
	}

	free(points);
	free(obstacles);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}