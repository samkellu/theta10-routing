#include "prog.h"
#include "graphics.h"
#include "algorithms.h"

point points[NUM_POINTS + NUM_OBSTACLES * 2 + 2];
edge obstacles[NUM_OBSTACLES];
const int s = 0;
const int t = 1;
const int p_start = 2 + NUM_OBSTACLES * 2;
const int p_end = NUM_POINTS + p_start;
int cur_point = 2;
int num_points = p_start;

void dispose_graph() {
	for (int i = 0; i < num_points; i++) {
		for (int j = 0; j < points[i].num_neighbours; j++)
			free(points[i].neighbours[j]);

		free(points[i].neighbours);
		points[i].neighbours = NULL;
		points[i].num_neighbours = 0;
	}
}

canonical_triangle* get_canonical_tri(point v, double al, double ar) {
 
	point bisect_end = {
		v.x + cosf(al + (ar - al) / 2) * CONE_LENGTH,
		v.y + sinf(al + (ar - al) / 2) * CONE_LENGTH,
		NULL, 0
	};

	edge bisect = {v, bisect_end};
	canonical_triangle* best = (canonical_triangle*) malloc(sizeof(canonical_triangle));
	*best = {NULL, al, ar, 0};
	double min_bi_dist = pow(2, 31);

	for (int j = 0; j < num_points; j++) {
		if (points[j].x == v.x && points[j].y == v.y) continue;

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

	printf("%lf %lf\n", best->al, best->ar);

	if (best->p == NULL) {
		free(best);
		return NULL;
	}


	return best;
}

int get_neighbours(SDL_Renderer* renderer, point v, canonical_triangle*** neighbours) {

	int num_subcones = 0;
	double* subcone_bounds = NULL;

	for (int i = 0; i < NUM_OBSTACLES; i++) {
		point p0 = obstacles[i].points[0];
		point p1 = obstacles[i].points[1];

		point end;
		if (v.x == p0.x && v.y == p0.y) end = p1;
		else if (v.x == p1.x && v.y == p1.y) end = p0;
		else continue;
		
		subcone_bounds = (double*) realloc(subcone_bounds, sizeof(double) * ++num_subcones);
		subcone_bounds[num_subcones - 1] = atan2(end.y - v.y, end.x - v.x);
	}

	int num_neighbours = 0;
	int subcone_curs = 0;
	canonical_triangle** to_add = NULL;
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

		canonical_triangle** neighbours = NULL;
		int num_neighbours = get_neighbours(renderer, points[i], &neighbours);
		for (int j = 0; j < num_neighbours; j++) {

			bool valid = true;
			for (int u = 0; u < points[i].num_neighbours; u++) {
				if (points[i].neighbours[u]->p == neighbours[j]->p) {
					valid = false;
					break;
				}
			}

			if (valid) {
				points[i].neighbours = (canonical_triangle**) realloc(points[i].neighbours, sizeof(canonical_triangle*) * ++points[i].num_neighbours);
				points[i].neighbours[points[i].num_neighbours - 1] = neighbours[j];
			}

			valid = true;
			for (int u = 0; u < neighbours[j]->p->num_neighbours; u++) {
				if (&points[i] == neighbours[j]->p) {
					valid = false;
					break;
				}
			}

			if (valid) {
				neighbours[j]->p->neighbours = (canonical_triangle**) realloc(neighbours[j]->p->neighbours, sizeof(canonical_triangle*) * ++neighbours[j]->p->num_neighbours);
				canonical_triangle* new_tri = (canonical_triangle*) malloc(sizeof(canonical_triangle));
				*new_tri = *neighbours[j];
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

		// cone best_cone = bisect_alg(renderer, cur_point, cones, n_cones, found_cone, points[s], points[t]);
		canonical_triangle* best = low_angle_alg(renderer, cur_point, points[s], points[t]);
		
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
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		bool valid = true;
		do {
			point p1 = {(double) (rand() % 1000), (double) (rand() % 1000), NULL, 0};
			point p2 = {(double) (rand() % 1000), (double) (rand() % 1000), NULL, 0};
			obstacles[i] = {p1, p2};

			for (int j = 0; j < i; j++) {
				valid = get_intersect(obstacles[i],  obstacles[j]).x == -1;
				if (!valid) break;
			}
		} while (!valid);

		points[cur_point++] = obstacles[i].points[0];
		points[cur_point++] = obstacles[i].points[1];
	}

	edge st;
	do {

		points[s] = {
			(double) (rand() % 1000),
			(double) (rand() % 1000),
			NULL,
			0};

		points[t] = {
			(double) (rand() % 1000),
			(double) (rand() % 1000),
			NULL,
			0};

		st = {points[s], points[t]};

	} while (!is_visible(points[s], points[t], obstacles, NUM_OBSTACLES));


	generate_graph(renderer);

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
					dispose_graph();
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&mx, &my);
					break;

				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&mx, &my);
					dispose_graph();
					points[cur_point] = {(double) mx, (double) my, NULL, 0};

					cur_point = cur_point + 1 > p_end ? p_start : cur_point + 1;
					num_points += num_points == p_end ? 0 : 1;

					generate_graph(renderer);
					break;

				case SDL_KEYDOWN:
					switch (e.key.keysym.sym) {
						case SDLK_s:
							dispose_graph();
							points[s] = {(double) mx, (double) my, NULL, 0};

							st.points[0] = points[s];
							generate_graph(renderer);
							break;

						case SDLK_t:
							dispose_graph();
							points[t] = {(double) mx, (double) my, NULL, 0};

							st.points[1] = points[t];
							generate_graph(renderer);
							break;

						case SDLK_SPACE:
							route(renderer);
							break;
					}
					break;
			}
		}

		point p = {(double) mx, (double) my, NULL, 0};
		p.num_neighbours = get_neighbours(renderer, p, &p.neighbours);
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);

		draw(renderer, points, num_points, obstacles, NUM_OBSTACLES, points[s], points[t]);
		for (int j = 0; j < p.num_neighbours; j++) {
            draw_line(renderer, p, *p.neighbours[j]->p, {100, 100, 100, 100});
        }
		SDL_RenderPresent(renderer);

		free(p.neighbours);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}