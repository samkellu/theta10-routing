#include "prog.h"
#include "graphics.h"
#include "algorithms.h"

point* points = (point*) malloc(sizeof(point) * (NUM_POINTS + NUM_OBSTACLES * 2 + 2));
edge obstacles[NUM_OBSTACLES];
const int s = 0;
const int t = 1;
const int p_start = 2 + NUM_OBSTACLES * 2;
const int p_end = NUM_POINTS + p_start;
int cur_point = 2;
int num_points = p_start;

void dispose_graph() {
	for (int i = 0; i < num_points; i++) {
		free(points[i].neighbours);
		points[i].neighbours = NULL;
	}
}

double* get_subcones(point v, int* n) {

	int n_subcones = 0;
	double* subcone_bounds = (double*) malloc(0);

	for (int i = 0; i < NUM_OBSTACLES; i++) {
		point p0 = obstacles[i].points[0];
		point p1 = obstacles[i].points[1];

		int end_x = -1, end_y = -1;
		if (v.x == p0.x && v.y == p0.y) {end_x = p1.x; end_y = p1.y;}
		if (v.x == p1.x && v.y == p1.y) {end_x = p0.x; end_y = p0.y;}
		if (end_x == -1) continue;
		
		subcone_bounds = (double*) realloc(subcone_bounds, sizeof(double) * (n_subcones + 1));
		subcone_bounds[n_subcones++] = atan2(end_y - v.y, end_x - v.x);
	}

	*n = n_subcones;
	return subcone_bounds;
}

point** get_neighbours(SDL_Renderer* renderer, point* v, int* num_neighbours) {

	int n_subcones;
	double* subcone_bounds = get_subcones(*v, &n_subcones);
	int n_cones = NUM_CONES + n_subcones;

	int curs = 0;
	int subcone_curs = 0;
	cone* cones = (cone*) malloc(sizeof(cone) * n_cones);
	for (int i = 0; i < NUM_CONES; i++) {

		// Find nearest visible point (bisect distance) in cone
		double theta = -PI + (i / (double) NUM_CONES) * 2 * PI;
		double thetaN = -PI + ((i+1) / (double) NUM_CONES) * 2 * PI;

		point endpoint = {v->x + cosf(theta + (thetaN - theta) / 2) * CONE_LENGTH,
						  v->y + sinf(theta + (thetaN - theta) / 2) * CONE_LENGTH,
					      NULL, 0};

		edge bisect = {*v, endpoint};

		while (subcone_curs < n_subcones && thetaN > subcone_bounds[subcone_curs] && theta < subcone_bounds[subcone_curs]) {			
			cones[curs++] = {*v, NULL, pow(2, 31), theta, subcone_bounds[subcone_curs], bisect, 1, 1};
			theta = subcone_bounds[subcone_curs++];
		}

		cones[curs++] = {*v, NULL, pow(2, 31), theta, thetaN, bisect, 1, 0};
	}

	free(subcone_bounds);

	point** neighbours = (point**) malloc(sizeof(point*) * n_cones);

	for (int i = 0; i < n_cones; i++) {
		
		neighbours[i] = NULL;
		cone c = cones[i];
		double theta = c.cone_left_angle;
		double thetaN = c.cone_right_angle;
		printf("%lf %lf\n", theta, thetaN);
		double min_bi_dist = pow(2, 31);
		edge bisect = c.bisect;

		for (int j = 0; j < num_points; j++) {

			point u = points[j];
			if (u.x == v->x && u.y == v->y) continue;

			double alpha = atan2(u.y - v->y, u.x - v->x);
			// Point is not in current cone
			if (alpha >= thetaN || alpha < theta) continue;
			
			double bisect_distance = get_orth_distance(u, bisect);
			if (bisect_distance >= min_bi_dist) continue;
			// Checks if the vector to the point intersects any walls
			if (!is_visible(*v, u, obstacles, NUM_OBSTACLES)) continue;

			neighbours[i] = &points[j];
			min_bi_dist = bisect_distance;
		}
	}

	free(cones);
	*num_neighbours = n_cones;
	return neighbours;
}

void generate_graph(SDL_Renderer* renderer) {
	for (int i = 0; i < num_points; i++) {


		int num_neighbours;
		point** neighbours = get_neighbours(renderer, &points[i], &num_neighbours);

		bool valid = true;
		for (int i = 0; i < num_neighbours; i++) {
			if (neighbours[i] == NULL) continue;
				points[i].num_neighbours++;
				points[i].neighbours = (point**) realloc(points[i].neighbours, sizeof(point*) * points[i].num_neighbours);
				points[i].neighbours[points[i].num_neighbours - 1] = neighbours[i];
				// printf("%lf %lf %lf %lf neighbours after %d\n\n",v.x, v.y, neighbours[i]->x, neighbours[i]->y, v.num_neighbours);
// 			for (int j = 0; j < v.num_neighbours; j++) {
// 				if (v.neighbours[j]->x == neighbours[i]->x && v.neighbours[j]->y == neighbours[i]->y) {
// 					valid = false;
// 					break;
// 				}
// 			}

// 			if (valid) {
// 				printf("neighbours before %d\n", v.num_neighbours);
// 				fflush(stdout);
// 				printf("isnull %d\n", v.neighbours == NULL);
// 				fflush(stdout);
// 			}
// 		}
// printf("fuck\n");
// 				fflush(stdout);
// 		for (int j = 0; j < v.num_neighbours; j++) {
// 			point* neighbour = v.neighbours[j];
// 			printf("nullies %d\n", neighbour == NULL);
// 				fflush(stdout);
// 				printf("%d\n", neighbour->num_neighbours);

// 			valid = true;
// 			for (int u = 0; u < neighbour->num_neighbours; u++) {
// 				if (v.x == neighbour->neighbours[u]->x && v.neighbours[j]->y == neighbour->neighbours[u]->y) {
// 					valid = false;
// 					break;
// 				}
// 			}

// 			if (valid) {
// 				neighbour->num_neighbours++;
// 				neighbour->neighbours = (point**) realloc(neighbour->neighbours, sizeof(point*) * neighbour->num_neighbours);
// 				neighbour->neighbours[neighbour->num_neighbours - 1] = &v;
// 			}
		}
	}
}

// void route(SDL_Renderer* renderer) {

// 	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
// 	SDL_RenderClear(renderer);
// 	draw(renderer, points, num_points, obstacles, NUM_OBSTACLES, points[s], points[t]);

// 	vec2 cur_point = points[s];
// 	cone* cones;
// 	int max_steps = 20;
// 	while (max_steps-- >= 0) {
// 		int n_cones = 0;
// 		cones = generate_cones(renderer, cur_point, &n_cones);

// 		cone found_cone; 
// 		found_cone.initialized = false;
// 		double t_angle = atan2(points[t].y - cur_point.y, points[t].x - cur_point.x);
// 		for (int i = 0; i < n_cones; i++) {
// 			cone c = cones[i];
// 			if (t_angle > c.cone_left_angle && t_angle <= c.cone_right_angle) {
// 				found_cone = c;
// 				break;
// 			}
// 		}

// 		if (!found_cone.initialized) {
// 			printf("No bueno\n");
// 			return;
// 		}

// 		printf("fc %d %d\n", found_cone.closest_pt.x, found_cone.closest_pt.y);
// 		// cone best_cone = bisect_alg(renderer, cur_point, cones, n_cones, found_cone, points[s], points[t]);
// 		cone best_cone = low_angle_alg(renderer, cur_point, cones, n_cones, found_cone, points[s], points[t]);
		
// 		if (!best_cone.initialized) {
// 			printf("Failed to find good cone\n");
// 			break;
// 		}

// 		vec2 best_pt = best_cone.closest_pt;

// 		draw_tri(renderer, cur_point, {255, 0, 0, 150}, best_cone.cone_left_angle, best_cone.cone_right_angle, best_cone.dist);
// 		draw_line(renderer, cur_point, best_pt, {255, 0, 0, 150});

// 		SDL_RenderPresent(renderer);
// 		SDL_Delay(1000);
// 		if (best_pt.x == points[t].x && best_pt.y == points[t].y) break;
// 		if (cur_point.x == best_pt.x && cur_point.y == best_pt.y) break;

// 		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
// 		for (int i = 0; i < n_cones; i++) {
// 			cone c = cones[i];
		
// 			int cx = cur_point.x + CONE_LENGTH * sin(c.cone_left_angle);
// 			int cy = cur_point.y + CONE_LENGTH * cos(c.cone_left_angle);
// 			SDL_RenderDrawLine(renderer, cur_point.x, cur_point.y, cx, cy);
// 			int crx = cur_point.x + CONE_LENGTH * sin(c.cone_right_angle);
// 			int cry = cur_point.y + CONE_LENGTH * cos(c.cone_right_angle);
// 			SDL_RenderDrawLine(renderer, cur_point.x, cur_point.y, crx, cry);
// 		}

// 		draw_tri(renderer, cur_point, {255, 0, 0, 150}, best_cone.cone_left_angle, best_cone.cone_right_angle, best_cone.dist);
// 		draw_line(renderer, cur_point, best_pt, {255, 0, 0, 150});

// 		SDL_RenderPresent(renderer);
// 		cur_point = best_pt; 

// 		free(cones);
// 		cones = NULL;
// 	}

// 	if (cones) free(cones);
// 	SDL_Delay(2000);
// 	return;
// }

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
					free(points);
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&mx, &my);
					break;

				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&mx, &my);
					points[cur_point] = {(double) mx, (double) my, NULL, 0};

					cur_point = cur_point + 1 > p_end ? p_start : cur_point + 1;
					num_points += num_points == p_end ? 0 : 1;

					dispose_graph();
					generate_graph(renderer);
					break;

				case SDL_KEYDOWN:
					switch (e.key.keysym.sym) {
						case SDLK_s:
							points[s] = {(double) mx, (double) my, NULL, 0};

							st.points[0] = points[s];
							dispose_graph();
							generate_graph(renderer);
							break;

						case SDLK_t:
							points[t] = {(double) mx, (double) my, NULL, 0};

							st.points[1] = points[t];
							dispose_graph();
							generate_graph(renderer);
							break;

						case SDLK_SPACE:
							// route(renderer);
							break;
					}
					break;
			}
		}

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);

		draw(renderer, points, num_points, obstacles, NUM_OBSTACLES, points[s], points[t]);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}