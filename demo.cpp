#include <SDL2/SDL.h>
#include <stdlib.h>
#include <time.h>

#define CONE_LENGTH 1000
#define NUM_CONES 10
#define NUM_POINTS 25
#define NUM_OBSTACLES 8
#define PI 3.14159

struct vec2 {
	float x;
	float y;
};

struct color {
	int r;
	int g;
	int b;
	int a;
};

struct obstacle {
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

	vec2 points[NUM_POINTS + 2];
	obstacle obstacles[NUM_OBSTACLES];
	int cur_point = 2;
	int num_points = 2;

	vec2* s = &points[0];
	vec2* t = &points[1];

	srand(time(NULL));
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		obstacles[i] = {(float) (rand() % 1000),
						(float) (rand() % 1000),
						(float) (rand() % 1000),
						(float) (rand() % 1000)};
	}

	SDL_Event e;
	// mouse coords
	int x3, y3;
	bool running = true;
	while (running) {

		SDL_Delay(10);
		while (SDL_PollEvent(&e)) {

			switch (e.type) {
				case SDL_QUIT:
					running = false;
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&x3, &y3);
					break;

				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&x3, &y3);
					points[cur_point].x = x3;
					points[cur_point].y = y3;
					cur_point = (cur_point + 1) % (NUM_POINTS + 2);
					cur_point = cur_point == 0 ? 2 : cur_point;
					printf("%d\n", cur_point);
					num_points = num_points + 1 > NUM_POINTS + 2 ? NUM_POINTS : num_points + 1;
					break;

				case SDL_KEYDOWN:
					switch (e.key.keysym.sym) {
						case SDLK_s:
							*s = {(float) x3, (float) y3};
							break;

						case SDLK_t:
							*t = {(float) x3, (float) y3};
							break;
					}
			}
		}

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);
		for (int i = 0; i < NUM_CONES; i++) {

			vec2 min_pt = {-1, -1};
			float min_dist = pow(2, 31);
			float closest_wall = pow(2, 31);

			// Find nearest visible point in cone
			float theta = -PI + (i / (float) NUM_CONES) * 2 * PI;
			float thetaN = -PI + ((i+1) / (float) NUM_CONES) * 2 * PI;
			for (int j = 0; j < num_points; j++) {

				float x4 = points[j].x;
				float y4 = points[j].y;
				float alpha = atan2(y4 - y3, x4 - x3);

				if (alpha >= thetaN || alpha < theta) continue;
				// Checks if the vector from the mouse to the point intersects any walls
				for (int w = 0; w < NUM_OBSTACLES; w++) {
					float x1 = obstacles[w].points[0].x;
					float y1 = obstacles[w].points[0].y;
					float x2 = obstacles[w].points[1].x;
					float y2 = obstacles[w].points[1].y;

					float denominator = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);

					// vectors do not ever intersect
					if (denominator == 0) continue;

					float t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denominator;
					float u = -((x1-x2)*(y1-y3)-(y1-y2)*(x1-x3))/denominator;

					// Vectors do not intersect
					if (t <= 0 || t >= 1 || u <= 0) continue;

					vec2 hit = { x1 + t * (x2 - x1), y1 + t * (y2 - y1) };
					float hit_dist = sqrt(pow(hit.y - y3, 2) + pow(hit.x - x3, 2));
					// Checks if the intersected wall is the closest to the camera
					closest_wall = std::min(closest_wall, hit_dist);
				}

				float distance = sqrt(pow(x4 - x3, 2) + pow(y4 - y3, 2));
				if (distance < min_dist && distance < closest_wall) {
					min_dist = distance;
					min_pt = points[j];
				}
			}

			// Draw cone lines
			int vx = CONE_LENGTH * cos(theta);
			int vy = CONE_LENGTH * sin(theta);
			SDL_SetRenderDrawColor(renderer, 100, 100, 100, 50);
			SDL_RenderDrawLine(renderer, x3, y3, x3 + vx, y3 + vy);

			// Draw line to nearest point in cone if it exists
			if (min_pt.x < 0) continue;

			SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
			SDL_RenderDrawLine(renderer, x3, y3, min_pt.x, min_pt.y);
		}

		for (int i = 2; i < NUM_POINTS + 2; i++) {
			draw_point(renderer, points[i], {255, 255, 255, 50});
		}

		draw_point(renderer, *s, {255, 0, 0, 255});
		draw_point(renderer, *t, {0, 0, 255, 255});

		SDL_SetRenderDrawColor(renderer, 0, 255, 0, 50);
		for (int i = 0; i < NUM_OBSTACLES; i++) {
			vec2 p0 = obstacles[i].points[0];
			vec2 p1 = obstacles[i].points[1];
			SDL_RenderDrawLine(renderer, p0.x, p0.y, p1.x, p1.y);
		}
		SDL_RenderPresent(renderer);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

