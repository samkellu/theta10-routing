#include <SDL2/SDL.h>
#include <stdlib.h>
#include <time.h>

#define CONE_LENGTH 1000
#define NUM_CONES 10
#define NUM_POINTS 25
#define NUM_OBSTACLES 8
#define PI 3.14159

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

	int points[NUM_POINTS][2];
	int obstacles[NUM_OBSTACLES][4];
	int cur_point = 0;
	int num_points = 0;

	srand(time(NULL));
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		obstacles[i][0] = rand()%1000;
		obstacles[i][1] = rand()%1000;
		obstacles[i][2] = rand()%1000;
		obstacles[i][3] = rand()%1000;
	}

	SDL_Event e;
	int x, y;
	bool running = true;
	while (running) {

		SDL_Delay(10);
		while (SDL_PollEvent(&e)) {

			switch (e.type) {
				case SDL_QUIT:
					running = false;
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&x, &y);
					break;

				case SDL_MOUSEBUTTONDOWN:
					SDL_GetMouseState(&x, &y);
					points[cur_point][0] = x;
					points[cur_point][1] = y;
					cur_point = (cur_point + 1) % NUM_POINTS;
					num_points = num_points + 1 > NUM_POINTS ? NUM_POINTS : num_points + 1;
					break;
			}
		}

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 50);
		SDL_RenderClear(renderer);

		for (int i = 0; i < NUM_CONES; i++) {

			float theta = -PI + (i/(float) NUM_CONES) * 2 * PI;
			float thetaN = -PI + ((i+1)/(float) NUM_CONES) * 2 * PI;

			// Find nearest point in cone
			int min_idx = -1;
			float min_dist = pow(2, 31);
			for (int j = 0; j < num_points; j++) {

				float alpha = atan2(y - points[j][1], x - points[j][0]);
				
				if (alpha < thetaN && alpha >= theta) {
					float distance = sqrt(pow(points[j][0] - x, 2) + pow(points[j][1] - y, 2));
					if (distance < min_dist || min_idx == -1) {
						min_dist = distance;
						min_idx = j;
					}
				}
			}
			int vx = CONE_LENGTH * cos(theta);
			int vy = CONE_LENGTH * sin(theta);
			SDL_SetRenderDrawColor(renderer, 100, 100, 100, 50);
			SDL_RenderDrawLine(renderer, x, y, x + vx, y + vy);
			if (min_idx != -1) {
				SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
				SDL_RenderDrawLine(renderer, x, y, points[min_idx][0], points[min_idx][1]);
			}

		}

		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 50);
		for (int i = 0; i < NUM_POINTS; i++) {
			SDL_RenderDrawPoint(renderer, points[i][0], points[i][1]);
		}

		SDL_SetRenderDrawColor(renderer, 0, 255, 0, 50);
		for (int i = 0; i < NUM_OBSTACLES; i++) {
			int x = obstacles[i][0];
			int y = obstacles[i][1];
			int dx = x - obstacles[i][2];
			int dy = y - obstacles[i][3];
			SDL_RenderDrawLine(renderer, x, y, dx, dy);
		}
		SDL_RenderPresent(renderer);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

