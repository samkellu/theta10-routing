#include <SDL2/SDL.h>

#define CONE_LENGTH 200
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

	SDL_Event e;
	int x, y;
	bool running = true;
	while (running) {

		SDL_Delay(1000);
		while (SDL_PollEvent(&e)) {

			switch (e.type) {
				case SDL_QUIT:
					running = false;
					break;

				case SDL_MOUSEMOTION:
					SDL_GetMouseState(&x, &y);
					break;
			}
		}

		printf("Mx %d My %d\n", x, y);
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
		for (int i = 0; i < 10; i++) {

			float theta = (i/(float) 10) * 2 * PI; 
			int vx = CONE_LENGTH * cos(theta);
			int vy = CONE_LENGTH * sin(theta);
			SDL_RenderDrawLine(renderer, x, y, x + vx, y + vy);
		}
		SDL_RenderPresent(renderer);
	}

    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}

