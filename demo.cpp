#include <SDL2/SDL.h>
#include <SDL2/SDL_syswm.h>

int main(int argc, char** argv) {
	
	if (SDL_Init(SDL_INIT_EVERYTHING) != 0) {
		fprintf(stderr, "error initialising SDL: %s\n", SDL_GetError());
	}
	SDL_Window* win = SDL_CreateWindow("DEMO", SDL_WINDOWPOS_CENTERED,
					   SDL_WINDOWPOS_CENTERED, 1000, 1000, 0);

	if (!win) {
		fprintf(stderr, "Failed to initialise window! Exitting...\n");
		return -1;
	}

	SDL_SysWMinfo inf;
	SDL_GetWindowWMInfo(win, &inf);
	if (inf.subsystem == SDL_SYSWM_WAYLAND) {
		printf("Is wayland\n");
	}

	SDL_Surface* surface = SDL_GetWindowSurface(win);
	SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0xFF, 0xFF, 0xFF));
	SDL_UpdateWindowSurface(win);
	SDL_Delay(10000);

	SDL_DestroyWindow(win);
	SDL_Quit();

	// while(1) {
	// 	printf("Sup mate\n");
	// }
	return 0;
}

