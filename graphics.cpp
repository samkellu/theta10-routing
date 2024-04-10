#include "prog.h"
#include "graphics.h"

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

void draw(SDL_Renderer* renderer, vec2* points, int num_points, edge* obstacles, int num_obstacles, vec2 s, vec2 t) {
	for (int i = 2; i < num_points; i++) {
		draw_point(renderer, points[i], {255, 255, 255, 50});
	}

	draw_point(renderer, s, {255, 0, 0, 255});
	draw_point(renderer, t, {0, 0, 255, 255});

	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 50);
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		vec2 p0 = obstacles[i].points[0];
		vec2 p1 = obstacles[i].points[1];
		SDL_RenderDrawLine(renderer, p0.x, p0.y, p1.x, p1.y);
	}

	SDL_SetRenderDrawColor(renderer, 255, 255, 0, 50);
	SDL_RenderDrawLine(renderer, s.x, s.y, t.x, t.y);
	SDL_RenderPresent(renderer);
}