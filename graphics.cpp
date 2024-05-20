#include "prog.h"
#include "graphics.h"

void draw_point(SDL_Renderer* renderer, point pos, color c) {

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

void draw_tri(SDL_Renderer* renderer, point p, canonical_triangle tri, color c) {

	double a = tri.bisect_distance;
	double o = a * tanf((tri.ar - tri.al) / 2);
	double h = sqrt(pow(a, 2) + pow(o, 2));

	// Draw triangle
	int blx = p.x + h * cosf(tri.al);
	int bly = p.y + h * sinf(tri.al);
	int brx = p.x + h * cosf(tri.ar);
	int bry = p.y + h * sinf(tri.ar);

	SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
	SDL_RenderDrawLine(renderer, blx, bly, brx, bry);
	SDL_RenderDrawLine(renderer, p.x, p.y, brx, bry);
	SDL_RenderDrawLine(renderer, p.x, p.y, blx, bly);
}

void draw_line(SDL_Renderer* renderer, point v1, point v2, color c) {
	SDL_SetRenderDrawColor(renderer, c.r, c.g, c.b, c.a);
	SDL_RenderDrawLine(renderer, v1.x, v1.y, v2.x, v2.y);
}

void draw(SDL_Renderer* renderer, point* points, int num_points, edge* obstacles, int num_obstacles, point s, point t) {
	for (int i = 2; i < num_points; i++) {
		draw_point(renderer, points[i], {255, 255, 255, 50});
	}

    for (int i = 0; i < num_points; i++) {
        for (int j = 0; j < points[i].num_neighbours; j++) {
            draw_line(renderer, points[i], *points[i].neighbours[j]->p, {100, 100, 100, 100});
        }
    }

	draw_point(renderer, s, {255, 0, 0, 255});
	draw_point(renderer, t, {0, 0, 255, 255});

	SDL_SetRenderDrawColor(renderer, 0, 255, 0, 50);
	for (int i = 0; i < NUM_OBSTACLES; i++) {
		point p0 = obstacles[i].points[0];
		point p1 = obstacles[i].points[1];
		SDL_RenderDrawLine(renderer, p0.x, p0.y, p1.x, p1.y);
	}

	SDL_SetRenderDrawColor(renderer, 255, 255, 0, 50);
	SDL_RenderDrawLine(renderer, s.x, s.y, t.x, t.y);
	SDL_RenderPresent(renderer);
}