void draw_point(SDL_Renderer* renderer, point pos, color c);
void draw_tri(SDL_Renderer* renderer, point p, canonical_triangle tri, color c);
void draw_line(SDL_Renderer* renderer, point v1, point v2, color c);
void draw(SDL_Renderer* renderer, point* points, int num_points, pl_edge* obstacles, int num_obstacles, point s, point t);