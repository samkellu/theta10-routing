void draw_point(SDL_Renderer* renderer, vec2 pos, color c);
void draw_tri(SDL_Renderer* renderer, vec2 v, color c, double theta, double thetaN, double a);
void draw_line(SDL_Renderer* renderer, vec2 v1, vec2 v2, color c);
void draw(SDL_Renderer* renderer, vec2* points, int num_points, edge* obstacles, int num_obstacles, vec2 s, vec2 t);