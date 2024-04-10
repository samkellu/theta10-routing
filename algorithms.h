vec2 orth_project(vec2 v, edge edge);
double get_orth_distance(vec2 v, edge edge);
double get_distance_from_edge(vec2 v, edge edge);
vec2 get_intersect(edge l1, edge l2);
bool is_visible(vec2 observer, vec2 pt, edge obstacles[], int n);
cone bisect_alg(SDL_Renderer* renderer, vec2 cur_point, cone* cones, int n_cones, cone found_cone, vec2 s, vec2 t);
cone low_angle_alg(SDL_Renderer* renderer, vec2 cur_point, cone* cones, int n_cones, cone found_cone, vec2 s, vec2 t);