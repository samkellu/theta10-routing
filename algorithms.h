point orth_project(point v, edge edge);
double get_orth_distance(point v, edge edge);
double get_distance_from_edge(point v, edge edge);
point get_intersect(edge l1, edge l2);
bool is_visible(point observer, point pt, edge obstacles[], int n);
cone bisect_alg(SDL_Renderer* renderer, point cur_point, cone* cones, int n_cones, cone found_cone, point s, point t);
point* low_angle_alg(SDL_Renderer* renderer, point cur_point, point s, point t);