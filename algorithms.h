point orth_project(point v, edge edge);
double get_orth_distance(point v, edge edge);
double get_distance_from_edge(point v, edge edge);
point get_intersect(edge l1, edge l2);
bool is_visible(point observer, point pt, edge obstacles[], int n);
canonical_triangle* bisect_alg(SDL_Renderer* renderer, point cur_point, point s, point t);
canonical_triangle* low_angle_alg(SDL_Renderer* renderer, point cur_point, point s, point t);
double get_angle(int v1x, int v1y, int v2x, int v2y);

enum side {
    LEFT,
    RIGHT
};