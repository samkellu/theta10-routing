point orth_project(point v,
edge edge);
double get_orth_distance(point v, pl_edge edge);
double get_distance_from_edge(point v, pl_edge edge);
point get_intersect(pl_edge l1, pl_edge l2);
bool is_visible(point observer, point pt, pl_edge obstacles[], int n);
canonical_triangle* bisect_alg(SDL_Renderer* renderer, point cur_point, point s, point t);
canonical_triangle* cone_alg(SDL_Renderer* renderer, point cur_point, point s, point t);
canonical_triangle* low_angle_alg(SDL_Renderer* renderer, point cur_point, point s, point t);
double get_angle(int v1x, int v1y, int v2x, int v2y);

enum side {
    LEFT,
    RIGHT
};

static const double cone_bounds[] = {
    PI / 10,
    3 * PI / 10,
    5 * PI / 10,
    7 * PI / 10,
    9 * PI / 10,
    11 * PI / 10,
    13 * PI / 10,
    15 * PI / 10,
    17 * PI / 10,
    19 * PI / 10
};