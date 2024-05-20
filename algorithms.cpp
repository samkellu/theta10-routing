#include "prog.h"
#include "graphics.h"
#include "algorithms.h"

point orth_project(point v, edge edge) {
	double recip_m = -(edge.points[1].x - edge.points[0].x) / (edge.points[1].y - edge.points[0].y);
	double m = (edge.points[1].y - edge.points[0].y) / (edge.points[1].x - edge.points[0].x);

	double recip_b = v.y - recip_m * v.x;
	double b = edge.points[0].y - m * edge.points[0].x;

	double x = recip_b / (m - recip_m) - b / (m - recip_m);
	double y = m * x + b;

	return {x, y};
}

double get_orth_distance(point v, edge edge) {
	
	point projected = orth_project(v, edge);
	return sqrt(pow(edge.points[0].y - projected.y, 2) + pow(edge.points[0].x - projected.x, 2));
}

double get_distance_from_edge(point v, edge edge) {
	
	point projected = orth_project(v, edge);
	// check projected hit is on the line at all

	point high_x = edge.points[0];
	point low_x = edge.points[1];
	if (low_x.x > high_x.x)
		high_x = edge.points[1];
		low_x = edge.points[0];

	double x_max = std::max(edge.points[0].x, edge.points[1].x); 
	double x_min = std::min(edge.points[0].x, edge.points[1].x);

	if (projected.x > high_x.x) {
		return sqrt(pow(v.y - high_x.y, 2) + pow(v.x - high_x.x, 2));
	
	} else if (projected.x < x_min) {
		return sqrt(pow(v.y - low_x.y, 2) + pow(v.x - low_x.x, 2));
	}

	return sqrt(pow(v.y - projected.y, 2) + pow(v.x - projected.x, 2));
}

point get_intersect(edge l1, edge l2) {

	double x[] = {l2.points[0].x,
				  l2.points[1].x,
				  l1.points[0].x,
				  l1.points[1].x};

	double y[] = {l2.points[0].y,
				  l2.points[1].y,
				  l1.points[0].y,
				  l1.points[1].y};

	double denominator = (x[0]-x[1])*(y[2]-y[3])-(y[0]-y[1])*(x[2]-x[3]);

	// vectors do not ever intersect
	if (denominator == 0) return {-1, -1};

	double t = ((x[0]-x[2])*(y[2]-y[3])-(y[0]-y[2])*(x[2]-x[3]))/denominator;
	double u = -((x[0]-x[1])*(y[0]-y[2])-(y[0]-y[1])*(x[0]-x[2]))/denominator;

	// Vectors do not intersect
	if (t <= 0 || t >= 1 || u <= 0) return {-1, -1};

	return { x[0] + t * (x[1] - x[0]), y[0] + t * (y[1] - y[0]) };
}

bool is_visible(point observer, point pt, edge obstacles[], int n) {

	edge vis_edge = {observer, pt};
	double distance = sqrt(pow(observer.x - pt.x, 2) + pow(observer.y - pt.y, 2));

	for (int i = 0; i < n; i++) {
		point hit = get_intersect(vis_edge, obstacles[i]);
		double hit_dist = sqrt(pow(hit.y - observer.y, 2) + pow(hit.x - observer.x, 2));
		if (hit_dist <= distance) return false;
	}

	return true;
}

canonical_triangle* bisect_alg(SDL_Renderer* renderer, point cur_point, point s, point t) {

	edge e = {s, t};
	canonical_triangle* best = NULL;

	double alpha = atan2(t.x - s.x, t.y - s.y);
	double best_diff = PI;
	SDL_SetRenderDrawColor(renderer, 100, 100, 100, 100);
	double region_l = alpha - PI/2;
	double region_r = alpha + PI/2;
	double target = alpha;
	point cur_proj = orth_project(cur_point, e);

	if (cur_point.x != s.x && cur_point.y != s.y) {
		double proj_angle = atan2(s.x - cur_proj.x, s.y - cur_proj.y);
		if (proj_angle > alpha) {
			region_l = alpha;
			target = region_r - PI/2;
		} else {
			region_r = alpha;
			target = region_l + PI/2;
		}
	}

	draw_line(renderer, cur_proj, {cur_proj.x + 100*sinf(region_r), cur_proj.y + 100*cosf(region_r)}, {0,0,255,100});
	draw_line(renderer, cur_proj, {cur_proj.x + 100*sinf(region_l), cur_proj.y + 100*cosf(region_l)}, {0,0,255,100});
	printf("alph:%lf l:%lf r:%lf (%lf, %lf) (%lf,%lf)\n", alpha, region_l, region_r, cur_proj.x, cur_proj.y, cur_proj.x + 100*sinf(region_r), cur_proj.y + 100*cosf(region_r));
	fflush(stdout);
	SDL_RenderPresent(renderer);
	SDL_Delay(2000);

	// angle from projection to curpoint is vect normal of st

	// Only check edges above haflplane of st facing t.
	// Check chain on current side of st first 
	 
	// Sweep edges from wlog. left to right relative to v from halfplane st

	

	for (int i = 0; i < cur_point.num_neighbours; i++) {

		canonical_triangle* neighbour_tri = cur_point.neighbours[i];
		point* neighbour = neighbour_tri->p;
		double neighbour_angle = atan2(neighbour->x - cur_proj.x, neighbour->y - cur_proj.y);
		double neighbour_cur_angle = 0;
		if (neighbour->x != cur_point.x)
			neighbour_cur_angle = atan2(neighbour->x - cur_point.x, neighbour->y - cur_point.y);

		double diff = abs(neighbour_cur_angle - target);
		printf("%lf %lf n:%lf l:%lf r:%lf\n", diff, best_diff, neighbour_angle, region_l, region_r);
		bool is_valid = (neighbour->x == t.x && neighbour->y == t.y) || (neighbour_angle >= region_l && neighbour_angle <= region_r && diff < best_diff);
		if (!is_valid) continue;

		best_diff = diff;
		best = cur_point.neighbours[i];
	}

	return best;
}

canonical_triangle* low_angle_alg(SDL_Renderer* renderer, point cur_point, point s, point t) {

	edge e = {s, t};
	canonical_triangle* best = NULL;

	double best_angle = 360;
	SDL_SetRenderDrawColor(renderer, 100, 100, 100, 100);
	for (int i = 0; i < cur_point.num_neighbours; i++) {

		// double t_angle = atan2(t.y - cur_point.y, t.x - cur_point.x);

		// if (c.cone_left_angle + (PI/2) < found_cone_bisect_angle || c.cone_right_angle - (PI/2) > found_cone_bisect_angle)
			// continue;

		// int cx = cur_point.x + CONE_LENGTH * cos(c.cone_left_angle);
		// int cy = cur_point.y + CONE_LENGTH * sin(c.cone_left_angle);
		// SDL_RenderDrawLine(renderer, cur_point.x, cur_point.y, cx, cy);
		// int crx = cur_point.x + CONE_LENGTH * cos(c.cone_right_angle);
		// int cry = cur_point.y + CONE_LENGTH * sin(c.cone_right_angle);
		// SDL_RenderDrawLine(renderer, cur_point.x, cur_point.y, crx, cry);

		if (cur_point.neighbours[i]->p->x == t.x && cur_point.neighbours[i]->p->y == t.y) return cur_point.neighbours[i];

		point v_proj = orth_project(cur_point, e);
		point u_proj = orth_project(*cur_point.neighbours[i]->p, e);
		double o = get_distance_from_edge(*cur_point.neighbours[i]->p, e);
		double a = sqrt(pow(v_proj.y - u_proj.y, 2) + pow(v_proj.x - u_proj.x, 2));
		double alpha = atan(o/a);

		double dist_vproj_t = sqrt(pow(v_proj.y - t.y, 2) + pow(v_proj.x - t.x, 2));
		double dist_uproj_t = sqrt(pow(t.y - u_proj.y, 2) + pow(t.x - u_proj.x, 2));
		if (dist_vproj_t < dist_uproj_t) continue;

		alpha *= alpha < 0 ? -1 : 1;
		if (alpha < best_angle) {
			best_angle = alpha;
			best = cur_point.neighbours[i];
		}

		// stay on the st side of any constraint
		// ortho prject both endpoints onto each line?
		// if subcone -> chose closer of two vertices
	}

	return best;
}