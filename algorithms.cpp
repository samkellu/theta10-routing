#include "prog.h"
#include "graphics.h"
#include "algorithms.h"


double dot(vec2 v1, vec2 v2) {
	return v1.x * v2.x + v1.y * v2.y;
}

double cross(vec2 v1, vec2 v2) {
	return v1.x * v2.y - v1.y * v2.x;
}

double get_angle(vec2 v1, vec2 v2)
{
	// |A·B| = |A| |B| COS(θ)
	// |A×B| = |A| |B| SIN(θ)
	return atan2(cross(v1, v2), dot(v1, v2));
}

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

canonical_triangle* find_bisect(SDL_Renderer* renderer, point cur_point, point cur_proj, point s, point t, double sweep_target, double region_l, double region_r, side lr) {

	canonical_triangle* best = NULL;
	double sweep_limit = lr == RIGHT ? PI : -PI;
	vec2 cur_to_proj = {cur_proj.x - cur_point.x, cur_proj.y - cur_point.y};

	if (cur_point.obstacle_endpoint != NULL) {
		vec2 cur_to_endpoint = {cur_point.obstacle_endpoint->x - cur_point.x, cur_point.obstacle_endpoint->y - cur_point.y};
		double endpoint_angle = get_angle(cur_to_proj, cur_to_endpoint);
		if ((lr == LEFT && endpoint_angle > sweep_limit && endpoint_angle < 0) || (lr == RIGHT && endpoint_angle < sweep_limit && endpoint_angle > 0))
			sweep_limit = endpoint_angle;
	}
	
	double best_neighbour_angle = sweep_limit;

#ifdef DRAW_BOUNDS
	double st_ang = atan2(t.y - s.y, t.x - s.x);
	double cur_ang = atan2(cur_proj.y - cur_point.y, cur_proj.x - cur_point.x);
	draw_line(renderer, cur_point, cur_proj, {0,0,255,100});
	draw_line(renderer, cur_point, {cur_point.x + 100*cosf(sweep_limit + cur_ang), cur_point.y + 100*sinf(sweep_limit + cur_ang)}, {0,0,255,100});
	draw_line(renderer, cur_proj, {cur_proj.x + 100*cosf(st_ang), cur_proj.y + 100*sinf(st_ang)}, {0,0,255,100});
	// draw_line(renderer, cur_proj, {cur_proj.x + 100*cosf(region_l + st_ang), cur_proj.y + 100*sinf(region_l + st_ang)}, {255,255,255,100});
	SDL_RenderPresent(renderer);
	SDL_Delay(1500);
#endif


	for (int i = 0; i < cur_point.num_neighbours; i++) {
		canonical_triangle* neighbour_tri = cur_point.neighbours[i];
		point* neighbour = neighbour_tri->p;

		if (neighbour->x == cur_point.x && neighbour->y == cur_point.y) continue; 
		if (neighbour->x == t.x && neighbour->y == t.y)
			return cur_point.neighbours[i];

		vec2 s_to_t = {t.x - s.x, t.y - s.y};
		vec2 proj_to_neighbour = {neighbour->x - cur_proj.x, neighbour->y - cur_proj.y};
		vec2 cur_to_neighbour = {neighbour->x - cur_point.x, neighbour->y - cur_point.y};
		double neighbour_angle = get_angle(s_to_t, proj_to_neighbour);
		double neighbour_cur_angle = get_angle(cur_to_proj, cur_to_neighbour);

		if (neighbour_angle <= region_l || neighbour_angle >= region_r) continue;

		if (lr == RIGHT) {
			if (neighbour_cur_angle > sweep_limit || neighbour_cur_angle < 0 || neighbour_cur_angle > best_neighbour_angle) continue;

		} else {
			if (neighbour_cur_angle < sweep_limit || neighbour_cur_angle > 0 || neighbour_cur_angle < best_neighbour_angle) continue;
		}
		
		printf("%s: lim %lf  val %lf best %lf)\n", lr == RIGHT ? "RIGHT" : "LEFT", sweep_limit, neighbour_cur_angle, best_neighbour_angle);
		best_neighbour_angle = neighbour_cur_angle;
		best = cur_point.neighbours[i];
	}

#ifdef DRAW_BOUNDS
	draw_line(renderer, cur_point, {cur_point.x + 100*cosf(best_neighbour_angle + cur_ang), cur_point.y + 100*sinf(best_neighbour_angle + cur_ang)}, {255,255,255,100});
	SDL_RenderPresent(renderer);
	SDL_Delay(2000);

	draw_line(renderer, cur_point, {cur_point.x + 100*cosf(best_neighbour_angle + cur_ang), cur_point.y + 100*sinf(best_neighbour_angle + cur_ang)}, {0,0,0,100});
	SDL_RenderPresent(renderer);
#endif

	return best;
}

canonical_triangle* closest_to_st(point cur_point, edge st, bool use_side = false, double al = -PI, double ar = PI, point* cur_proj = NULL) {

	double best_dist = pow(2, 31);
	canonical_triangle* best_neighbour = NULL;
	for (int i = 0; i < cur_point.num_neighbours; i++) {
		canonical_triangle* neighbour = cur_point.neighbours[i];

		if (use_side) {
			point s = st.points[0];
			point t = st.points[1];

			vec2 proj_to_t = {t.x - cur_proj->x, t.y - cur_proj->y};
			vec2 proj_to_neighbour = { neighbour->p->x - cur_proj->x, neighbour->p->y - cur_proj->y};
			double cur_angle = get_angle(proj_to_t, proj_to_neighbour);
			if (cur_angle < al || cur_angle > ar) continue;
		}

		double dist = get_distance_from_edge(*neighbour->p, st);
		if (dist < best_dist) {
			best_dist = dist;
			best_neighbour = neighbour;
		}
	}

	return best_neighbour;
}

canonical_triangle* bisect_alg(SDL_Renderer* renderer, point cur_point, point s, point t) {

	edge e = {s, t};
	if (cur_point.x == s.x && cur_point.y == s.y)
		return closest_to_st(cur_point, e);

	canonical_triangle* best = NULL;
	double region_l = -PI/2; 
	double region_r = PI/2;
	point cur_proj = orth_project(cur_point, e);
	double target;
	side lr;

	vec2 s_to_t = {t.x - s.x, t.y - s.y};
	vec2 s_to_cur = {cur_point.x - s.x, cur_point.y - s.y};
	double cur_angle = get_angle(s_to_t, s_to_cur);
	if (cur_angle > 0) {
		lr = RIGHT;
		region_l = 0;
		target = -region_r;
	
	} else {
		lr = LEFT;
		region_r = 0;
		target = -region_l;
	}

	best = find_bisect(renderer, cur_point, cur_proj, s, t, target, region_l, region_r, lr);

	// Cross st if no path exists to point closest to st
	if (best == NULL) {
		if (lr == LEFT)
			best = closest_to_st(cur_point, e, true, 0, -region_l, &cur_proj);
		else
			best = closest_to_st(cur_point, e, true, -region_r, 0, &cur_proj);
	}

	return best;
}

vec2 find_t_cone(point cur_point, point t) {

	double alpha = atan2(t.y - cur_point.y, t.x - cur_point.x);

	for (int i = 0; i <= NUM_CONES; i++) {
		printf("%lf <= %lf < %lf\n", cone_bounds[i], alpha, cone_bounds[(i+1) % NUM_CONES]);
		if (alpha >= cone_bounds[i] && alpha < cone_bounds[(i+1) % NUM_CONES]) {
			return {(double) cone_bounds[i], (double) cone_bounds[(i+1) % NUM_CONES]};
		}
	}

	return {0, 0};
}

canonical_triangle* find_cone(SDL_Renderer* renderer, point cur_point, point cur_proj, point s, point t, double sweep_target, double region_l, double region_r, side lr) {

	canonical_triangle* best = NULL;
	double sweep_limit = lr == RIGHT ? PI : -PI;
	vec2 cur_to_proj = {cur_proj.x - cur_point.x, cur_proj.y - cur_point.y};

	if (cur_point.obstacle_endpoint != NULL) {
		vec2 cur_to_endpoint = {cur_point.obstacle_endpoint->x - cur_point.x, cur_point.obstacle_endpoint->y - cur_point.y};
		double endpoint_angle = get_angle(cur_to_proj, cur_to_endpoint);
		if ((lr == LEFT && endpoint_angle > sweep_limit && endpoint_angle < 0) || (lr == RIGHT && endpoint_angle < sweep_limit && endpoint_angle > 0))
			sweep_limit = endpoint_angle;
	}
	
	double best_neighbour_angle = sweep_limit;

#ifdef DRAW_BOUNDS
	double st_ang = atan2(t.y - s.y, t.x - s.x);
	double cur_ang = atan2(cur_proj.y - cur_point.y, cur_proj.x - cur_point.x);
	draw_line(renderer, cur_point, cur_proj, {0,0,255,100});
	draw_line(renderer, cur_point, {cur_point.x + 100*cosf(sweep_limit + cur_ang), cur_point.y + 100*sinf(sweep_limit + cur_ang)}, {0,0,255,100});
	draw_line(renderer, cur_proj, {cur_proj.x + 100*cosf(st_ang), cur_proj.y + 100*sinf(st_ang)}, {0,0,255,100});
	// draw_line(renderer, cur_proj, {cur_proj.x + 100*cosf(region_l + st_ang), cur_proj.y + 100*sinf(region_l + st_ang)}, {255,255,255,100});
	SDL_RenderPresent(renderer);
	SDL_Delay(1500);
#endif


	vec2 t_cone_bounds = find_t_cone(cur_point, t);
	for (int i = 0; i < cur_point.num_neighbours; i++) {
		canonical_triangle* neighbour_tri = cur_point.neighbours[i];
		point* neighbour = neighbour_tri->p;

		if (neighbour->x == cur_point.x && neighbour->y == cur_point.y) continue; 
		if (neighbour->x == t.x && neighbour->y == t.y)
			return cur_point.neighbours[i];

		vec2 s_to_t = {t.x - s.x, t.y - s.y};
		vec2 proj_to_neighbour = {neighbour->x - cur_proj.x, neighbour->y - cur_proj.y};
		vec2 cur_to_neighbour = {neighbour->x - cur_point.x, neighbour->y - cur_point.y};
		double neighbour_angle = get_angle(s_to_t, proj_to_neighbour);
		double neighbour_cur_angle = get_angle(cur_to_proj, cur_to_neighbour);

		double alpha = atan2(neighbour->y - cur_point.y, neighbour->x - cur_point.x);
		bool in_t_cone = alpha >= t_cone_bounds.x && alpha < t_cone_bounds.y;

		printf("%lf <= %lf < %lf in t cone %s\n", t_cone_bounds.x, alpha, t_cone_bounds.y, in_t_cone ? "yes" : "no");

		if (!in_t_cone && (neighbour_angle <= region_l || neighbour_angle >= region_r)) continue;

		if (lr == RIGHT) {
			if (neighbour_cur_angle > sweep_limit || neighbour_cur_angle < 0 || neighbour_cur_angle > best_neighbour_angle) continue;
			
		} else {
			if (neighbour_cur_angle < sweep_limit || neighbour_cur_angle > 0 || neighbour_cur_angle < best_neighbour_angle) continue;
		}
		
		printf("%s: lim %lf  val %lf best %lf)\n", lr == RIGHT ? "RIGHT" : "LEFT", sweep_limit, neighbour_cur_angle, best_neighbour_angle);
		best_neighbour_angle = neighbour_cur_angle;
		best = cur_point.neighbours[i];
	}

#ifdef DRAW_BOUNDS
	draw_line(renderer, cur_point, {cur_point.x + 100*cosf(best_neighbour_angle + cur_ang), cur_point.y + 100*sinf(best_neighbour_angle + cur_ang)}, {255,255,255,100});
	SDL_RenderPresent(renderer);
	SDL_Delay(2000);

	draw_line(renderer, cur_point, {cur_point.x + 100*cosf(best_neighbour_angle + cur_ang), cur_point.y + 100*sinf(best_neighbour_angle + cur_ang)}, {0,0,0,100});
	SDL_RenderPresent(renderer);
#endif

	return best;
}

canonical_triangle* cone_alg(SDL_Renderer* renderer, point cur_point, point s, point t) {

	edge e = {s, t};
	if (cur_point.x == s.x && cur_point.y == s.y)
		return closest_to_st(cur_point, e);

	canonical_triangle* best = NULL;
	double region_l = -PI/2; 
	double region_r = PI/2;
	point cur_proj = orth_project(cur_point, e);
	double target;
	side lr;

	vec2 s_to_t = {t.x - s.x, t.y - s.y};
	vec2 s_to_cur = {cur_point.x - s.x, cur_point.y - s.y};
	double cur_angle = get_angle(s_to_t, s_to_cur);
	if (cur_angle > 0) {
		lr = RIGHT;
		region_l = 0;
		target = -region_r;
	
	} else {
		lr = LEFT;
		region_r = 0;
		target = -region_l;
	}

	best = find_cone(renderer, cur_point, cur_proj, s, t, target, region_l, region_r, lr);

	// Cross st if no path exists to point closest to st
	// if (best == NULL) {
	// 	if (lr == LEFT)
	// 		best = closest_to_st(cur_point, e, true, 0, -region_l, &cur_proj);
	// 	else
	// 		best = closest_to_st(cur_point, e, true, -region_r, 0, &cur_proj);
	// }

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