#ifndef __RTA_INTERSECT_H__ 
#define __RTA_INTERSECT_H__ 

#include "basic_types.h"

namespace rta {

	template<typename tri> heterogenous bool intersect_tri_opt(const tri &t, const vec3_t *ray_origin, const vec3_t *ray_dir, triangle_intersection<tri> &info)
	{
		float_t a_x = x_comp(vertex_a(t));
		float_t a_y = y_comp(vertex_a(t));
		float_t a_z = z_comp(vertex_a(t));
		
		const float_t &a = a_x - x_comp(vertex_b(t));
		const float_t &b = a_y - y_comp(vertex_b(t));
		const float_t &c = a_z - z_comp(vertex_b(t));
		const float_t &d = a_x - x_comp(vertex_c(t));
		const float_t &e = a_y - y_comp(vertex_c(t));
		const float_t &f = a_z - z_comp(vertex_c(t));
		const float_t &g = x_comp(*ray_dir);
		const float_t &h = y_comp(*ray_dir);
		const float_t &i = z_comp(*ray_dir);
		const float_t &j = a_x - x_comp(*ray_origin);
		const float_t &k = a_y - y_comp(*ray_origin);
		const float_t &l = a_z - z_comp(*ray_origin);

		float_t common1 = e*i - h*f;
		float_t common2 = g*f - d*i;
		float_t common3 = d*h - e*g;
		float_t M 	= a * common1  +  b * common2  +  c * common3;
		float_t beta 	= j * common1  +  k * common2  +  l * common3;

		common1 = a*k - j*b;
		common2 = j*c - a*l;
		common3 = b*l - k*c;
		float_t gamma = i * common1  +  h * common2  +  g * common3;
		float_t tt	= -(f * common1  +  e * common2  +  d * common3);
			
		beta /= M;
		gamma /= M;
		tt /= M;
		
		if (tt > 0)
			if (beta > 0 && gamma > 0 && beta + gamma <= 1)
			{
				info.t = tt;
				info.beta = beta;
				info.gamma = gamma;
				return true;
			}
		
		return false;
	}	

	
	template<typename aabb> heterogenous bool intersect_aabb(const aabb &box, const vec3f *ray_origin, const vec3f *ray_dir, float &is)
	{
		float_t t_near = -FLT_MAX;
		float_t t_far  =  FLT_MAX;

		{
		const float_t d_x = x_comp(*ray_dir);
		const float_t e_x = x_comp(*ray_origin);
		const float_t x_max = x_comp(max(box));
		const float_t x_min = x_comp(min(box));
		
		if (d_x == 0)
		{
			if (e_x < x_min || e_x > x_max)
				return false;
		}
		else
		{
			float_t t1 = (x_min - e_x) / d_x;
			float_t t2 = (x_max - e_x) / d_x;

			if (t1 > t2)	{	float_t tmp = t1;	t1 = t2; t2 = tmp; 	}

			if (t1 > t_near)	t_near = t1;
			if (t2 < t_far)		t_far = t2;

			if (t_near > t_far)	// box missed
				return false;

			if (t_far < 0)		// box behind ray
				return false;
		}
		}
		{
		
		const float_t d_y = y_comp(*ray_dir);
		const float_t e_y = y_comp(*ray_origin);
		const float_t y_max = y_comp(max(box));
		const float_t y_min = y_comp(min(box));

		if (d_y == 0)
		{
			if (e_y < y_min || e_y > y_max)
				return false;
		}
		else
		{
			float_t t1 = (y_min - e_y) / d_y;
			float_t t2 = (y_max - e_y) / d_y;

			if (t1 > t2)	{	float_t tmp = t1;	t1 = t2; t2 = tmp; 	}

			if (t1 > t_near)	t_near = t1;
			if (t2 < t_far)		t_far = t2;

			if (t_near > t_far)	// box missed
				return false;

			if (t_far < 0)		// box behind ray
				return false;
		}
		}
		{

		const float_t d_z = z_comp(*ray_dir);
		const float_t e_z = z_comp(*ray_origin);
		const float_t z_max = z_comp(max(box));
		const float_t z_min = z_comp(min(box));

		if (d_z == 0)
		{
			if (e_z < z_min || e_z > z_max)
				return false;
		}
		else
		{
			float_t t1 = (z_min - e_z) / d_z;
			float_t t2 = (z_max - e_z) / d_z;

			if (t1 > t2)	{	float_t tmp = t1;	t1 = t2; t2 = tmp; }

			if (t1 > t_near)	t_near = t1;
			if (t2 < t_far)		t_far = t2;

			if (t_near > t_far)	// box missed
				return false;

			if (t_far < 0)		// box behind ray
				return false;
		}
		}

		is = t_near;
		return true;
	}


}

#endif

