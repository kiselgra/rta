#ifndef __TRI_H__ 
#define __TRI_H__ 

#include "basic_types.h"

namespace rta {

	template<typename tri> typename tri::vec3_t center_of_gravity(const tri &t) {
		typename tri::vec3_t v;
		v.x = (x_comp(vertex_a(t))+x_comp(vertex_b(t))+x_comp(vertex_c(t)))*0.33333;
		v.y = (y_comp(vertex_a(t))+y_comp(vertex_b(t))+y_comp(vertex_c(t)))*0.33333;  
		v.z = (z_comp(vertex_a(t))+z_comp(vertex_b(t))+z_comp(vertex_c(t)))*0.33333;
		return v;
	}

	/*
	template<typename tri> vec3_t center_of_gravity_d(const tri &t) {
		vec3_t v;
		std::cout << "        " << x_comp(vertex_a(t)) << " + " << x_comp(vertex_b(t)) << " + " << x_comp(vertex_c(t)) << " * 1/3  \t = " << (x_comp(vertex_a(t))+x_comp(vertex_b(t))+x_comp(vertex_c(t))) << " * 1/3 \t = " << (x_comp(vertex_a(t))+x_comp(vertex_b(t))+x_comp(vertex_c(t)))*0.33333<< std::endl;
		std::cout << "        " << y_comp(vertex_a(t)) << " + " << y_comp(vertex_b(t)) << " + " << y_comp(vertex_c(t)) << " * 1/3  \t = " << (y_comp(vertex_a(t))+y_comp(vertex_b(t))+y_comp(vertex_c(t))) << " * 1/3 \t = " << (y_comp(vertex_a(t))+y_comp(vertex_b(t))+y_comp(vertex_c(t)))*0.33333<< std::endl;
		std::cout << "        " << z_comp(vertex_a(t)) << " + " << z_comp(vertex_b(t)) << " + " << z_comp(vertex_c(t)) << " * 1/3  \t = " << (z_comp(vertex_a(t))+z_comp(vertex_b(t))+z_comp(vertex_c(t))) << " * 1/3 \t = " << (z_comp(vertex_a(t))+z_comp(vertex_b(t))+z_comp(vertex_c(t)))*0.33333<< std::endl;
		
		
		v.x = (x_comp(vertex_a(t))+x_comp(vertex_b(t))+x_comp(vertex_c(t)))*0.33333;
		v.y = (y_comp(vertex_a(t))+y_comp(vertex_b(t))+y_comp(vertex_c(t)))*0.33333;  
		v.z = (z_comp(vertex_a(t))+z_comp(vertex_b(t))+z_comp(vertex_c(t)))*0.33333;
		std::cout << "        --> " << v << std::endl;
		return v;
	}
	*/

	//! to and dir may point to the same location \attention modifies \c dir
	template<typename vec3_t> inline void reflect(vec3_t *to, vec3_t *normal, vec3_t *dir) {
		// to = dir - 2(dir|normal)normal
		normalize_vec3f(dir);
		float_t dot = dot_vec3f(dir, normal);
		mul_vec3f_by_scalar(to, normal, 2*dot);
		sub_components_vec3f(to, dir, to);
	}
	
	template<typename vec3_t> inline void barycentric_interpolation(vec3_t *to, const vec3_t *b_coord, const vec3_t *a, const vec3_t *b, const vec3_t *c) {
		vec3_t tmp;
		mul_vec3f_by_scalar(to,   a,  x_comp(*b_coord));
		mul_vec3f_by_scalar(&tmp, b,  y_comp(*b_coord));
		add_components_vec3f(to , to, &tmp);
		mul_vec3f_by_scalar(&tmp, c,  z_comp(*b_coord));
		add_components_vec3f(to,  to, &tmp);
	}

	template<typename vec2_t, typename vec3_t> inline void barycentric_interpolation(vec2_t *to, const vec3_t *b_coord, const vec2_t *a, const vec2_t *b, const vec2_t *c) {
		vec2_t tmp;
		mul_vec2f_by_scalar(to,   a,  x_comp(*b_coord));
		mul_vec2f_by_scalar(&tmp, b,  y_comp(*b_coord));
		add_components_vec2f(to , to, &tmp);
		mul_vec2f_by_scalar(&tmp, c,  z_comp(*b_coord));
		add_components_vec2f(to,  to, &tmp);
	}

// 	void interpolate_normal(vec3_t *to, const triangle_intersection &is) {
// 		vec3_t bcoord = { is.beta, is.gamma, (float_t)(1.0 - is.beta - is.gamma) };
// 		vec3_t interpol = 
// 		administrative_arith_t n = barycentric_interpolation(administrative_arith_t(alpha, beta, gamma), state.triangle_ref->normal_a(), 
// 				state.triangle_ref->normal_b(), state.triangle_ref->normal_c());
// 		normalize(n);
// 	}
}

#endif

