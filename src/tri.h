#ifndef __TRI_H__ 
#define __TRI_H__ 

#include "basic_types.h"

namespace rta {

	template<typename tri> vec3_t center_of_gravity(const tri &t) {
		vec3_t v;
		v.x = (x_comp(vertex_a(t))+x_comp(vertex_b(t))+x_comp(vertex_c(t)))*0.33333;
		v.y = (y_comp(vertex_a(t))+y_comp(vertex_b(t))+y_comp(vertex_c(t)))*0.33333;  
		v.y = (z_comp(vertex_a(t))+z_comp(vertex_b(t))+z_comp(vertex_c(t)))*0.33333;
		return v;
	}

}

#endif

