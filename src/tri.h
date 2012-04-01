#ifndef __TRI_H__ 
#define __TRI_H__ 

#include "basic_types.h"

namespace rta {

	template<typename tri> vec3_t center_of_gravity(const tri &t) {
		vec3_t v;
		v.x = (x_comp(vertex_a(t))+x_comp(vertex_b(t))+x_comp(vertex_c(t)))*0.33333;
		v.y = (y_comp(vertex_a(t))+y_comp(vertex_b(t))+y_comp(vertex_c(t)))*0.33333;  
		v.z = (z_comp(vertex_a(t))+z_comp(vertex_b(t))+z_comp(vertex_c(t)))*0.33333;
		return v;
	}

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

}

#endif

