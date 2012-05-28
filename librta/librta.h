#ifndef __LIBRTA_H__ 
#define __LIBRTA_H__ 

#include "image.h"
#include "basic_types.h"
#include "intersect.h"
#include "raytrav.h"

namespace rta {
	template<box_t__and__tri_t> struct rt_set {
		declare_traits_types;
		acceleration_structure<forward_traits> *as;
		acceleration_structure_constructor<forward_traits> *ctor;
		rta::bouncer *bouncer;
		basic_raytracer<forward_traits> *rt;
		ray_generator *rgen;
		rt_set() : as(0), ctor(0), bouncer(0), rt(0), rgen(0) {}
	};
}



#endif

