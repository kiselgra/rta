#ifndef __LIBRTA_H__ 
#define __LIBRTA_H__ 

#include "image.h"
#include "basic_types.h"
#include "intersect.h"
#include "raytrav.h"

namespace rta {
	struct rt_set {
		acceleration_structure *as;
		acceleration_structure_constructor *ctor;
		rta::bouncer *bouncer;
		raytracer *rt;
		ray_generator *rgen;
		rt_set() : as(0), ctor(0), bouncer(0), rt(0), rgen(0) {}

		template<typename box_t, typename tri_t> basic_raytracer<box_t, tri_t>* basic_rt() { 
			return dynamic_cast<basic_raytracer<box_t, tri_t>*>(rt);
		}
		template<typename box_t, typename tri_t> basic_acceleration_structure<box_t, tri_t>* basic_as() { 
			return dynamic_cast<basic_acceleration_structure<box_t, tri_t>*>(as);
		}
		template<typename box_t, typename tri_t> basic_acceleration_structure_constructor<box_t, tri_t>* basic_ctor() { 
			return dynamic_cast<basic_acceleration_structure_constructor<box_t, tri_t>*>(ctor);
		}
	};
}



#endif

