#include <iostream>
#include <cstring>

#include "bbvh.h"

using namespace std;
using namespace rta;

extern "C" {
	char* description() {
		return (char*)"a binary bvh ray tracer";
	}


// 	rt_set<simple_aabb, simple_triangle> make_bvh_stuff(bouncer *b, ray_generator *raygen, std::list<flat_triangle_list> triangle_lists) {
// 		typedef simple_triangle tri_t;
// 		typedef simple_aabb box_t;
// 		typedef binary_bvh<box_t, tri_t> bvh_t;
// 
// 		bbvh_constructor_using_median<bvh_t> *ctor = new bbvh_constructor_using_median<bvh_t>(bbvh_constructor_using_median<bvh_t>::spatial_median);
// 		bvh_t *bvh = ctor->build(&triangle_lists.front());
// 
// 		basic_raytracer<box_t, tri_t> *rt = 0;
// 		if (cmdline.bvh_trav == Cmdline::cis)
// 			rt = new bbvh_child_is_tracer<box_t, tri_t>(raygen, bvh, b);
// 		else
// 			rt = new bbvh_direct_is_tracer<box_t, tri_t>(raygen, bvh, b);
// 
// 		rt_set<box_t, tri_t> set;
// 		set.as = bvh;
// 		set.ctor = ctor;
// 		set.bcr = b;
// 		set.rt = rt;
// 		set.rgen = raygen;
// 
// 		return set;
// 	}

	rt_set<simple_aabb, simple_triangle> create_rt_set(std::list<flat_triangle_list> &triangle_lists) {
		typedef simple_triangle tri_t;
		typedef simple_aabb box_t;
		typedef binary_bvh<box_t, tri_t> bvh_t;

		bbvh_constructor_using_median<bvh_t> *ctor = new bbvh_constructor_using_median<bvh_t>(bbvh_constructor_using_median<bvh_t>::spatial_median);
		bvh_t *bvh = ctor->build(&triangle_lists.front());

		basic_raytracer<box_t, tri_t> *rt = 0;
// 		if (cmdline.bvh_trav == Cmdline::cis)
			rt = new bbvh_child_is_tracer<box_t, tri_t>(0, bvh, 0);
// 		else
// 			rt = new bbvh_direct_is_tracer<box_t, tri_t>(0, bvh, 0);

		rt_set<box_t, tri_t> set;
		set.as = bvh;
		set.ctor = ctor;
		set.rt = rt;
		
		return set;
	}
}

int main(int argc, char **argv)
{	

	return 0;
}


