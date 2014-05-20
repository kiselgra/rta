#include "cmdline.h"
#include "bruteforce.h"

//// plugin interface

extern "C" {
	char* description() {
		return (char*)"a brute force ray caster";
	}

	void initialize() {
		rta::cuda::plugin_with_cuda_loaded = true;
	}

	int parse_cmdline(int argc, char **argv) {
		my_parse_cmdline(argc, argv);
	}

	rt_set create_rt_set(basic_flat_triangle_list<simple_triangle> &triangle_lists, int w, int h) {
		using namespace rta::cuda::example;
		using namespace rta::cuda;

		typedef rta::cuda::simple_triangle tri_t;
		typedef rta::cuda::simple_aabb box_t;

		rt_set set;
		bruteforce_dummy_as_ctor<box_t, tri_t> *ctor = new bruteforce_dummy_as_ctor<box_t, tri_t>;
		set.ctor = ctor;
		cuda_ftl ftl(triangle_lists);
		set.as = ctor->build(&ftl);
		set.rt = new bruteforce_tracer<box_t, tri_t>(0, 0, dynamic_cast<bruteforce_dummy_accel_struct<box_t, tri_t>*>(set.as));
		set.rgen = new raygen_with_buffer<cam_ray_generator_shirley>(w, h);
		return set;
	}
}



/* vim: set foldmethod=marker: */

