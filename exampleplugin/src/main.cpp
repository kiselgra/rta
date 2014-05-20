#include "cmdline.h"
#include "bruteforce.h"


//// plugin interface

extern "C" {
	char* description() {
		return (char*)"a brute force ray caster";
	}

	void initialize() {}

	int parse_cmdline(int argc, char **argv) {
		my_parse_cmdline(argc, argv);
	}

	rt_set create_rt_set(basic_flat_triangle_list<simple_triangle> &triangle_lists, int w, int h) {
		using namespace rta::example;

		typedef simple_triangle tri_t;
		typedef simple_aabb box_t;

		rt_set set;
		bruteforce_dummy_as_ctor<box_t, tri_t> *ctor = new bruteforce_dummy_as_ctor<box_t, tri_t>;
		set.ctor = ctor;
		set.as = ctor->build(&triangle_lists);	// here we just use the first triangle list; this is actually stupid.
		set.rt = new bruteforce_tracer<box_t, tri_t>(0, 0, dynamic_cast<bruteforce_dummy_accel_struct<box_t, tri_t>*>(set.as));
		
		return set;
	}
}



/* vim: set foldmethod=marker: */

