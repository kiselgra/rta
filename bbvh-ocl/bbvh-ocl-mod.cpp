#include <iostream>
#include <cstring>
#include <sstream>
#include <argp.h>

#include "bbvh-ocl.h"

using namespace std;
using namespace rta;


// 
// cmdline
//

struct Cmdline {
	enum bvh_trav_t { bbvh_cis, bbvh_dis };
	bvh_trav_t bvh_trav;
	bool verbose;

	Cmdline() : bvh_trav(bbvh_cis), verbose(false) {}
};
static Cmdline cmdline;

const char *argp_program_version = VERSION;

static char plugin_name[] = "bbvh-ocl";
static char doc[]       = "bbvh-ocl: a rta plugin providing gpu ray tracing of bbvhs using open cl.";
static char args_doc[]  = "";

enum { FIRST = 300, BT, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,          0, "Be verbose." },
	{ "bvh-trav", BT, "mode",  0, "Intersection mode of the bvh traversal: \n"
													"    dis: bbvh-style traversal using direct-is (i.e. not stackless!).\n"
													"    cis: bbvh-style traversal using child-is (i.e. not stackless!).\n"
													"    Default: cis." },
	{ 0 }
};

string& replace_nl(string &s)
{
	for (int i = 0; i < s.length(); ++i)
		if (s[i] == '\n' || s[i] == '\r')
			s[i] = ' ';
	return s;
}

vec3f read_vec3f(const std::string &s) {
	istringstream iss(s);
	vec3f v;
	char sep;
	iss >> v.x >> sep >> v.y >> sep >> v.z;
	return v;
}

static error_t parse_options(int key, char *arg, argp_state *state)
{
	// call argp_usage to stop program execution if something is wrong
	string sarg;
	if (arg)
		sarg = arg;
	sarg = replace_nl(sarg);

	switch (key)
	{
	case 'v':     cmdline.verbose = true; 	break;
	case BT:      if (sarg == "dis")      cmdline.bvh_trav = Cmdline::bbvh_dis;
	              else if (sarg == "cis") cmdline.bvh_trav = Cmdline::bbvh_cis;
				  else {
					  cerr << "Unknown bvh traversal scheme: " << sarg << endl;
					  argp_usage(state);
				  }
				  break;


	case ARGP_KEY_ARG:		// process arguments. 
							// state->arg_num gives number of current arg
				  return ARGP_ERR_UNKNOWN;

	default:
		return ARGP_ERR_UNKNOWN;
	}


	return 0;
}


static struct argp parser = { options, parse_options, args_doc, doc };


// 
// dlopen interface
//

cl::context *ctx;

extern "C" {
	char* description() {
		return (char*)"a binary bvh ray tracer";
	}

	void initialize() {
		rta::ocl::context = new cl::context;
	}

	int parse_cmdline(int argc, char **argv)
	{
		argv[0] = plugin_name;
		int ret = argp_parse(&parser, argc, argv, 0/*ARGP_NO_EXIT*//*0*/, 0, 0);
		return ret;
	}

	rt_set<simple_aabb, simple_triangle> create_rt_set(std::list<flat_triangle_list> &triangle_lists, int w, int h) {
		cout << "setting up open cl" << endl;
		cl::verbose = true;
		ctx = rta::ocl::context;

		cout << "creating set" << endl;
		typedef simple_triangle tri_t;
		typedef simple_aabb box_t;
		typedef ocl::binary_bvh<box_t, tri_t> obvh_t;
		typedef bbvh_constructor_using_median<obvh_t> std_bbvh_ctor_t;
		typedef ocl::bbvh_constructor<std_bbvh_ctor_t> obvh_ctor_t;
		cout << "building bvh" << endl;
		acceleration_structure_constructor<box_t, tri_t> *ctor = 0;
		acceleration_structure<box_t, tri_t> *bvh = 0;

// 		ctor = new bbvh_ctor_t(bbvh_ctor_t::spatial_median);
// 		bvh = ctor->build(&triangle_lists.front());

		obvh_ctor_t *ocl_ctor = new obvh_ctor_t(*ctx, std_bbvh_ctor_t::spatial_median);
		bvh = ocl_ctor->build(&triangle_lists.front());

		cout << "create rt" << endl;
		basic_raytracer<box_t, tri_t> *rt = 0;
// 		switch (cmdline.bvh_trav) {
// 			case Cmdline::bbvh_cis:
// 				rt = new bbvh_child_is_tracer<box_t, tri_t, bvh_t>(0, dynamic_cast<bvh_t*>(bvh), 0);
// 				break;
// 			case Cmdline::bbvh_dis:
				rt = new ocl::bbvh_direct_is_tracer<box_t, tri_t, obvh_t>(0, dynamic_cast<obvh_t*>(bvh), 0, *ctx);
// 				break;
// 			default:
// 				cerr << "unhandled case in bbvh-ocl trav switch! (" << cmdline.bvh_trav << ")" << endl;
// 				exit(EXIT_FAILURE);
// 		}

		rt_set<box_t, tri_t> set;
		set.as = bvh;
		set.ctor = ocl_ctor;
		set.rt = rt;
		set.rgen = new ocl::cam_ray_buffer_generator_shirley(w, h, *ctx);

		cout << "OK" << endl;

		return set;

// 		bvh_t *bvh = ctor->build(&triangle_lists.front());
// 
// 		basic_raytracer<box_t, tri_t> *rt = 0;
// 		if (cmdline.bvh_trav == Cmdline::cis)
// 			rt = new bbvh_child_is_tracer<box_t, tri_t>(0, bvh, 0);
// 		else
// 			rt = new bbvh_direct_is_tracer<box_t, tri_t>(0, bvh, 0);
// 
// 		rt_set<box_t, tri_t> set;
// 		set.as = bvh;
// 		set.ctor = ctor;
// 		set.rt = rt;
// 		
// 		return set;
	}
}

/* vim: set foldmethod=marker: */

