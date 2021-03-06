#include <iostream>
#include <cstring>
#include <sstream>
#include <argp.h>

#include "sbvh.h"

using namespace std;
using namespace rta;


// 
// cmdline
//

struct Cmdline {
	enum bvh_trav_t { bbvh_cis, bbvh_dis, preorder_sbvh, sbvh_oi };
	bvh_trav_t bvh_trav;
	bool verbose;

	Cmdline() : bvh_trav(bbvh_cis), verbose(false) {}
};
static Cmdline cmdline;

const char *argp_program_version = VERSION;

static char plugin_name[] = "sbvh";
static char doc[]       = "sbvh: a rta plugin.";
static char args_doc[]  = "";

enum { FIRST = 300, BT, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,          0, "Be verbose." },
	{ "bvh-trav", BT, "mode",  0, "Intersection mode of the bvh traversal: \n"
		                                            "    sbvh: order independent sbvh traversal.\n"
													"    po-sbvh: pre order stackess bvh traversal: stackless with a fixed traversal order. see --randomize.\n"
													"    bbvh-dis: bbvh-style traversal using direct-is (i.e. not stackless!).\n"
													"    bbvh-cis: bbvh-style traversal using child-is (i.e. not stackless!).\n"
													"    Default: sbvh." },
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
	case 'v':	cmdline.verbose = true; 	break;
	case BT:      if (sarg == "bbvh-dis")      cmdline.bvh_trav = Cmdline::bbvh_dis;
	              else if (sarg == "bbvh-cis") cmdline.bvh_trav = Cmdline::bbvh_cis;
	              else if (sarg == "po-sbvh")  cmdline.bvh_trav = Cmdline::preorder_sbvh;
	              else if (sarg == "sbvh")  cmdline.bvh_trav = Cmdline::sbvh_oi;
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


extern "C" {
	char* description() {
		return (char*)"a binary bvh ray tracer";
	}

	void initialize() {}

	int parse_cmdline(int argc, char **argv)
	{
		argv[0] = plugin_name;
		int ret = argp_parse(&parser, argc, argv, 0/*ARGP_NO_EXIT*//*0*/, 0, 0);
		return ret;
	}

	rt_set create_rt_set(basic_flat_triangle_list<simple_triangle> &triangle_lists, int w, int h) {
		cout << "creating set" << endl;
		typedef simple_triangle tri_t;
		typedef simple_aabb box_t;
		typedef binary_bvh<box_t, tri_t> bvh_t;
		typedef binary_bvh_with_split_axis<box_t, tri_t> bvh_with_sa_t;
		typedef stackless_bvh<box_t, tri_t> sbvh_t;
		typedef order_independent_sbvh<box_t, tri_t> oi_sbvh_t;
		typedef preorder_stackless_bvh<box_t, tri_t> preorder_sbvh_t;
		typedef bbvh_constructor_using_median<bvh_t> bbvh_ctor_t;
		typedef bbvh_constructor_using_median<bvh_with_sa_t> bbvh_ctor_with_sa_t;
		cout << "building bvh" << endl;
		basic_acceleration_structure_constructor<box_t, tri_t> *ctor = 0;
		basic_acceleration_structure<box_t, tri_t> *sbvh = 0;

		switch (cmdline.bvh_trav) {
			case Cmdline::bbvh_cis:
			case Cmdline::bbvh_dis:
				ctor = new sbvh_constructor<sbvh_t, bbvh_ctor_t>(bbvh_ctor_t::spatial_median);
				sbvh = ctor->build(&triangle_lists);
				break;
			case Cmdline::preorder_sbvh:
				ctor = new sbvh_preorder_constructor<preorder_sbvh_t, bbvh_ctor_t>(bbvh_ctor_t::spatial_median);
				sbvh = ctor->build(&triangle_lists);
				break;
			case Cmdline::sbvh_oi:
				ctor = new sbvh_constructor<oi_sbvh_t, bbvh_ctor_with_sa_t>(bbvh_ctor_with_sa_t::spatial_median);
				sbvh = ctor->build(&triangle_lists);
				break;
			default:
				cerr << "unhandled case in sbvh ctor switch! (" << cmdline.bvh_trav << ")" << endl;
				exit(EXIT_FAILURE);
		}

		cout << "create rt" << endl;
		basic_raytracer<box_t, tri_t> *rt = 0;
		switch (cmdline.bvh_trav) {
			case Cmdline::bbvh_cis:
				rt = new bbvh_child_is_tracer<box_t, tri_t, sbvh_t>(0, dynamic_cast<sbvh_t*>(sbvh), 0);
				break;
			case Cmdline::bbvh_dis:
				rt = new bbvh_direct_is_tracer<box_t, tri_t, sbvh_t>(0, dynamic_cast<sbvh_t*>(sbvh), 0);
				break;
			case Cmdline::preorder_sbvh:
				rt = new preorder_sbvh_tracer<box_t, tri_t, preorder_sbvh_t>(0, dynamic_cast<preorder_sbvh_t*>(sbvh), 0);
				break;
			case Cmdline::sbvh_oi:
				cout << "---> " << sbvh << endl;
				cout << "---> " << dynamic_cast<oi_sbvh_t*>(sbvh) << endl;
				rt = new order_independent_sbvh_tracer<box_t, tri_t, oi_sbvh_t>(0, dynamic_cast<oi_sbvh_t*>(sbvh), 0);
				break;
			default:
				cerr << "unhandled case in sbvh trav switch! (" << cmdline.bvh_trav << ")" << endl;
				exit(EXIT_FAILURE);
		}

		rt_set set;
		set.as = sbvh;
		set.ctor = ctor;
		set.rt = rt;

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


