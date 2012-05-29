#include <iostream>
#include <cstring>
#include <sstream>
#include <argp.h>

#include "bbvh.h"

using namespace std;
using namespace rta;


// 
// cmdline
//

struct Cmdline {
	enum bvh_trav_t { cis, dis };
	bvh_trav_t bvh_trav;
	bool verbose;

	Cmdline() : bvh_trav(cis), verbose(false) {}
};
static Cmdline cmdline;

const char *argp_program_version = VERSION;

static char plugin_name[] = "bbvh";
static char doc[]       = "bbvh: a rta plugin.";
static char args_doc[]  = "";

enum { FIRST = 300, BT, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,          0, "Be verbose." },
	{ "bvh-trav", BT, "cis|dis",  0, "Intersection mode of the bvh traversal: direct-is, child-is. Default: cis." },
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
	case BT:      if (sarg == "dis") cmdline.bvh_trav = Cmdline::dis;
	              else if (sarg == "cis") cmdline.bvh_trav = Cmdline::cis;
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

	int parse_cmdline(int argc, char **argv)
	{
		argv[0] = plugin_name;
		int ret = argp_parse(&parser, argc, argv, 0/*ARGP_NO_EXIT*//*0*/, 0, 0);
		return ret;
	}

	rt_set<simple_aabb, simple_triangle> create_rt_set(std::list<flat_triangle_list> &triangle_lists) {
		typedef simple_triangle tri_t;
		typedef simple_aabb box_t;
		typedef binary_bvh<box_t, tri_t> bvh_t;

		bbvh_constructor_using_median<bvh_t> *ctor = new bbvh_constructor_using_median<bvh_t>(bbvh_constructor_using_median<bvh_t>::spatial_median);
		bvh_t *bvh = ctor->build(&triangle_lists.front());

		basic_raytracer<box_t, tri_t> *rt = 0;
		if (cmdline.bvh_trav == Cmdline::cis)
			rt = new bbvh_child_is_tracer<box_t, tri_t>(0, bvh, 0);
		else
			rt = new bbvh_direct_is_tracer<box_t, tri_t>(0, bvh, 0);

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


