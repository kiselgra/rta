#include <iostream>
#include <cstring>
#include <sstream>
#include <argp.h>

#include "bbvh-cuda.h"
#include "lbvh.h"

using namespace std;
using namespace rta;


// 
// cmdline
//

struct Cmdline {
	enum bvh_t { median, lbvh };
	enum bvh_trav_t { cis, dis };
	enum bvh_layout_t { layout_cpu, layout_2f4 };
	bvh_t bvh_build;
	bvh_trav_t bvh_trav;
	bvh_layout_t layout;
	bool shadow_rays;
	bool verbose;

	Cmdline() : bvh_build(median), bvh_trav(cis), verbose(false), layout(layout_cpu), shadow_rays(false) {}
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
	{ "verbose", 'v', 0,               0, "Be verbose." },
	{ "bvh",     'b', "<median|lbvh>", 0, "Which kind of bvh to use. Default: median." },
	{ "layout",  'l', "<cpu|2f4>",     0, "Which bvh layout to choose: Standard cpu tracing layout or node encoded in 2 float4. Default: cpu." },
	{ "shadow",  's', 0,               0, "Use the shadow tracer, i.e. early out at first triangle intersection." },
	{ "trav",    't', "<dis|cis>",     0, "Traverse dis/cis. Default: Dis." },
// 	{ "bvh-trav", BT, "cis|dis",  0, "Intersection mode of the bvh traversal: direct-is, child-is. Default: cis." },
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
	case 't':   if (sarg == "dis") cmdline.bvh_trav = Cmdline::dis;
	            else if (sarg == "cis") cmdline.bvh_trav = Cmdline::cis;
				else {
					  cerr << "Unknown bvh traversal scheme: " << sarg << endl;
					  argp_usage(state);
				}
				break;
	case 'b':	if (sarg == "lbvh") cmdline.bvh_build = Cmdline::lbvh;
				break;

	case 'l': 	if (sarg == "2f4") cmdline.layout = Cmdline::layout_2f4;
				break;
	
	case 's': 	cmdline.shadow_rays = true;
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

	void initialize() {
		rta::cuda::plugin_with_cuda_loaded = true;
	}

	int parse_cmdline(int argc, char **argv)
	{
		argv[0] = plugin_name;
		int ret = argp_parse(&parser, argc, argv, 0/*ARGP_NO_EXIT*//*0*/, 0, 0);
		return ret;
	}


	/*   WHAT TODO
	 *
	 *   tri vs cuda::tri -> derive special ctor (in plugin) to handle conversion of triangle types (see bruteforce).
	 *
	 *   maybe we should add some triangle-handling portion of the ctor to a general cuda::ctor interface?
	 *
	 *   hm
	 *
	 */

	rt_set create_rt_set(basic_flat_triangle_list<simple_triangle> &triangle_lists, int w, int h) {
		typedef cuda::simple_triangle tri_t;
		typedef cuda::simple_aabb box_t;
// 		typedef cuda::lbvh_constructor<box_t, tri_t, cuda::binary_lbvh<box_t, tri_t, cuda::binary_bvh<box_t, tri_t>>> lbvh_ctor_t;
		
		acceleration_structure *as = 0;
		acceleration_structure_constructor *base_ctor = 0;
		basic_raytracer<box_t, cuda::simple_triangle> *rt = 0;

		if (cmdline.layout == Cmdline::layout_cpu) {
			typedef cuda::binary_bvh<box_t, tri_t, rta::binary_bvh<box_t, tri_t>> cuda_bvh_t;
			typedef bbvh_constructor_using_median<cuda_bvh_t> std_bbvh_ctor_t;
			typedef cuda::lbvh_constructor<box_t, tri_t, cuda::binary_lbvh<box_t, tri_t, cuda::binary_bvh<box_t, tri_t, rta::binary_bvh<box_t, tri_t>>>> lbvh_ctor_t;
			cuda_bvh_t *bvh = 0;
			if (cmdline.bvh_build == Cmdline::median) {
				std_bbvh_ctor_t *ctor = new std_bbvh_ctor_t(std_bbvh_ctor_t::spatial_median);
				cuda_ftl ftl(triangle_lists);
				bvh = ctor->build(&ftl);
				base_ctor = ctor;
			}
			else {
				cuda_ftl tmp_ftl(triangle_lists);
				cuda_ftl ftl;
				cudaMalloc((void**)&ftl.triangle, sizeof(cuda::simple_triangle)*tmp_ftl.triangles);
				cudaMemcpy(ftl.triangle, tmp_ftl.triangle, sizeof(cuda::simple_triangle)*tmp_ftl.triangles, cudaMemcpyHostToDevice);
				ftl.triangles = tmp_ftl.triangles;
				lbvh_ctor_t *ctor = new lbvh_ctor_t;
				bvh = ctor->build(&ftl);
				base_ctor = ctor;
			}
			as = bvh;
			if (cmdline.bvh_trav == Cmdline::dis)
				if (cmdline.shadow_rays)
					rt = new cuda::bbvh_gpu_dis_shadow_tracer<box_t, cuda::simple_triangle, cuda_bvh_t>(0, bvh, 0);
				else
					rt = new cuda::bbvh_gpu_dis_tracer<box_t, cuda::simple_triangle, cuda_bvh_t>(0, bvh, 0);
			else
				if (cmdline.shadow_rays)
					rt = new cuda::bbvh_gpu_cis_shadow_tracer<box_t, cuda::simple_triangle, cuda_bvh_t>(0, bvh, 0);
				else
					rt = new cuda::bbvh_gpu_cis_tracer<box_t, cuda::simple_triangle, cuda_bvh_t>(0, bvh, 0);
		}
		else {
			if (cmdline.bvh_build == Cmdline::median) {
				// the bvh cannot be gpu only, as the ctor builds it on the host.
				typedef cuda::binary_bvh<box_t, tri_t, binary_bvh<box_t, tri_t, cuda::bbvh_node_float4<box_t>>> bvh_t;
				typedef bbvh_constructor_using_median<bvh_t> ctor_t;
				ctor_t *ctor = new ctor_t(ctor_t::spatial_median);
				cuda_ftl ftl(triangle_lists);
				bvh_t *bvh = ctor->build(&ftl);;
				base_ctor = ctor;
				as = bvh;
				if (cmdline.shadow_rays)
					rt = new cuda::bbvh_gpu_dis_shadow_tracer<box_t, tri_t, bvh_t>(0, bvh, 0);
				else
					rt = new cuda::bbvh_gpu_dis_tracer<box_t, tri_t, bvh_t>(0, bvh, 0);
			}
			else {
				typedef cuda::binary_bvh<box_t, tri_t, binary_bvh<box_t, tri_t, cuda::bbvh_node_float4<box_t>>> bvh_t;
				typedef cuda::binary_lbvh<box_t, tri_t, bvh_t> lbvh_t;
				typedef cuda::lbvh_constructor<box_t, tri_t, lbvh_t> ctor_t;
				cuda_ftl tmp_ftl(triangle_lists);
				cuda_ftl ftl;
				cudaMalloc((void**)&ftl.triangle, sizeof(cuda::simple_triangle)*tmp_ftl.triangles);
				cudaMemcpy(ftl.triangle, tmp_ftl.triangle, sizeof(cuda::simple_triangle)*tmp_ftl.triangles, cudaMemcpyHostToDevice);
				ftl.triangles = tmp_ftl.triangles;
				ctor_t *ctor = new ctor_t;
				lbvh_t *lbvh = ctor->build(&ftl);
				base_ctor = ctor;
				as = lbvh;
				if (cmdline.bvh_trav == Cmdline::dis)
					if (cmdline.shadow_rays)
						rt = new cuda::bbvh_gpu_dis_shadow_tracer<box_t, tri_t, bvh_t>(0, lbvh, 0);
					else
						rt = new cuda::bbvh_gpu_dis_tracer<box_t, tri_t, bvh_t>(0, lbvh, 0);
				else
					if (cmdline.shadow_rays)
						rt = new cuda::bbvh_gpu_cis_shadow_tracer<box_t, tri_t, bvh_t>(0, lbvh, 0);
					else
						rt = new cuda::bbvh_gpu_cis_tracer<box_t, tri_t, bvh_t>(0, lbvh, 0);
			}
		}

// 		if (cmdline.bvh_trav == Cmdline::cis)
// 			rt = new bbvh_child_is_tracer<box_t, tri_t, binary_bvh<box_t, tri_t>>(0, bvh, 0);
// 		else

		rt_set set;
		set.as = as;
		set.ctor = base_ctor;
		set.rt = rt;
// 		set.rgen = new cuda::raygen_with_buffer<cam_ray_generator_shirley>(w, h);
		set.rgen = new cuda::cam_ray_generator_shirley(w, h);
		
		return set;
	}
}

