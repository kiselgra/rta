#include "rct-cmdline.h"


#include <argp.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cstdlib>



using namespace std;
using namespace lib3dmath;

const char *argp_program_version = VERSION;

static char doc[]       = PACKAGE ": description";
static char args_doc[]  = "argumentdescription";

// long option without corresponding short option have to define a symbolic constant >= 300
enum { FIRST = 300, TESTAABB, DUMP, MBVH, SPA_MED, OBJ_MED, SAH_MED, BVH_TRAIT, FSSEMA, FNOMA, FSEQTIS, FNOTIS, NTRICA, TRICASZ, MBVH_TRAIT, PAABB_OPT,
	   ROTAX, INITUP, INITDIR, EYEDIST, BBB, BBB2, MBVH_PC, RECOUT, SER };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 	'v', 			0,				0, 		"Be verbose." },
	{ "threads", 	't', 			"n",			0,		"Number of threads used."},
	{ "sse",		's',			0,				0,		"Use SSE version."},
	{ "test-aabb",	TESTAABB,		"type",			0,		"Test/benchmark a ray/box intersection method. help lists the available methods." },
	{ "dump",		DUMP,			0,				0,		"Dump the generated hierarchy." },
	{ "mbvh",		MBVH,			0,				0,		"Use MBVH." },
	{ "mbvh-precomp", MBVH_PC,		0,				0,		"Use precomputed slabs test for MBVH. Valid in combination with mbvh-opts 4, only!" },
	{ "spatial-median", SPA_MED,	0,				0,		"Use spatial median when building the BVH. Applies to MBVH, too." },
	{ "object-median", OBJ_MED,		0,				0,		"Use object median when building the BVH. Applies to MBVH, too." },
	{ "sah", 		SAH_MED,		0,				0,		"Use surface area heuristics when building the BVH. Applies to MBVH, too." },
	{ "bvh-opts",	BVH_TRAIT,		"nr",			0,		"Configure how a single bvh node is traversed. The valid possibilities are given "
															"by specific numbers. 1: intersect the bounding volume and push the child nodes unsorted "
															"onto the stack. 2: same as 1, but the traversal does not continue if the hitpoint with "
															"the box is behind an already found triangle. 3: intersect the child nodes of a given "
															"bounding volume and push each of them (if hit) in order." },
	{ "force-sse-median-adjustment", FSSEMA, 0,		0,		"Create bounding volumes with exactly 4 triangles per node, regardless of the use of SSE. "
															"There will be at most one node with less than 4 triangles." },
	{ "force-no-median-adjustment", FNOMA, 0,		0,		"Create bounding volumes exactly according to the computed median, regardless of the use "
															"of SSE." },
	{ "force-sequential-triangle-is", FSEQTIS, 0,	0,		"Use the sequential triangle intersection algorithm, even if using SSE." },
	{ "force-non-optimized-triangle-is", FNOTIS, 0,	0,		"Don't use the optimized triangle intersection algorithm. This is only valid if "
															"the triangle intersection is not done using SSE." },
	{ "no-triangle-cache", NTRICA,	0,				0,		"Don't use the triangle cache. This is only valid if the triangle intersection is done "
															"using SSE." },
	{ "mbvh-opts",	MBVH_TRAIT,		"nr",			0,		"Configure how a single mbvh node is traversed. The valid possibilities are given "
															"by specific number. 1: most simple proof-of-concept implementation. 2: simple traversal " 
															"using sse triangle intersection, but sequential box intersection. 3: traversal using sse "
															"bounding volumes and sequential triangle intersection. 4: use sse implementation "
															"for box and triangle intersection."	},
#ifdef WITH_MATRIX
	{ "matrix",		'm',			"matrix",		0,		"TEMP" },
#endif
	{ "p-aabb",		'p',			0,				0,		"Use parallel AABB intersection for BBVH. Only valid in normal BVH traversal mode." },
	{ "p-aabb-opt",	PAABB_OPT,		"nr",			0,		"Select paabb implementation. 1: default. 2: expoit slabs test precomputations." },
	{ "images",		'N',			"number",		0,		"Specify the number of images rendered while rotating the camera around the moedl." },
	{ "rotation-axis", ROTAX,		"vector",		0,		"Axis around which the animation rotates." },
	{ "init-up",	INITUP,			"vector",		0,		"Initial up-vector." },
	{ "init-dir",	INITDIR,		"vector",		0,		"Initial viewing direction." },
	{ "eye-dist",	EYEDIST,		"scalar",		0,		"Distance of the eye to the rotation point (center of the model)." },
	{ "model",		'o',			"filename",		0,		"Name of the .obj model to load." },
	{ "bbb",		BBB,			0,				0,		"Create BVH using the Binning Bvh Builder (instead of object or spatial median)." },
	{ "bbb2",		BBB2,			0,				0,		"Create BVH using the Binning Bvh Builder (instead of object or spatial median). Improved version." },
	{ "rec-out",	RECOUT,			"filename",		0,		"Appends the recutils style output to the given file." },
	{ "series",		SER,			"text",			0,		"Information about the current measurement series to be put into the recfile." },
	{ 0 }
};	

string& replace_nl(string &s)
{
	for (int i = 0; i < s.length(); ++i)
		if (s[i] == '\n' || s[i] == '\r')
			s[i] = ' ';
	return s;
}


static error_t parse_options(int key, char *arg, argp_state *state)
{
	// call argp_usage to stop program execution if something is wrong
	
	string sarg;
	if (arg)
		sarg = arg;
	sarg = replace_nl(sarg);
	int val;

	switch (key)
	{
#ifdef WITH_MATRIX
	case 'm':	cmdline.matrix = strtomat4f(sarg);	cout << "----" << endl << cmdline.matrix << endl << "----" << endl; break;
#endif
	case 'v':	cmdline.verbose = true; 	break;

	case 't':
				val = atoi(arg);
				if (val <= 0)
					cerr << "invalid number of threads. determining appropriate number myself." << endl;
				else
					cmdline.threads = val;
				break;

	case 's': 		cmdline.use_sse = true;	break;
	case DUMP:		cmdline.dump = true;	break;
	case MBVH:		cmdline.traversal_scheme = Cmdline::MBVH;	break;
	case MBVH_PC:	cmdline.mbvh_precomp = true;	break;
	case 'p':		cmdline.traversal_scheme = Cmdline::PBVH;	break;
	case OBJ_MED:	cmdline.median = Cmdline::OBJECT;	break;
	case SPA_MED:	cmdline.median = Cmdline::SPATIAL;	break;
	case SAH_MED:	cmdline.median = Cmdline::SAH;	break;
	case BBB:		cmdline.median = Cmdline::BBB;	break;
	case BBB2:		cmdline.median = Cmdline::BBB2;	break;
	case FSSEMA:	cmdline.force_sse_median_adjustment = true; break;
	case FNOMA:		cmdline.force_no_median_adjustment = true; break;
	case FSEQTIS:	cmdline.force_sequential_tri_is = true; break;
	case FNOTIS:	cmdline.force_nonopt_tri_is = true; break;
	case NTRICA:	cmdline.no_triangle_cache = true; break;
	case 'o':		cmdline.model_filename = sarg; break;

	case ROTAX:		cmdline.rotation_axis = strtovec3f(sarg);	break;
	case INITUP:	cmdline.init_up = strtovec3f(sarg);	break;
	case INITDIR:	cmdline.init_dir = strtovec3f(sarg);	break;
	case EYEDIST:	cmdline.eye_dist = atof(arg); cmdline.eye_dist_set = true; break;
	case RECOUT:	cmdline.rec_out = sarg; 	break;
	case SER:		cmdline.measuring_series = sarg;	break;
	case 'N':		cmdline.render_passes = atoi(arg);
					if (cmdline.render_passes <= 0 || cmdline.render_passes > 800)
					{
						cerr << "invalid number of render passes: " << cmdline.render_passes << endl;
						cmdline.render_passes = 16;
					}
					break;

	case BVH_TRAIT: val = atoi(arg);
					switch (val)
					{
					case 1:		cmdline.bvh_opt = Cmdline::UNSRT_PIH;	break;
					case 2:		cmdline.bvh_opt = Cmdline::UNSRT_PIS;	break;
					case 3:		cmdline.bvh_opt = Cmdline::SORT_PIHS;	break;
					default:	cerr << "invalid bvh option '" << arg << "'" << endl;
					}
					break;
	
	case MBVH_TRAIT:val = atoi(arg);
					switch (val)
					{
					case 1:		cmdline.mbvh_opt = Cmdline::MBVH_SIMPLE;			break;
					case 2:		cmdline.mbvh_opt = Cmdline::MBVH_SIMPLE_SSE_TRI;	break;
					case 3:		cmdline.mbvh_opt = Cmdline::MBVH_SIMPLE_SSE_BB;		break;
					case 4:		cmdline.mbvh_opt = Cmdline::MBVH_SIMPLE_SSE_BB_TRI;	break;
					default:	cerr << "invalid mbvh option '" << arg << "'" << endl;
					}
					break;

	case PAABB_OPT: val = atoi(arg);
					switch (val)
					{
					case 1:		cmdline.pbvh_opt = Cmdline::PBVH_INIT;	break;
					case 2:		cmdline.pbvh_opt = Cmdline::PBVH_OPT;	break;
					default:	cerr << "invalid p-aabb option '" << arg << "'" << endl;
					}
					break;

	case TESTAABB:{
					if (sarg == "help")
					{
						cout << "possible values: " << endl;
						cout << "\tdefault:    the basic implementation of the slabs test." << endl;
						cout << "\tsimple-sse: a simple approach to emmploy the sse instruction set in the intersection computation." << endl;
						exit(EXIT_SUCCESS);
					}
					cmdline.test_aabb_intersection = true;
					if (sarg == "default")				cmdline.test_aabb_type = Cmdline::CPU_STD;
					else if (sarg == "simple-sse")		cmdline.test_aabb_type = Cmdline::SIMPLE_SSE;
					else
					{
						cerr << "invalid test-aabb mode: '" << sarg << "'" << endl;
						exit(EXIT_FAILURE);
					}
				  }
				  break;

	default:
		return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

static struct argp parser = { options, parse_options, args_doc, doc };

int librc_parse_cmdline(int argc, char **argv)
{
	int ret = argp_parse(&parser, argc, argv, /*ARGP_NO_EXIT*/0, 0, 0);

	// cmdline sanity checks
	if (cmdline.force_nonopt_tri_is && cmdline.use_sse && !cmdline.force_sequential_tri_is)
	{
		cerr << "invalid combination of flags: non-optimized triangle intersection using SSE." << endl;
		return -1;
	}
	if (!(cmdline.use_sse || cmdline.traversal_scheme == Cmdline::MBVH) && cmdline.no_triangle_cache)
	{
		cerr << "option --no-triangle-cache is only valid in SSE or MBVH mode." << endl;
		return -1;
	}
	if (cmdline.mbvh_precomp && cmdline.mbvh_opt != Cmdline::MBVH_SIMPLE_SSE_BB_TRI)
	{
		cerr << "invalid combination of --mbvh-precomp and --mbvh-opts" << endl;
		return -1;
	}

	return ret;
}
	
Cmdline cmdline;

