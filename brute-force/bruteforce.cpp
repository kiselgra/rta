#include <argp.h>

#include <vector>

#include "librta/librta.h"
#include "librta/wall-timer.h"

using namespace std;
using namespace rta;



//// cmdline handling



struct Cmdline {
	bool verbose;

	Cmdline() : verbose(false) {}
};
static Cmdline cmdline;

const char *argp_program_version = VERSION;

static char plugin_name[] = "bruteforce";
static char doc[]       = "bruteforce: an example rta plugin.";
static char args_doc[]  = "";

enum { FIRST = 300, BT, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,          0, "Be verbose." },	// note: verbose would/should be available via rta, too.
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

	switch (key)
	{
	case 'v':	cmdline.verbose = true; 	break;
	case ARGP_KEY_ARG:		// process arguments. 
							// state->arg_num gives number of current arg
				  return ARGP_ERR_UNKNOWN;
	default:
		return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

static struct argp parser = { options, parse_options, args_doc, doc };



//// brute force implementation



namespace rta {
	namespace example {

		/*! \brief Implementation of the \ref acceleration_structure interface for brute force ray tracing.
		 *  \note For the BF case this is bogus, of course :)
		 */
		template<box_t__and__tri_t> class bruteforce_dummy_accel_struct : public acceleration_structure<forward_traits> {
		public:
			declare_traits_types;

			std::vector<tri_t> triangles;

			/*! take the triangles stored in the array. 
			 * 	\attention does so destructively!
			 */
			void take_triangle_array(std::vector<tri_t> &t) {
				triangles.swap(t);
			}

			tri_t* triangle_ptr() { return &triangles[0]; }
			int triangle_count() { return triangles.size(); }

			virtual std::string identification() { return "dummy accel struct for brute force ray tracing"; }
		};

		/*! \brief Constructor for the dummy acceleration structure, \ref bruteforce_dummy_accel_struct, implementing \ref acceleration_structure_constructor.
		 *  \note For the case of BF RT this is really bogus.
		 *  \note We forward our template parameters but it is not really required to happen this way, so feel free.
		 */
		template<box_t__and__tri_t> class bruteforce_dummy_as_ctor : public acceleration_structure_constructor<forward_traits> {
		public:
			declare_traits_types;

			/*! We just pass on the triangle list; your version may be permuted or made larger by the introduction of duplicates.
			 * 	The triangle data returned by the acceleration structure must reflect those changes or 
			 * 		return intersection information respecting the original configuration (which would probably be harder to do).
			 */
			bruteforce_dummy_accel_struct<forward_traits>* build(flat_triangle_list *tris) {
				bruteforce_dummy_accel_struct<forward_traits> *as = new bruteforce_dummy_accel_struct<forward_traits>;
				std::vector<tri_t> tmp;
				for (int i = 0; i < tris->triangles; ++i)
					tmp.push_back(tris->triangle[i]);
				as->take_triangle_array(tmp);
				return as;
			}
			
			virtual std::string identification() { return "brute force ray tracer's dummy acceleration structure ctor"; }
		};

		/*! \brief Implemantation of the \ref raytracer interface via \ref basic_raytracer to integrate nicely into the evaluation code.
		 */
		template<box_t__and__tri_t> class bruteforce_tracer : public basic_raytracer<forward_traits> {
			declare_traits_types;

			// we "use" these here to avoid more ugly syntax in the functions using them.
			using basic_raytracer<forward_traits>::raygen;
			using basic_raytracer<forward_traits>::cpu_bouncer;

			bruteforce_dummy_accel_struct<forward_traits> *as;

		public:
			bruteforce_tracer(ray_generator *gen, bouncer *b, bruteforce_dummy_accel_struct<forward_traits> *as) : basic_raytracer<forward_traits>(gen, b, as), as(as) {
			}

			virtual float trace_rays() {
				wall_time_timer wtt; wtt.start();
				traversal_state<tri_t> state;
				for (uint y = 0; y < raygen->res_y(); ++y) {
					if (cmdline.verbose)
						cout << "~" << flush;
					for (uint x = 0; x < raygen->res_x(); ++x) {
						state.reset(x,y);
						trace_ray(state, raygen->origin(x,y), raygen->direction(x,y));
						cpu_bouncer->save_intersection(x,y,state.intersection);
					}
				}
				float ms = wtt.look();
				return ms;
			}

			/*! The separation between \ref trace_rays and \ref trace_ray is used throughout the code.
			 *  \ref trace_rays is usually the same code, it is, however, not integrated into the interface
			 *  to prevent the \ref trace_ray from requiring a virtual call for each single ray.
			 *
			 *  We hope this is a price worth paying :)
			 */
			void trace_ray(traversal_state<tri_t> &state, const vec3_t *origin, const vec3_t *dir) {
				tri_t *tri = as->triangle_ptr();
				uint tris = as->triangle_count();

				for (int i = 0; i < tris; ++i) {
					triangle_intersection<tri_t> is(i);
					if (intersect_tri_opt(tri[i], origin, dir, is)) {
						if (is.t < state.intersection.t)
							state.intersection = is;
					}
				}
			}
			virtual std::string identification() { return "brute force ray tracer"; }
		};
	}
}



//// plugin interface



extern "C" {
	char* description() {
		return (char*)"a brute force ray caster";
	}

	void initialize() {}

	int parse_cmdline(int argc, char **argv)
	{
		argv[0] = plugin_name;
		int ret = argp_parse(&parser, argc, argv, 0, 0, 0);
		return ret;
	}

	rt_set<simple_aabb, simple_triangle> create_rt_set(flat_triangle_list &triangle_lists, int w, int h) {
		using namespace rta::example;

		typedef simple_triangle tri_t;
		typedef simple_aabb box_t;

		rt_set<box_t, tri_t> set;
		set.ctor = new bruteforce_dummy_as_ctor<box_t, tri_t>;
		set.as = set.ctor->build(&triangle_lists);	// here we just use the first triangle list; this is actually stupid.
		set.rt = new bruteforce_tracer<box_t, tri_t>(0, 0, dynamic_cast<bruteforce_dummy_accel_struct<box_t, tri_t>*>(set.as));
		
		return set;
	}
}



/* vim: set foldmethod=marker: */

