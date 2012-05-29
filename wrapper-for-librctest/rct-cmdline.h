#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>

#include <lib3dmath/matrix.h>

//! \brief Translated command line options
struct Cmdline
{
	typedef lib3dmath::mat4f mat4f;
	typedef lib3dmath::vec3f vec3f;
	bool verbose;	//!< wheather or not to be verbose
	int threads;
	unsigned int res_x, res_y;

	bool use_sse;

	bool test_aabb_intersection;
	enum aabb_is_t { CPU_STD, SIMPLE_SSE };
	aabb_is_t test_aabb_type;

	bool dump;

	enum acceleration_structures { BVH, MBVH, PBVH };
	acceleration_structures traversal_scheme;

	enum median_t { OBJECT, SPATIAL, SAH, BBB, BBB2 };
	median_t median;

	enum bvh_opt_t { UNSRT_PIH, UNSRT_PIS, SORT_PIHS };
	bvh_opt_t bvh_opt;

	bool force_sse_median_adjustment;
	bool force_no_median_adjustment;
	bool force_sequential_tri_is;
	bool force_nonopt_tri_is;
	bool no_triangle_cache;

	int triangle_cache_size;

	enum mbvh_opt_t { MBVH_SIMPLE, MBVH_SIMPLE_SSE_TRI, MBVH_SIMPLE_SSE_BB, MBVH_SIMPLE_SSE_BB_TRI  };
	mbvh_opt_t mbvh_opt;
	bool mbvh_precomp;

	enum pbvh_opt_t { PBVH_INIT, PBVH_OPT };
	pbvh_opt_t pbvh_opt;

	int render_passes;

	std::string model_filename;
	vec3f rotation_axis, init_up, init_dir;
	bool eye_dist_set;
	float eye_dist;

	std::string rec_out;
	std::string measuring_series;

#ifdef WITH_MATRIX
	mat4f matrix;
#endif

	Cmdline() : verbose(false), threads(-1), res_x(512), res_y(256), use_sse(false), test_aabb_intersection(false), dump(false), 
			traversal_scheme(BVH), median(OBJECT), bvh_opt(SORT_PIHS), 
			force_sse_median_adjustment(false), force_no_median_adjustment(false), 
			force_sequential_tri_is(false), force_nonopt_tri_is(false), no_triangle_cache(false),
			mbvh_opt(MBVH_SIMPLE), mbvh_precomp(false), pbvh_opt(PBVH_INIT), render_passes(16),
			rotation_axis(0,1,0), init_up(0,1,0), init_dir(0,0,1), eye_dist_set(false), eye_dist(7), model_filename("bunny-70k.obj"),
			measuring_series("none")
	{
	}
};

extern Cmdline cmdline;

int parse_cmdline(int argc, char **argv);

#endif

