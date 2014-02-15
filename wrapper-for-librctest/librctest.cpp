#include <librc-test/rc.h>
#include <librc-test/objloader.h>

#include <librc-test/test_ray_box.h>

#include <librc-test/profiling.h>

#include <librc-test/vecs.h>

#include "rct-cmdline.h"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <vector>
#include <ctime>

#include "librta/librta.h"
#include "librta/wall-timer.h"

namespace rctest {

using namespace std;
using namespace rc;
using namespace lib3dmath;

int argc; char **argv;

typedef lib3dmath::vec3f vec3f;
typedef lib3dmath::vec4f vec4f;
typedef lib3dmath::vec3i vec3i;

rta::flat_triangle_list *flat_tris = 0;

ifs_preperator<vec3f>* flat_tris_to_ifs_prep(rta::flat_triangle_list &ftl) {
	flat_tris = &ftl;

	std::vector<vec3f> v, n, t;
	v.resize(ftl.triangles*3);
	n.resize(ftl.triangles*3);
	t.resize(ftl.triangles*3);
	std::vector<vec3i> vi, ni, ti;
	vi.resize(ftl.triangles);
	ni.resize(ftl.triangles);
	ti.resize(ftl.triangles);
	cout << "ftl: " << ftl.triangles << endl;
	for (int i = 0; i < ftl.triangles; i++) {
		v[3*i+0] = vec3f(ftl.triangle[i].a.x, ftl.triangle[i].a.y, ftl.triangle[i].a.z);
		v[3*i+1] = vec3f(ftl.triangle[i].b.x, ftl.triangle[i].b.y, ftl.triangle[i].b.z);
		v[3*i+2] = vec3f(ftl.triangle[i].c.x, ftl.triangle[i].c.y, ftl.triangle[i].c.z);
		n[3*i+0] = vec3f(ftl.triangle[i].na.x, ftl.triangle[i].na.y, ftl.triangle[i].na.z);
		n[3*i+1] = vec3f(ftl.triangle[i].nb.x, ftl.triangle[i].nb.y, ftl.triangle[i].nb.z);
		n[3*i+2] = vec3f(ftl.triangle[i].nc.x, ftl.triangle[i].nc.y, ftl.triangle[i].nc.z);
		t[3*i+0] = t[3*i+1] = t[3*i+2] = 0;
	}
	cout << "S " << v.size() << endl;
	for (int i = 0; i < ftl.triangles; i++) {
		vi[i].x = 3*i;
		vi[i].y = 3*i+1;
		vi[i].z = 3*i+2;
		ni[i].x = 3*i;
		ni[i].y = 3*i+1;
		ni[i].z = 3*i+2;
		ti[i] = i/3;	// this is a bad hack: we use the texture coordinate to be able to index the triangle in the original array.
	}
	ifs_preperator<vec3f> *prep = new ifs_preperator<vec3f>;
	prep->add_model_data(&v, &n, &t);
	prep->insert_mesh_data(&vi, &ni, &ti);
	prep->ready();
	return prep;
}

std::string bvh_string();

struct rctest_component {
};

template<typename Trav, typename InnerTrav, typename RC, typename AS> struct concrete_rctest_component : public rctest_component {
	RC *rc;
	AS *as;
};


	/*! this scheme traverses primary rays and stores their intersection results
	 * 	note that this is not restricted to camera rays.
	 * 	\attention: you have to allocate the image yourself!
	 */
	template<typename Trav> class collect_primary_ray_intersection_information_plugin
	{
	public:
		typedef typename Trav::Rc Rc;
		typedef typename Trav::bvh_type bvh_type;
		typedef typename Trav::bvh_trav_state bvh_trav_state;
		typedef typename Rc::vector_type administrative_arith_t;
		typedef typename Rc::triangle_type triangle_type;
		typedef typename image<triangle_intersection, 1>::ref hitpoints_ref;
		typedef typename image<triangle_type*, 1>::ref hittri_ref;
		static hitpoints_ref result_is;
		static hittri_ref result_tri;

		static void reset()
		{
		}
		static void for_ray(int x, int y, const typename Trav::Rc::vector_type &rc_dir, const typename Trav::Rc::vector_type &rc_eye,  
							typename Trav::bvh_trav_state &state, typename Trav::Rc *rc, const typename Trav::bvh_type &bvh)
		{
			state.reset(x, y);
			typename Trav::vector_type eye(rc_eye);
			typename Trav::triangle_traits::vector_type eye_tis(rc_eye);
			typename Trav::vector_type dir(rc_dir);
			typename Trav::triangle_traits::vector_type dir_tis(rc_dir);

			Trav::trav(eye, dir, eye_tis, dir_tis, state, bvh);
			
			result_is->pixel(state.x, state.y, 0) = state.intersection;
			result_tri->pixel(state.x, state.y, 0) = state.triangle_ref;
		}
		static void allocate(unsigned int w, unsigned int h)
		{
			result_is = hitpoints_ref(new image<triangle_intersection, 1>(w, h));
			result_tri = hittri_ref(new image<triangle_type*, 1>(w, h));
		}
	};
	template<typename Trav> typename collect_primary_ray_intersection_information_plugin<Trav>::hitpoints_ref collect_primary_ray_intersection_information_plugin<Trav>::result_is;
	template<typename Trav> typename collect_primary_ray_intersection_information_plugin<Trav>::hittri_ref collect_primary_ray_intersection_information_plugin<Trav>::result_tri;

template<typename Trav, typename InnerTrav, typename RC, typename AS> 
struct rctest_raytracer : public rta::basic_raytracer<rta::simple_aabb, rta::simple_triangle>, public concrete_rctest_component<Trav, InnerTrav, RC, AS> {
	using basic_raytracer<rta::simple_aabb, rta::simple_triangle>::raygen;
	using basic_raytracer<rta::simple_aabb, rta::simple_triangle>::cpu_bouncer;
	typename collect_primary_ray_intersection_information_plugin<InnerTrav>::hitpoints_ref result_is;
	rctest_raytracer(rta::ray_generator *raygen, rta::bouncer *bouncer, rta::acceleration_structure<rta::simple_aabb, rta::simple_triangle> *as) 
	: basic_raytracer(raygen, bouncer, as) {
		result_is = collect_primary_ray_intersection_information_plugin<InnerTrav>::result_is;
	}
	virtual float trace_rays() {
		for (uint y = 0; y < raygen->res_y(); ++y)
			for (uint x = 0; x < raygen->res_x(); ++x) {
				rc::triangle_intersection &ti = collect_primary_ray_intersection_information_plugin<InnerTrav>::result_is->pixel(x,y,0);
				ti.t = FLT_MAX;
			}
		wall_time_timer wtt; wtt.start();
		this->rc->template cast_through<Trav>(*this->as, cmdline.threads);
		float ms = wtt.look();
		for (uint y = 0; y < raygen->res_y(); ++y)
			for (uint x = 0; x < raygen->res_x(); ++x) {
				rc::triangle_intersection ti = collect_primary_ray_intersection_information_plugin<InnerTrav>::result_is->pixel(x,y,0);
				typename RC::triangle_type tri;
			  	if (collect_primary_ray_intersection_information_plugin<InnerTrav>::result_tri->pixel(x,y,0))
					tri = *collect_primary_ray_intersection_information_plugin<InnerTrav>::result_tri->pixel(x,y,0);
				int idx = tri.ta;
				rta::triangle_intersection<rta::simple_triangle> is; 
				is.beta = ti.beta; 
				is.gamma = ti.gamma;
				is.t = ti.t;
// 				is.ref = this->accel_struct->triangle_ptr() + idx;
				is.ref = idx;
				cpu_bouncer->save_intersection(x,y,is);
			}
		return ms;
	}
	std::string identification() {
		return "kai's da rt (" + bvh_string() + ")";
	}
	rctest_raytracer* copy() { return new rctest_raytracer(*this); }
};

template<typename Trav, typename InnerTrav, typename RC, typename AS> 
struct rctest_as_ctor : public rta::acceleration_structure_constructor<rta::simple_aabb, rta::simple_triangle>, public concrete_rctest_component<Trav, InnerTrav, RC, AS> {
	virtual rta::acceleration_structure<rta::simple_aabb, rta::simple_triangle>* build(rta::flat_triangle_list *tris) {
		return 0;
	}
	std::string identification() {
		return "kai's da rt ctor. dummy.";
	}
};

template<typename Trav, typename InnerTrav, typename RC, typename AS> 
struct rctest_as : public rta::acceleration_structure<rta::simple_aabb, rta::simple_triangle>, public concrete_rctest_component<Trav, InnerTrav, RC, AS> {
	int tri_cnt;
	tri_t *p;
	rctest_as() : tri_cnt(flat_tris->triangles),  p(flat_tris->triangle) {}
	std::string identification() {
		return "kai's da rt bvh: " + bvh_string();
	}
	int triangle_count() {
		return tri_cnt;
	}
	tri_t *triangle_ptr() {
		return p;
	}
};

ostream& operator<<(ostream &o, rta::vec3_t v) {
	o << v.x << " " << v.y << " " << v.z;
	return o;
}

template<typename Trav, typename InnerTrav, typename RC, typename AS> 
struct rctest_rgen : public rta::cam_ray_generator_shirley, public concrete_rctest_component<Trav, InnerTrav, RC, AS> {
	rctest_rgen(int w, int h, RC *rc) : cam_ray_generator_shirley(w, h) { this->rc = rc; }
	virtual void generate_rays() {
		typedef typename RC::vector_type V;
	   	V p(position.x, position.y, position.z);
		V d(dir.x, dir.y, dir.z);
		V u(up.x, up.y, up.z);
		this->rc->prepare_new_frame(p, d, u, fovy);
	}
};


static rta::rt_set<rta::simple_aabb, rta::simple_triangle>	set;

int select_mode(ifs_preperator<vec3f> &ifsgen, rctest_component *&res);
int width = 0, height = 0;

extern "C" {
	char* description() {
		return (char*)"kai's da ray tracer";
	}

	int parse_cmdline(int argc, char **argv)
	{
		argv[0] = (char*)"librctest";
		int ret = librc_parse_cmdline(argc, argv);
		return ret;
	}

	rta::rt_set<rta::simple_aabb, rta::simple_triangle> create_rt_set(rta::flat_triangle_list &triangle_lists, int w, int h) {
		ifs_preperator<vec3f> *ifsp = flat_tris_to_ifs_prep(triangle_lists);

		rctest_component *cmp = 0;
		width = w, height = h;
		select_mode(*ifsp, cmp);
		
		return set;
	}

}

// i guess this is far from complete...
std::string bvh_string()
{
	ostringstream oss;
	switch (cmdline.traversal_scheme)
	{
	case Cmdline::BVH:		oss << "bvh-";
							switch (cmdline.bvh_opt)
							{
							case Cmdline::UNSRT_PIH:	oss << "unsrt.pih-";	break;
							case Cmdline::UNSRT_PIS:	oss << "unsrt.pis-";	break;
							case Cmdline::SORT_PIHS:	oss << "srt.pihs-";		break;
							}
							if (cmdline.use_sse)
							{
								oss << "sse-";
								if (cmdline.force_no_median_adjustment)	oss << "no.med.adj-";
								if (cmdline.force_nonopt_tri_is)		oss << "non.opt.tri.is-";
								if (cmdline.force_sequential_tri_is)	oss << "non.sse.tri.is-";
								if (cmdline.no_triangle_cache)			oss << "no.tri.cache-";
								else									oss << "with.tri.cache-";
							}
							else
							{
								if (cmdline.force_sse_median_adjustment)	oss << "nosse.but.sse.med.adj-";
								if (cmdline.force_nonopt_tri_is)			oss << "non.opt.tri.is-";
							}
							break;
	case Cmdline::MBVH:		oss << "mbvh-";
							switch (cmdline.mbvh_opt)
							{
							case Cmdline::MBVH_SIMPLE:				oss << "t:std-b:std-";	break;
							case Cmdline::MBVH_SIMPLE_SSE_TRI:		oss << "t:sse-b:std-";	break;
							case Cmdline::MBVH_SIMPLE_SSE_BB:		oss << "t:std-b:sse-";	break;
							case Cmdline::MBVH_SIMPLE_SSE_BB_TRI:	oss << "t:sse-b:sse-";	break;
							}
							if (cmdline.mbvh_precomp) 		oss << "precomp-";
							if (cmdline.no_triangle_cache)	oss << "no.tri.cache-";
							else							oss << "with.tri.cache-";
							break;
	case Cmdline::PBVH: 	oss << "pbvh-";
							if (cmdline.pbvh_opt == Cmdline::PBVH_OPT) oss << "precomp-";
							break;
	}
	switch (cmdline.median)
	{
	case Cmdline::OBJECT:	oss << "med:obj";			break;
	case Cmdline::SPATIAL:	oss << "med:spa";			break;
	case Cmdline::SAH:		oss << "med:obsolete-sah";	break;
	case Cmdline::BBB:		oss << "med:bins";			break;
	case Cmdline::BBB2:		oss << "med:binsopt";		break;
	}
	return oss.str();
}

vec3f upper_light_pos, lower_light_pos;



template<typename Trav, typename InnerTrav, typename RC, typename AS> void render(RC *rc, AS *as, ifs<typename RC::vector_type> *i, rctest_component *&res)
{

	collect_primary_ray_intersection_information_plugin<InnerTrav>::allocate(width, height);


	res = new concrete_rctest_component<Trav, InnerTrav, RC, AS>;
	((concrete_rctest_component<Trav, InnerTrav, RC, AS>*)res)->rc = rc;
	((concrete_rctest_component<Trav, InnerTrav, RC, AS>*)res)->as = as;
	rctest_as<Trav, InnerTrav, RC, AS> *my_as = new rctest_as<Trav, InnerTrav, RC, AS>;
	rctest_as_ctor<Trav, InnerTrav, RC, AS> *my_ctor = new rctest_as_ctor<Trav, InnerTrav, RC, AS>;
	rctest_raytracer<Trav, InnerTrav, RC, AS> *my_rt = new rctest_raytracer<Trav, InnerTrav, RC, AS>(0, 0, my_as);
	((concrete_rctest_component<Trav, InnerTrav, RC, AS>*)my_rt)->rc = rc;
	((concrete_rctest_component<Trav, InnerTrav, RC, AS>*)my_rt)->as = as;
	set.rt = my_rt;
	set.as = my_as; // my ass
	set.ctor = my_ctor;
	set.rgen = new rctest_rgen<Trav, InnerTrav, RC, AS>(width, height, rc);


	tbb::task_scheduler_init task_sched(cmdline.threads);

	return;

#ifdef HAVE_CALLGRIND
	CALLGRIND_START_INSTRUMENTATION;
#endif

// 	print_rusage("start of render");

	typedef typename RC::vector_type V;
// 	const int N = 360;
	const int N = cmdline.render_passes;
	for (int i = 0; i < N; ++i)
	{
// 		print_rusage("in loop");
#ifdef HAVE_CALLGRIND
		CALLGRIND_START_INSTRUMENTATION;
#endif
		
		mat4f m(cmdline.rotation_axis, i * (360.0/N));
		vec3f eye = -cmdline.eye_dist * cmdline.init_dir;
		eye = m*eye;

		V real_eye = V(eye.x, eye.y, eye.z);
		V real_dir = -real_eye;
		V real_up = V(cmdline.init_up.x, cmdline.init_up.y, cmdline.init_up.z);
		normalize(real_dir);

// 		cout << "system: " << real_eye << ", " << real_dir << ", " << real_up << endl;
		cout << "frame " << i << endl;
		rc->prepare_new_frame(real_eye, real_dir, real_up);
		rc->template cast_through<Trav>(*as, cmdline.threads);

#ifdef HAVE_CALLGRIND
		CALLGRIND_STOP_INSTRUMENTATION;
#endif

		std::ostringstream fn; fn << "/tmp/test-" << std::setfill('0') << std::setw(3) << i << ".png";
		rc->save_image(fn.str());
	}
}

define_profile_type(bvh_builder_profile);

template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 	// why doesn't my default value work?
		 typename Adj,
		 template<typename> class BvhTravTr,
		 template<typename, typename, typename> class RcTriTr> 
void ray_casting_with_bvh(vector<ifs_triangle<V> > &tris, ifs<V> *i, rctest_component *&res)
{
	typedef bvh<ifs_triangle<V>, BvhTravTr> Bvh;
	typedef ray_caster<ifs_triangle<V>, RcTriTr, Bvh, rccfg<true, true> > ray_caster;
	ray_caster *rc = new ray_caster(width, height);
	
	cout << "building bvh..." << endl;
	cout << "sizeof bvh: " << sizeof(bvh<ifs_triangle<V>, BvhTravTr>) << endl;
	
	
	wall_time_timer wtt;
	wtt.start();
	BvhBuilder<ifs_triangle<V>, BvhTravTr, Adj, false> bvh_builder;
	bvh<ifs_triangle<V>, BvhTravTr> *mybvh;
	{
	Profile<bvh_builder_profile<> > p;
	mybvh = bvh_builder(tris);
	}
	cout << "bvh construction took " << wtt.look() /1000.0f<< "s" << endl;
		


	cout << "starting the real action..." << endl;
	if (cmdline.dump)
	{
		ofstream dumpfile("bvh.out");
		mybvh->dump(dumpfile);
	}
	else
	{
		render<outer_traversal_loop<ray_caster, collect_primary_ray_intersection_information_plugin<bvh_traversal<ray_caster>>>, bvh_traversal<ray_caster>>(rc, mybvh, i, res);
// 		render<outer_traversal_loop<ray_caster, primary_rays_with_shading<bvh_traversal<ray_caster> > > >(rc, mybvh, i, res);
	}
}

template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 
		 typename Adj,
		 template<typename> class MbvhTravTr,
		 template<typename, typename, typename> class RcTriTr,
		 bool Precomp> 
void ray_casting_with_mbvh(vector<ifs_triangle<V> > &tris, ifs<V> *i, rctest_component *&res)
{
	#define dont_care bvh_traits_intersect_children_push_if_hit_and_sensible
	typedef bvh<ifs_triangle<V>, dont_care> Bvh;
	typedef mbvh<ifs_triangle<V>, MbvhTravTr> Mbvh;
	typedef BvhBuilder<ifs_triangle<V>, dont_care, Adj, true> BvhB;
	typedef ray_caster<ifs_triangle<V>, RcTriTr, Mbvh, rccfg<true,true> > ray_caster;
	ray_caster *rc = new ray_caster(width, height);
	
	cout << "building bvh..." << endl;
	cout << "sizeof bvh: " << sizeof(Mbvh) << endl;
	BvhB bvh_builder;

	Bvh *tmp_bvh = bvh_builder(tris);
	Mbvh *mymbvh = new Mbvh(tmp_bvh);
		
	typedef typename if_then_else<	!Precomp,
									outer_traversal_loop<ray_caster, collect_primary_ray_intersection_information_plugin<mbvh_traversal<ray_caster> > >,
									outer_traversal_loop<ray_caster, collect_primary_ray_intersection_information_plugin<mbvh_traversal_precomp<ray_caster> > > >::Result traversal_func;

	typedef typename if_then_else<	!Precomp,
									mbvh_traversal<ray_caster>,
									mbvh_traversal_precomp<ray_caster>>::Result inner_traversal_func;

	cout << "starting the real action..." << endl;
	if (cmdline.dump)
	{
		ofstream dumpfile("bvh.out");
		mymbvh->dump(dumpfile);
	}
	else
	{
		render<traversal_func, inner_traversal_func>(rc, mymbvh, i, res);
	}
	// clear image if dump was set. prevents stupid errors.
// 	rc->save_image("/tmp/test.png");
	
	#undef dont_care
}

template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 
		 typename Adj,
		 bool Precomp,
		 template<typename, typename, typename> class RcTriTr> 
void ray_casting_with_pbvh(vector<ifs_triangle<V> > &tris, ifs<V> *i, rctest_component *&res)
{
	#define dont_care bvh_traits_intersect_children_push_if_hit_and_sensible
	typedef bvh<ifs_triangle<V>, dont_care> Bvh;
	typedef pbvh<ifs_triangle<V>, Precomp> Pbvh;
	enum { i_need_external_split_axis_db = false };
	typedef BvhBuilder<ifs_triangle<V>, dont_care, Adj, i_need_external_split_axis_db> BvhB;
	typedef ray_caster<ifs_triangle<V>, RcTriTr, Pbvh, rccfg<true,true> > ray_caster;
	ray_caster *rc = new ray_caster(width, height);
	
	cout << "building bvh..." << endl;
	cout << "sizeof bvh: " << sizeof(Pbvh) << endl;
	BvhB bvh_builder;
	Bvh *tmp_bvh = bvh_builder(tris);
	Pbvh *mypbvh = new Pbvh(tmp_bvh);

	cout << "bvh is ready." << endl;
	typedef typename if_then_else<is_pbvh_no_precomp<Pbvh>::Result, 
								  outer_traversal_loop<ray_caster, collect_primary_ray_intersection_information_plugin<pbvh_traversal<ray_caster> > >, 
								  outer_traversal_loop<ray_caster, collect_primary_ray_intersection_information_plugin<pbvh_traversal_precomp<ray_caster> > > >::Result traversal_func;
	typedef typename if_then_else<is_pbvh_no_precomp<Pbvh>::Result, 
								  pbvh_traversal<ray_caster>, 
								  pbvh_traversal_precomp<ray_caster>>::Result inner_traversal_func;
	render<traversal_func, inner_traversal_func>(rc, mypbvh, i, res);
	
	#undef dont_care
}



template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 
		 typename Adj,
		 template<typename> class BvhTravTr>
inline void append_triangle_traits(vector<ifs_triangle<V> > &tris, ifs<V> *i, rctest_component *&res)
{
	if ( ! cmdline.use_sse || cmdline.force_sequential_tri_is)
	{
		if ( ! cmdline.force_nonopt_tri_is)
		{
			cout << " * using optimized sequential triangle intersection algorithm" << endl;
			ray_casting_with_bvh<	V,
									BvhBuilder,
									Adj,
									BvhTravTr,
									triangle_traits_sequential_intersection_opt>	(tris, i, res);
		}
		else
		{
			cout << " * using not optimized sequential triangle intersection algorithm" << endl;
			ray_casting_with_bvh<	V,
									BvhBuilder,
									Adj,
									BvhTravTr,
									triangle_traits_sequential_intersection>	(tris, i, res);
		}
	}
	else 
	{
		if ( ! cmdline.no_triangle_cache)
		{
			cout << " * using sse triangle intersection algorithm" << endl;
			ray_casting_with_bvh<	V,
									BvhBuilder,
									Adj,
									BvhTravTr,
// 									triangle_traits_sse_intersection>	(tris, i);
									triangle_traits_sse_intersection_opt> (tris, i, res);
		}
		else
		{
			cout << " * using sse triangle intersection algorithm without triangle cache" << endl;
			ray_casting_with_bvh<	V,
									BvhBuilder,
									Adj,
									BvhTravTr,
// 									triangle_traits_sse_intersection_no_cache>	(tris, i);
									triangle_traits_sse_intersection_opt> (tris, i, res);
		}
	}
}


template<typename V, template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, typename Adj> 
void ray_casting_path(vector<ifs_triangle<V> > &tris, ifs<V> *i, rctest_component *&res)
{
	aabb<V> scenebb = compute_aabb<V>(tris);
	cout << scenebb.min << " x " << scenebb.max << endl;

	vec3f bbmax = vec3f(x_comp(scenebb.max), y_comp(scenebb.max), z_comp(scenebb.max));
	vec3f bbmin = vec3f(x_comp(scenebb.min), y_comp(scenebb.min), z_comp(scenebb.min));
	upper_light_pos = bbmax;
	lower_light_pos = bbmin;
	
	// ffd29c
	point_light<V>::point_lights.push_back(point_light<V>(V(upper_light_pos.x,upper_light_pos.y,upper_light_pos.z), V(1,.82,.61)));
	// 9cc1ff
	point_light<V>::point_lights.push_back(point_light<V>(V(lower_light_pos.x,lower_light_pos.y,lower_light_pos.z), V(.61,.75,1)));

	
	if (cmdline.traversal_scheme == Cmdline::BVH)
	{
		if (cmdline.bvh_opt == Cmdline::UNSRT_PIH)
		{
			cout << " * will intersect parent node and push the children unsorted" << endl;
			append_triangle_traits<V, BvhBuilder, Adj, bvh_traits_intersect_box_push_unsorted_if_hit>(tris, i, res);
		}
		else if (cmdline.bvh_opt == Cmdline::UNSRT_PIS)
		{
			cout << " * will intersect parent node and push the children unsorted, but only if sensible" << endl;
			append_triangle_traits<V, BvhBuilder, Adj, bvh_traits_intersect_box_push_unsorted_if_sensible>(tris, i, res);
		}
		else if (cmdline.bvh_opt == Cmdline::SORT_PIHS)
		{
			cout << " * will intersect a nodes children and push them in order" << endl;
			append_triangle_traits<V, BvhBuilder, Adj, bvh_traits_intersect_children_push_if_hit_and_sensible>(tris, i, res);
		}
		else
		{
			cerr << __FILE__ << ":" << __LINE__ << " check for cmdline option missing!" << endl;
		}
	}
	else if (cmdline.traversal_scheme == Cmdline::MBVH)
	{
		switch (cmdline.mbvh_opt)
		{
		case Cmdline::MBVH_SIMPLE:
			cout << " * using mbvh (simple) " << endl;
			ray_casting_with_mbvh<	V,
									BvhBuilder,
									Adj,
									mbvh_traits_simple_traversal,
									triangle_traits_sequential_intersection_opt, false>	(tris, i, res);
			break;
		case Cmdline::MBVH_SIMPLE_SSE_TRI:
			{
				if ( ! cmdline.no_triangle_cache)
				{
					cout << " * using mbvh (sse-tri) " << endl;
					ray_casting_with_mbvh<	V,
											BvhBuilder,
											Adj,
											mbvh_traits_simple_traversal_sse_tri,
											triangle_traits_sse_intersection, false>	(tris, i, res);
				}
				else
				{
					cout << " * using mbvh (sse-tri, no cache) " << endl;
					ray_casting_with_mbvh<	V,
											BvhBuilder,
											Adj,
											mbvh_traits_simple_traversal_sse_tri,
											triangle_traits_sse_intersection_no_cache, false>	(tris, i, res);
				}

			}
			break;
		case Cmdline::MBVH_SIMPLE_SSE_BB:
			cout << " * using mbvh (sse-bb) " << endl;
			ray_casting_with_mbvh<	V,
									BvhBuilder,
									Adj,
									mbvh_traits_simple_traversal_sse_bb,
									triangle_traits_sequential_intersection_opt, false>	(tris, i, res);
			break;

		case Cmdline::MBVH_SIMPLE_SSE_BB_TRI:
			{
				if ( ! cmdline.mbvh_precomp)
				{
					if ( ! cmdline.no_triangle_cache)
					{
						cout << " * using mbvh (sse-bb, sse-tri) " << endl;
						ray_casting_with_mbvh<	V,
												BvhBuilder,
												Adj,
												mbvh_traits_simple_traversal_sse_bb_sse_tris,
												triangle_traits_sse_intersection, false>	(tris, i, res);
					}
					else
					{
						cout << " * using mbvh (sse-bb, sse-tri, no cache) " << endl;
						ray_casting_with_mbvh<	V,
												BvhBuilder,
												Adj,
												mbvh_traits_simple_traversal_sse_bb_sse_tris,
												triangle_traits_sse_intersection_no_cache, false>	(tris, i, res);
					}
				}
				else
				{
					if ( ! cmdline.no_triangle_cache)
					{
						cout << " * using mbvh (sse-bb, sse-tri) with precomputed slabs test" << endl;
						ray_casting_with_mbvh<	V,
												BvhBuilder,
												Adj,
												mbvh_traits_simple_traversal_sse_bb_sse_tris,
												triangle_traits_sse_intersection, true>	(tris, i, res);
					}
					else
					{
						cout << " * using mbvh (sse-bb, sse-tri, no cache) with precomputed slabs test" << endl;
						ray_casting_with_mbvh<	V,
												BvhBuilder,
												Adj,
												mbvh_traits_simple_traversal_sse_bb_sse_tris,
												triangle_traits_sse_intersection_no_cache, true>	(tris, i, res);
					}
				}

			}
			break;

		}
	}
	else if (cmdline.traversal_scheme == Cmdline::PBVH)
	{
		if (cmdline.pbvh_opt == Cmdline::PBVH_INIT)
		{
			cout << " * using p-aabb approach" << endl;
			ray_casting_with_pbvh<	V,
									BvhBuilder,
									Adj,
									false,
									triangle_traits_sse_intersection_opt>	// TODO: switch on this?
// 									triangle_traits_sse_intersection>	// TODO: switch on this?
// 									triangle_traits_sequential_intersection_opt>	// TODO: switch on this?
										(tris, i, res);
		}
		else if (cmdline.pbvh_opt == Cmdline::PBVH_OPT)
		{
			cout << " * using p-aabb approach with slabs precomputation" << endl;
			ray_casting_with_pbvh<	V,
									BvhBuilder,
									Adj,
									true,
									triangle_traits_sse_intersection_opt>	// TODO: switch on this?
// 									triangle_traits_sse_intersection>	// TODO: switch on this?
// 									triangle_traits_sequential_intersection_opt>	// TODO: switch on this?
										(tris, i, res);
		}
	}
}

define_profile_type(main_profile);


int select_mode(ifs_preperator<vec3f> &ifsgen, rctest_component *&res)
{	
		start_tbb(cmdline.threads);

		vector<ifs_triangle<vec3f> > &tris = ifsgen.tri_arr;
		ifs<vec3f> *i = ifsgen.set;

		if (cmdline.use_sse || cmdline.traversal_scheme == Cmdline::MBVH || cmdline.traversal_scheme == Cmdline::PBVH)
		{
			cout << " * using sse median adj" << endl;
			clock_t start = clock();
			ifs<vec4fsse> *isse = convert_ifs<vec4fsse>(i);		// maybe we should do this at construction time...
			vector<ifs_triangle<vec4fsse> > ssetris;
			for (int i = 0; i < tris.size(); ++i)
				ssetris.push_back(ifs_triangle<vec4fsse>(tris[i], isse));
			clock_t end = clock();
// 			cout << "conversion took " << (end-start)/(CLOCKS_PER_SEC/1000) << " msec" << endl;

			if ( ! cmdline.force_no_median_adjustment)
			{
				typedef sse_bvh_adjustment adj;
				if (cmdline.median == Cmdline::OBJECT)
				{
					cout << " * using object median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_object_median, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_spatial_median, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_sah, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins_opt, adj>(ssetris, isse, res);
				}
				else
					cerr << "unkonw splitting approach" << endl;
			}
			else
			{
				cout << " * forced not to apply sse median adjustment" << endl;
				typedef no_bvh_adjustment adj;
				if (cmdline.median == Cmdline::OBJECT)
				{
					cout << " * using object median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_object_median, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_spatial_median, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_sah, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins, adj>(ssetris, isse, res);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins_opt, adj>(ssetris, isse, res);
				}
				else
					cerr << "unkonw splitting approach" << endl;
			}
		}
		else
		{
			cout << " * not using sse" << endl;
			if (!cmdline.force_sse_median_adjustment)
			{
				typedef no_bvh_adjustment adj;
				if (cmdline.median == Cmdline::OBJECT)
				{
					cout << " * using object median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_object_median, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_spatial_median, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec3f, bvh_builder_using_sah, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins_opt, adj>(tris, i, res);
				}
				else
					cerr << "unkonw splitting approach" << endl;
			}
			else
			{
				cout << " * forced to apply sse median adjustment" << endl;
				typedef sse_bvh_adjustment adj;
				if (cmdline.median == Cmdline::OBJECT)
				{
					cout << " * using object median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_object_median, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_spatial_median, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec3f, bvh_builder_using_sah, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins, adj>(tris, i, res);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins_opt, adj>(tris, i, res);
				}
				else
					cerr << "unkonw splitting approach" << endl;
			}
		}
	return 0;
}

}
