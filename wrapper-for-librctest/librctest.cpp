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

using namespace std;
using namespace rc;
using namespace lib3dmath;

int argc; char **argv;

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

template<typename Trav, typename RC, typename AS> void render(RC &rc, AS *as)
{
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
		rc.prepare_new_frame(real_eye, real_dir, real_up);
		rc.template cast_through<Trav>(*as, cmdline.threads);

#ifdef HAVE_CALLGRIND
		CALLGRIND_STOP_INSTRUMENTATION;
#endif

		std::ostringstream fn; fn << "/tmp/test-" << std::setfill('0') << std::setw(3) << i << ".png";
		rc.save_image(fn.str());
	}

	// output statistics
	if (cmdline.rec_out != "")
	{
		ofstream out(cmdline.rec_out.c_str(), ios_base::app | ios_base::out);
		out << "series: " << cmdline.measuring_series << endl;
		out << "cmdline: "; 
		for (int i = 0; i < ::argc; ++i) 
			out << ::argv[i] << " "; 
		out << endl;
		out << "threads: " << cmdline.threads << endl;
		out << "model: " << cmdline.model_filename << endl;
		out << "bvh-string: " << bvh_string() << endl;
		out << "bvh-short: " << ((cmdline.traversal_scheme == Cmdline::BVH) ? 
		   						string("bvh") : (cmdline.traversal_scheme == Cmdline::MBVH) ? 
		   							string("mbvh") : string("pbvh")) << endl;
		out << "precomp: " << ((cmdline.traversal_scheme==Cmdline::MBVH && cmdline.mbvh_precomp 
		   					|| cmdline.traversal_scheme==Cmdline::PBVH && cmdline.pbvh_opt==Cmdline::PBVH_OPT) ? 
		   						string("yes") : string("no")) << endl;
		out << "frames-rendered: " << cmdline.render_passes << endl;
		out << "resolution: " << cmdline.res_x << "x" << cmdline.res_y << endl;
		out << "time-ms: " << PB::time_str_of("raycasting") << endl;
		char hostname[64];
		gethostname(hostname, 64);
		out << "workstation: " << hostname << endl;
		out << endl;
	}
}

define_profile_type(bvh_builder_profile);

template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 	// why doesn't my default value work?
		 typename Adj,
		 template<typename> class BvhTravTr,
		 template<typename, typename, typename> class RcTriTr> 
void ray_casting_with_bvh(vector<ifs_triangle<V> > &tris, ifs<V> *i)
{
	typedef bvh<ifs_triangle<V>, BvhTravTr> Bvh;
	typedef ray_caster<ifs_triangle<V>, RcTriTr, Bvh, rccfg<true, true> > ray_caster;
	ray_caster rc(cmdline.res_x, cmdline.res_y);
	
	cout << "building bvh..." << endl;
	cout << "sizeof bvh: " << sizeof(bvh<ifs_triangle<V>, BvhTravTr>) << endl;
	
	
// 	print_rusage("before building bvh");
	BvhBuilder<ifs_triangle<V>, BvhTravTr, Adj, false> bvh_builder;
	bvh<ifs_triangle<V>, BvhTravTr> *mybvh;
	{
	Profile<bvh_builder_profile<> > p;
	mybvh = bvh_builder(tris);
	}
// 	print_rusage("after building bvh");
		


	cout << "starting the real action..." << endl;
	if (cmdline.dump)
	{
		ofstream dumpfile("bvh.out");
		mybvh->dump(dumpfile);
	}
	else
	{
// 		rc.cast_through(*mybvh, cmdline.threads);
		render<outer_traversal_loop<ray_caster, primary_rays_with_shading<bvh_traversal<ray_caster> > > >(rc, mybvh);
	}
	// clear image if dump was set. prevents stupid errors.
	rc.save_image("/tmp/test.png");
}

template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 
		 typename Adj,
		 template<typename> class MbvhTravTr,
		 template<typename, typename, typename> class RcTriTr,
		 bool Precomp> 
void ray_casting_with_mbvh(vector<ifs_triangle<V> > &tris, ifs<V> *i)
{
	#define dont_care bvh_traits_intersect_children_push_if_hit_and_sensible
	typedef bvh<ifs_triangle<V>, dont_care> Bvh;
	typedef mbvh<ifs_triangle<V>, MbvhTravTr> Mbvh;
	typedef BvhBuilder<ifs_triangle<V>, dont_care, Adj, true> BvhB;
	typedef ray_caster<ifs_triangle<V>, RcTriTr, Mbvh, rccfg<true,true> > ray_caster;
	ray_caster rc(cmdline.res_x, cmdline.res_y);
	
	cout << "building bvh..." << endl;
	cout << "sizeof bvh: " << sizeof(Mbvh) << endl;
	BvhB bvh_builder;

// 	print_rusage("before building mbvh's bvh");
	Bvh *tmp_bvh = bvh_builder(tris);
// 	print_rusage("after building bvh, now converting to mbvh");
	Mbvh *mymbvh = new Mbvh(tmp_bvh);
// 	print_rusage("after building mbvh");
		
	typedef typename if_then_else<	!Precomp,
									outer_traversal_loop<ray_caster, primary_rays_with_shading<mbvh_traversal<ray_caster> > >,
									outer_traversal_loop<ray_caster, primary_rays_with_shading<mbvh_traversal_precomp<ray_caster> > > >::Result traversal_func;

	cout << "starting the real action..." << endl;
	if (cmdline.dump)
	{
		ofstream dumpfile("bvh.out");
		mymbvh->dump(dumpfile);
	}
	else
	{
		render<traversal_func>(rc, mymbvh);
	}
	// clear image if dump was set. prevents stupid errors.
	rc.save_image("/tmp/test.png");
	
	#undef dont_care
}

template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 
		 typename Adj,
		 bool Precomp,
		 template<typename, typename, typename> class RcTriTr> 
void ray_casting_with_pbvh(vector<ifs_triangle<V> > &tris, ifs<V> *i)
{
	#define dont_care bvh_traits_intersect_children_push_if_hit_and_sensible
	typedef bvh<ifs_triangle<V>, dont_care> Bvh;
	typedef pbvh<ifs_triangle<V>, Precomp> Pbvh;
	enum { i_need_external_split_axis_db = false };
	typedef BvhBuilder<ifs_triangle<V>, dont_care, Adj, i_need_external_split_axis_db> BvhB;
	typedef ray_caster<ifs_triangle<V>, RcTriTr, Pbvh, rccfg<true,true> > ray_caster;
	ray_caster rc(cmdline.res_x, cmdline.res_y);
	
	cout << "building bvh..." << endl;
	cout << "sizeof bvh: " << sizeof(Pbvh) << endl;
	BvhB bvh_builder;
	Bvh *tmp_bvh = bvh_builder(tris);
	Pbvh *mypbvh = new Pbvh(tmp_bvh);

	cout << "starting the real action..." << endl;
/*
	if (cmdline.dump)
	{
		ofstream dumpfile("bvh.out");
		mymbvh->dump(dumpfile);
	}
	else
	{
	*/
	typedef typename if_then_else<is_pbvh_no_precomp<Pbvh>::Result, 
								  outer_traversal_loop<ray_caster, primary_rays_with_shading<pbvh_traversal<ray_caster> > >, 
								  outer_traversal_loop<ray_caster, primary_rays_with_shading<pbvh_traversal_precomp<ray_caster> > > >::Result traversal_func;
	render<traversal_func>(rc, mypbvh);
// 	}
	// clear image if dump was set. prevents stupid errors.
	rc.save_image("/tmp/test.png");
// 	*/
	
	#undef dont_care
}



template<typename V, 
		 template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, 
		 typename Adj,
		 template<typename> class BvhTravTr>
inline void append_triangle_traits(vector<ifs_triangle<V> > &tris, ifs<V> *i)
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
									triangle_traits_sequential_intersection_opt>	(tris, i);
		}
		else
		{
			cout << " * using not optimized sequential triangle intersection algorithm" << endl;
			ray_casting_with_bvh<	V,
									BvhBuilder,
									Adj,
									BvhTravTr,
									triangle_traits_sequential_intersection>	(tris, i);
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
									triangle_traits_sse_intersection_opt> (tris, i);
		}
		else
		{
			cout << " * using sse triangle intersection algorithm without triangle cache" << endl;
			ray_casting_with_bvh<	V,
									BvhBuilder,
									Adj,
									BvhTravTr,
// 									triangle_traits_sse_intersection_no_cache>	(tris, i);
									triangle_traits_sse_intersection_opt> (tris, i);
		}
	}
}


template<typename V, template<typename, template<typename> class Tr, typename Adj, bool> class BvhBuilder, typename Adj> 
void ray_casting_path(vector<ifs_triangle<V> > &tris, ifs<V> *i)
{
	// ffd29c
	point_light<V>::point_lights.push_back(point_light<V>(V(upper_light_pos.x,upper_light_pos.y,upper_light_pos.z), V(1,.82,.61)));
	// 9cc1ff
	point_light<V>::point_lights.push_back(point_light<V>(V(lower_light_pos.x,lower_light_pos.y,lower_light_pos.z), V(.61,.75,1)));

	aabb<V> scenebb = compute_aabb<V>(tris);
	cout << scenebb.min << " x " << scenebb.max << endl;

	
	/*
// 	bvh_builder_using_object_median<ifs_triangle<V>, bvh_default_traits> bvh_builder;
// 	BvhBuilder<ifs_triangle<V>, bvh_default_traits, sse_bvh_adjustment> bvh_builder;
	BvhBuilder<ifs_triangle<V>, bvh_default_traits, no_bvh_adjustment> bvh_builder;
// 	BvhBuilder<ifs_triangle<V>, bvh_default_traits, Adj> bvh_builder;
	bvh<ifs_triangle<V>, bvh_default_traits> *mybvh = bvh_builder(tris);
	*/

	if (cmdline.traversal_scheme == Cmdline::BVH)
	{
		if (cmdline.bvh_opt == Cmdline::UNSRT_PIH)
		{
			cout << " * will intersect parent node and push the children unsorted" << endl;
			append_triangle_traits<V, BvhBuilder, Adj, bvh_traits_intersect_box_push_unsorted_if_hit>(tris, i);
		}
		else if (cmdline.bvh_opt == Cmdline::UNSRT_PIS)
		{
			cout << " * will intersect parent node and push the children unsorted, but only if sensible" << endl;
			append_triangle_traits<V, BvhBuilder, Adj, bvh_traits_intersect_box_push_unsorted_if_sensible>(tris, i);
		}
		else if (cmdline.bvh_opt == Cmdline::SORT_PIHS)
		{
			cout << " * will intersect a nodes children and push them in order" << endl;
			append_triangle_traits<V, BvhBuilder, Adj, bvh_traits_intersect_children_push_if_hit_and_sensible>(tris, i);
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
									triangle_traits_sequential_intersection_opt, false>	(tris, i);
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
											triangle_traits_sse_intersection, false>	(tris, i);
				}
				else
				{
					cout << " * using mbvh (sse-tri, no cache) " << endl;
					ray_casting_with_mbvh<	V,
											BvhBuilder,
											Adj,
											mbvh_traits_simple_traversal_sse_tri,
											triangle_traits_sse_intersection_no_cache, false>	(tris, i);
				}

			}
			break;
		case Cmdline::MBVH_SIMPLE_SSE_BB:
			cout << " * using mbvh (sse-bb) " << endl;
			ray_casting_with_mbvh<	V,
									BvhBuilder,
									Adj,
									mbvh_traits_simple_traversal_sse_bb,
									triangle_traits_sequential_intersection_opt, false>	(tris, i);
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
												triangle_traits_sse_intersection, false>	(tris, i);
					}
					else
					{
						cout << " * using mbvh (sse-bb, sse-tri, no cache) " << endl;
						ray_casting_with_mbvh<	V,
												BvhBuilder,
												Adj,
												mbvh_traits_simple_traversal_sse_bb_sse_tris,
												triangle_traits_sse_intersection_no_cache, false>	(tris, i);
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
												triangle_traits_sse_intersection, true>	(tris, i);
					}
					else
					{
						cout << " * using mbvh (sse-bb, sse-tri, no cache) with precomputed slabs test" << endl;
						ray_casting_with_mbvh<	V,
												BvhBuilder,
												Adj,
												mbvh_traits_simple_traversal_sse_bb_sse_tris,
												triangle_traits_sse_intersection_no_cache, true>	(tris, i);
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
										(tris, i);
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
										(tris, i);
		}
	}


	/*
	cout << "starting the real action..." << endl;
	if (cmdline.traversal_scheme == Cmdline::BVH)
	{
		if (cmdline.dump)
		{
			ofstream dumpfile("bvh.out");
			mybvh->dump(dumpfile);
		}
		else
			rc.cast_through(*mybvh, cmdline.threads);
	}
	else if (cmdline.traversal_scheme == Cmdline::MBVH)
	{
		mbvh<ifs_triangle<V>, bvh_traits_intersect_box_push_unsorted_if_hit> mymbvh(mybvh);
		if (cmdline.dump)
		{
			ofstream dumpfile("bvh.out");
			mymbvh.dump(dumpfile);
		}
	}
	*/
	
				
// 	for (typename std::vector<point_light<V> >::iterator it = point_light<V>::point_lights.begin(); it != point_light<V>::point_lights.end(); ++it)
// 		cout << it->pos << ", " << it->color << endl;
}

define_profile_type(main_profile);

ObjFileLoader* create_test_scene()
{
	ObjFileLoader *obj = new ObjFileLoader(ObjFileLoader::FAKE, "");
	int N = 3;
	int idx = 0;
	for (int x = -N; x < N; ++x)
		for (int y = -N; y < N; ++y)
			for (int z = -N; z < N; ++z)
			{
				obj->AddVertex(x, y, z);
				obj->AddVertex(x, y+1, z+1);
				obj->AddVertex(x+1, y+1, z);
				cout << " " << x << " " << y << " " << z << " " << endl;
				cout << " " << x << " " << y+1 << " " << z+1 << " " <<  endl;
				cout << " " << x+1 << " " << y+1 << " " << z << " " <<  endl;
				obj->AddFaceNode(ObjFileLoader::V, ++idx);
				obj->AddFaceNode(ObjFileLoader::V, ++idx);
				obj->AddFaceNode(ObjFileLoader::V, ++idx);
				obj->FaceDone();
			}
	cout << idx << " triangles, " << idx/3 << "faces " << endl;
// 	exit(0);

	return obj;
}

ObjFileLoader* load_model(const std::string &filename)
{
	mat4f trafo;
	ObjFileLoader *obj = new ObjFileLoader(filename, trafo);

	// applies the trafo
	vector<ifs_triangle<vec3f> > tris;
	ifs<vec3f> *i = obj->convert(tris);
	aabb<vec3f> bb = compute_aabb<vec3f>(tris);

	float size = 8.0;
	if (!cmdline.eye_dist_set)
		cmdline.eye_dist = 1.5 * size;

	upper_light_pos = cmdline.init_up * size * 1.5;
	lower_light_pos = -cmdline.init_up * size * 1.5;

	// scale
	vec3f dist = bb.max - bb.min;
	float largest = max(dist.x, max(dist.y, dist.z));
	mat4f scale(vec4f(size/largest, size/largest, size/largest, 1));

	// translate
	vec3f trans = -size*(dist/2 + bb.min);
	scale[0][3] = trans.x / largest;
	scale[1][3] = trans.y / largest;
	scale[2][3] = trans.z / largest;

	obj->Trafo(scale * obj->Trafo());

	cout << cmdline.init_up << endl;
	cout << cmdline.init_dir << endl;
	cout << cmdline.rotation_axis << endl;
	return obj;
}

int main(int argc, char **argv)
{	
	::argc = argc;
	::argv = argv;
#ifdef HAVE_CALLGRIND
	CALLGRIND_STOP_INSTRUMENTATION;
#endif
	int done = 1;
	try
	{
		Profile<main_profile<> > p;

		if (parse_cmdline(argc, argv) != 0)
			return -1;

		if (cmdline.test_aabb_intersection)
		{
			test_aabb_intersection();
			throw done;
		}

// 		tbb::task_scheduler_init task_sched(cmdline.threads);
		start_tbb(cmdline.threads);

	// 	cout << "threads = " << cmdline.threads << endl;

// 		print_rusage("starting to load the model now");
		cout << "loading model " << cmdline.model_filename << " ..." << endl;

// 		ObjFileLoader obj("bunny-70k.obj", "0.05 0 0 0   0 0 -0.05 4   0 -0.05 0 -6   0 0 0 1");
// 		ObjFileLoader obj("bunny-5000.obj", "30 0 0 0   0 30 0 -2   0 0 30 -6   0 0 0 1");
		ObjFileLoader *obj = load_model(cmdline.model_filename);
		
// 		vector<ifs_triangle<vec3f> > tris;
// 		ifs<vec3f> *i = obj->convert(tris);

// 		vector<ifs_triangle<vec3f> > tris;
// 		ifs<vec3f> *i = obj->convert(tris);

// 		print_rusage("making ifs");
		ifs_preperator<vec3f> ifsgen;
		obj->convert(&ifsgen);
		ifsgen.ready();

		vector<ifs_triangle<vec3f> > &tris = ifsgen.tri_arr;
		ifs<vec3f> *i = ifsgen.set;
// 		print_rusage("done makeing ifs");

// 		delete obj;
// 		print_rusage("deleted obj");
		
		compute_aabb<vec3f>(tris[0]);

// 		print_rusage("main, after loading the obj model");

		cout << "converting model..." << endl;

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

// 			print_rusage("after converting the model to sse");
			if ( ! cmdline.force_no_median_adjustment)
			{
				typedef sse_bvh_adjustment adj;
				if (cmdline.median == Cmdline::OBJECT)
				{
					cout << " * using object median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_object_median, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_spatial_median, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_sah, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins_opt, adj>(ssetris, isse);
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
					ray_casting_path<vec4fsse, bvh_builder_using_object_median, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_spatial_median, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_sah, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins, adj>(ssetris, isse);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec4fsse, bvh_builder_using_bins_opt, adj>(ssetris, isse);
				}
				else
					cerr << "unkonw splitting approach" << endl;
			}
		}
		else
		{
			cout << " * not using sse" << endl;
// 			print_rusage("after doing nothing because sse is off");
			if (!cmdline.force_sse_median_adjustment)
			{
				typedef no_bvh_adjustment adj;
				if (cmdline.median == Cmdline::OBJECT)
				{
					cout << " * using object median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_object_median, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_spatial_median, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec3f, bvh_builder_using_sah, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins_opt, adj>(tris, i);
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
					ray_casting_path<vec3f, bvh_builder_using_object_median, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::SPATIAL)
				{
					cout << " * using spatial median" << endl;
					ray_casting_path<vec3f, bvh_builder_using_spatial_median, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::SAH)
				{
					cout << " * using sah" << endl;
					ray_casting_path<vec3f, bvh_builder_using_sah, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::BBB)
				{
					cout << " * using binning bvh builder" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins, adj>(tris, i);
				}
				else if (cmdline.median == Cmdline::BBB2)
				{
					cout << " * using binning bvh builder++" << endl;
					ray_casting_path<vec3f, bvh_builder_using_bins_opt, adj>(tris, i);
				}
				else
					cerr << "unkonw splitting approach" << endl;
			}
		}

		delete obj;

	}
	catch (int &val)
	{
	}
	PB::print_all_results();

	return 0;
}

