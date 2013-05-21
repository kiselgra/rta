#include "librta/librta.h"
#include "cmdline.h"
#include "librta/wall-timer.h"
#include "librta/ocl.h"

#include <libobjloader/default.h>
#include <libplyloader/plyloader.h>

#include <libmcm/matrix.h>

#include <string>
#include <sstream>
#include <list>
#include <float.h>
#include <unistd.h>

#include <dlfcn.h>

/*! \defgroup main Main Evaluation Program
 *
 * 	The main evaluation program is (rather crudely) designed to simply load a scene (and atm this is somewhat restricted, too)
 * 		and ray trace a few images of it.
 *	We support three basic types of measurements:
 *	\li Positional, i.e. you specify a lookat position at the cmdline and the program generates a single result image.
 *	\li Axial series, i.e. you specify a rotation axis and a number of samples to be taken. This results in as images as specified, all rotated around this axis.
 *	\li Sphere series, i.e. you specify a model in <em>standford's ply format</em> and the system traces an image from each vertex (scaled to be outside the scene) towards the origin. We provide a few uniformly tesselated sphere models for this purpose.
 *	
 *	Note that, on the cmdline, you can specify all parameters and choose which set to use by the --force switch. See --help.
 *	
 *	By default, the resulting images are stored in <tt>/tmp</tt>.
 */

/*
todo:
	- got to work for flat-tri-list and ifs
	- bbvh ctor must work for om/sm
	- got to work for simple-tri, tex-tri
	- gotta work for two aabb types (not fully implemented, but conceptually)
*/

using namespace std;

namespace rta {

	int res_x, res_y; // gets parsed from command line

////////////////////
////////////////////

std::list<flat_triangle_list> load_objfile_to_flat_tri_list(const std::string &filename) {
	obj_default::ObjFileLoader loader(filename, "1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 1");

	std::list<flat_triangle_list> lists;

	for (auto &group : loader.groups) {
		flat_triangle_list ftl(group.load_idxs_v.size());
		// building those is expensive!
		auto vertex = [=](int id, int i) { auto v = loader.load_verts[((int*)&group.load_idxs_v[i])[id]]; vec3_t r = {v.x,v.y,v.z}; return r; };
		auto normal = [=](int id, int i) { auto v = loader.load_norms[((int*)&group.load_idxs_n[i])[id]]; vec3_t r = {v.x,v.y,v.z}; return r; };
		for (int i = 0; i < ftl.triangles; ++i)	{
			ftl.triangle[i].a = vertex(0, i);
			ftl.triangle[i].b = vertex(1, i);
			ftl.triangle[i].c = vertex(2, i);
			ftl.triangle[i].na = normal(0, i);
			ftl.triangle[i].nb = normal(1, i);
			ftl.triangle[i].nc = normal(2, i);
		}
		lists.push_back(ftl);
	}

	return lists;
}

////////////////////

}

// #include "bbvh/bbvh.h"


////////////////////

// typedef auto function;

#define defun auto
auto foo(int x, int y) -> int {
	return x + y;
}

defun foo2 (int x, int y) -> int {
	return x + y;
}

bool operator==(const vec3f &a, const vec3f &b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

vec3f make_vec3f(float x, float y, float z) {
	vec3f r;
	make_vec3f(&r, x, y, z);
	return r;
}

using namespace rta;

/*! \brief Runs the actual measurements.
 * 	\ingroup main
 *
 * 	This runs the actual measurements as described above (\ref main).
 * 	After the intersection points for all rays have been found we shade the scene by pointlights placed at the corners of its bounding box.
 */
template<box_t__and__tri_t> class directional_analysis_pass {
		declare_traits_types;
		ply sphere;
		vec3f *vertex;
		int vertices;
		float *timings;
		rta::cam_ray_generator_shirley *crgs;
// 		rta::basic_raytracer<box_t, tri_t> *rt;
		lighting_collector<forward_traits> *shader;
		rt_set<box_t, tri_t> the_set;
		vec3f obj_center;
		box_t bb;
		float bb_diam;
		float dist_scale;
		int res_x, res_y;

	public:
		directional_analysis_pass(const std::string &sphere_file, int res_x, int res_y) 
		: vertex(0), vertices(0), timings(0), dist_scale(1.5), res_x(res_x), res_y(res_y), crgs(0), shader(0) {
			setup_sample_positions(sphere_file);
			crgs = new cam_ray_generator_shirley(res_x, res_y);
		}
		directional_analysis_pass(vec3_t &axis, vec3_t &anchor, int samples, int res_x, int res_y)
		: vertex(0), vertices(0), timings(0), dist_scale(1.5), res_x(res_x), res_y(res_y), crgs(0) {
			setup_rotation_positions(axis, anchor, samples);
			crgs = new cam_ray_generator_shirley(res_x, res_y);
		}
		~directional_analysis_pass() {
			delete [] vertex;
			delete [] timings;
		}
		void setup_sample_positions(const std::string &sphere_file) {
			load_ply_file(sphere_file.c_str(), &sphere);
			auto elem = sphere.element("vertex");
			vertices = elem->count;
			vertex = new vec3f[vertices];
			timings = new float[vertices];
			int x = elem->index_of("x"),
				y = elem->index_of("y"),
				z = elem->index_of("z");
			int at = 0;
			for (int i = 0; i < vertices; ++i) {
				bool had_that_one_already = false;
				vec3f new_vert; make_vec3f(&new_vert, elem->ref(i, x), elem->ref(i, y), elem->ref(i, z));
				for (int j = 0; j < i; ++j)
					if (vertex[j] == new_vert) {
						had_that_one_already = true;
						break;
					}
				if (had_that_one_already) 
					continue;
				vertex[at++] = new_vert;
			}
			vertices = at;
			cout << "got " << vertices << " distinct vertices" << endl;
		}
		void mod_and_save_ply(const std::string &out) {
			sphere.allocate_new_vertex_data({"rps"}, "vertex", ply::property_t::t_float32);
			auto elem = sphere.element("vertex");
			int n = elem->count;
			int x = elem->index_of("x"),
				y = elem->index_of("y"),
				z = elem->index_of("z"),
				m = elem->index_of("rps");
			for (int i = 0; i < vertices; ++i)
				for (int j = 0; j < n; ++j) {
					vec3f new_vert; 
					make_vec3f(&new_vert, elem->ref(j, x), elem->ref(j, y), elem->ref(j, z));
					if (vertex[i] == new_vert)
						elem->ref(j, m) = timings[i];
				}

			char host[65];	// see gethostname(2)
			gethostname(host, 65);
			sphere.add_comment("meta host: " + string(host));

			sphere.add_comment("meta accel-struct: " + the_set.as->identification());
			sphere.add_comment("meta accel-struct-ctor: " + the_set.ctor->identification());
			sphere.add_comment("meta ray-tracer: " + the_set.rt->identification());
// 			sphere.add_comment("meta bouncer: " + the_set.bouncer->identification());
			sphere.add_comment("meta bouncer: " + bouncer()->identification());

			save_ply_file(out, &sphere);
		}
		void setup_rotation_positions(vec3_t &axis, vec3_t &anchor, int samples) {
			vertices = samples;
			vertex = new vec3_t[samples];
			timings = new float[samples];
			for (int i = 0; i < samples; ++i) {
				matrix4x4f m;
				vec4_t a = { x_comp(anchor), x_comp(anchor), x_comp(anchor), 0 }, b;
				make_rotation_matrix4x4f(&m, &axis, (2*3.1416/float(samples))*i);
				multiply_matrix4x4f_vec4f(&b, &m, &a);
				x_comp(vertex[i]) = x_comp(b);
				y_comp(vertex[i]) = y_comp(b);
				z_comp(vertex[i]) = z_comp(b);
			}
		}
		rta::cam_ray_generator_shirley*& ray_gen() { 
			return crgs; 
		}
		void set(rt_set<forward_traits> set) {
			the_set = set;
			tracer(set.rt);
			shader = dynamic_cast<lighting_collector<forward_traits>*>(set.bouncer);
			shader->triangle_ptr(the_set.as->triangle_ptr());
		}
		//! supposes that the tracer's accelstruct has been set up, already
		void tracer(rta::basic_raytracer<box_t, tri_t> *tracer) { 
			tri_t *tris = tracer->accel_struct->triangle_ptr();
			int n = tracer->accel_struct->triangle_count();
			bb = rta::compute_aabb<box_t>(tris, 0, n);
			vec3f diff;
			sub_components_vec3f(&diff, &max(bb), &min(bb));
			bb_diam = length_of_vec3f(&diff);
			mul_vec3f_by_scalar(&diff, &diff, 0.5);
			add_components_vec3f(&obj_center, &min(bb), &diff);
		}
		rta::bouncer* bouncer() { 
			return the_set.bouncer; 
		}
		void run() {
			
			vec3f null = make_vec3f(0,0,0);
			float sum = 0;
			for (int i = 0; i < vertices; ++i) {
				vec3f pos;// = vertex[i];

				// position around object
				vec3f from_origin = vertex[i];
				normalize_vec3f(&from_origin);
				mul_vec3f_by_scalar(&from_origin, &from_origin, bb_diam * dist_scale);
				add_components_vec3f(&pos, &obj_center, &from_origin);

				// correct lookat system
				vec3f dir; sub_components_vec3f(&dir, &null, &vertex[i]);
				normalize_vec3f(&dir);
				vec3f up = make_vec3f(0,0,1);
				if (fabs(dot_vec3f(&dir, &up)) > 0.8)
					up = make_vec3f(0,1,0);
				vec3f right;
				cross_vec3f(&right, &dir, &up);
				cross_vec3f(&up, &dir, &right);
				crgs->setup(&pos, &dir, &up, 45);
				crgs->generate_rays();

				the_set.rt->trace();
				float rps = res_x * res_y * (1000.0f / the_set.rt->timings.front());
				sum += rps;
				timings[i] = rps;

				ostringstream oss;
				oss << "/tmp/blub-";
				if (i < 100) oss << "0";
				if (i < 10) oss << "0";
				oss << i;
				oss << ".png";

				box_t light_box = bb;
				vec3_t diff, delta;
				sub_components_vec3f(&diff, &max(bb), &min(bb));
				mul_vec3f_by_scalar(&delta, &diff, 0.1);
				sub_components_vec3f(&min(light_box), &min(light_box), &delta);
				add_components_vec3f(&max(light_box), &max(light_box), &delta);
				
				vec3_t center;
				mul_vec3f_by_scalar(&diff, &diff, 0.5);
				add_components_vec3f(&center, &min(light_box), &diff);

				shader->lights.clear();
				float_t I = 0.3;
				float_t l_r = I * cmdline.light_col.x,
						l_g = I * cmdline.light_col.y,
						l_b = I * cmdline.light_col.z;

				vec3_t mi = min(light_box);
				vec3_t ma = max(light_box);

				shader->add_pointlight({x_comp(min(light_box)), y_comp(min(light_box)), z_comp(min(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(min(light_box)), y_comp(min(light_box)), z_comp(max(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(min(light_box)), y_comp(max(light_box)), z_comp(min(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(min(light_box)), y_comp(max(light_box)), z_comp(max(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(max(light_box)), y_comp(min(light_box)), z_comp(min(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(max(light_box)), y_comp(min(light_box)), z_comp(max(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(max(light_box)), y_comp(max(light_box)), z_comp(min(light_box))}, {l_r, l_g, l_b});
				shader->add_pointlight({x_comp(max(light_box)), y_comp(max(light_box)), z_comp(max(light_box))}, {l_r, l_g, l_b});

				shader->reset(cmdline.back_col);

				shader->shade(cmdline.binary_intersection_debug);
				shader->save(oss.str());

				cout << "." << flush;
			}
			cout << "\naverage rps per frame: " << (sum/vertices)/1000.0f << " K" << endl;
		}
};

/*
rt_set<simple_aabb, simple_triangle> make_bvh_stuff(bouncer *b, ray_generator *raygen, std::list<flat_triangle_list> triangle_lists) {
	typedef simple_triangle tri_t;
	typedef simple_aabb box_t;
	typedef binary_bvh<box_t, tri_t> bvh_t;

	bbvh_constructor_using_median<bvh_t> *ctor = new bbvh_constructor_using_median<bvh_t>(bbvh_constructor_using_median<bvh_t>::spatial_median);
	bvh_t *bvh = ctor->build(&triangle_lists.front());

	basic_raytracer<box_t, tri_t> *rt = 0;
	if (cmdline.bvh_trav == Cmdline::cis)
		rt = new bbvh_child_is_tracer<box_t, tri_t>(raygen, bvh, b);
	else
		rt = new bbvh_direct_is_tracer<box_t, tri_t>(raygen, bvh, b);

	rt_set<box_t, tri_t> set;
	set.as = bvh;
	set.ctor = ctor;
	set.bcr = b;
	set.rt = rt;
	set.rgen = raygen;

	return set;
}
*/

void *lib_handle = 0;
char* (*plugin_description)() = 0;
int (*plugin_parse_cmdline)(int argc, char **argv) = 0;
void (*plugin_initialize)() = 0;
rt_set<simple_aabb, simple_triangle> (*plugin_create_rt_set)(std::list<flat_triangle_list>&,int,int) = 0;

template<typename T> void load_plugin_function(const std::string &name, T &to) {
	to = (T)dlsym(lib_handle, name.c_str());
	char *error;
	if ((error = dlerror()) != NULL)  
	{
		fprintf(stderr, "%s\n", error);
		exit(1);
	}
}

void load_plugin_functions() {
	if (cmdline.module == "") {
		cerr << "No module specified!" << endl;
		exit(EXIT_FAILURE);
	}
	lib_handle = dlopen(("built-plugins/" + cmdline.module + ".so").c_str(),RTLD_LAZY);
	if (lib_handle == 0) {
		lib_handle = dlopen(("built-plugins/librta-" + cmdline.module + ".so").c_str(),RTLD_LAZY);
		if (lib_handle == 0) {
			lib_handle = dlopen((PKG_LIB_DIR "/" + cmdline.module + ".so").c_str(),RTLD_LAZY);
			if (lib_handle == 0) {
				lib_handle = dlopen(cmdline.module.c_str(),RTLD_LAZY);
				if (!lib_handle)
					cout << "error: " << dlerror() << endl;
			}
		}
	}
	if (lib_handle == 0) {
		cerr << "Cannot load plugin " << cmdline.module << "." << endl;
		exit(-1);
	}

	load_plugin_function("description", plugin_description);
	cout << "PD: " << plugin_description() << endl;
	
	load_plugin_function("initialize", plugin_initialize);
	load_plugin_function("create_rt_set", plugin_create_rt_set);
	load_plugin_function("parse_cmdline", plugin_parse_cmdline);
	plugin_initialize();
}

int main(int argc, char **argv) {
	parse_cmdline(argc, argv);

	res_x = cmdline.res_x;
	res_y = cmdline.res_y;

	using namespace rta;
	using namespace std;

	typedef simple_triangle tri_t;
	typedef simple_aabb box_t;
// 	typedef binary_bvh<box_t, tri_t> bvh_t;
// 	typedef multi_bvh_sse<box_t, tri_t> mbvh_t;
	load_plugin_functions();
	int plugin_argc = 0;
	char **plugin_argv = plugin_args(&plugin_argc);
	plugin_parse_cmdline(plugin_argc, plugin_argv);


	cout << "loading object " << cmdline.model << endl;
	auto triangle_lists = load_objfile_to_flat_tri_list(cmdline.model.c_str());
	if (triangle_lists.size() > 1)
		cerr << "Models with more than one submesh are not supported, yet." << endl;
	
	if (ocl::using_ocl()) {
		res_x = ocl::pow2roundup(res_x);
		res_y = ocl::pow2roundup(res_y);
	}

	rt_set<simple_aabb, simple_triangle> set = plugin_create_rt_set(triangle_lists, res_x, res_y);

	if (cmdline.axial_series || cmdline.sphere_series)
	{
		directional_analysis_pass<box_t, tri_t> *dap = 0;
		if (cmdline.axial_series) {
			cout << "running axial series..." << endl;
			dap = new directional_analysis_pass<box_t, tri_t>(cmdline.axis, cmdline.anchor, cmdline.samples, res_x, res_y);
		}
		else {
			cout << "running spherical series..." << endl;
			dap = new directional_analysis_pass<box_t, tri_t>(cmdline.sphere_file, res_x, res_y);
		}
	
		/*

		bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::spatial_median);
		bvh_t *bvh = ctor.build(&triangle_lists.front());
		bbvh_child_is_tracer<box_t, tri_t> rt(dap->ray_gen(), bvh, dap->bouncer());
		dap->tracer(&rt);
		dap->run();
		if (cmdline.outfile != "" && cmdline.sphere_series)
			dap->mod_and_save_ply(cmdline.outfile);

		*/

		if (!ocl::using_ocl())
			set.bouncer = new direct_diffuse_illumination<box_t, tri_t>(res_x, res_y);
#if RTA_HAVE_LIBLUMOCL == 1
		else
			set.bouncer = new ocl::direct_diffuse_illumination<box_t, tri_t>(res_x, res_y, *ocl::context);
#endif

		set.rt->ray_bouncer(set.bouncer);
		if (set.rgen) {
			set.rt->ray_generator(set.rgen);
			cam_ray_generator_shirley *crgs = dynamic_cast<cam_ray_generator_shirley*>(set.rgen);
			if (crgs == 0)
				throw std::logic_error("the ray generator set by the plugin is not compatible!");
			dap->ray_gen() = crgs;
		}
		else {
			set.rgen = dap->ray_gen();
			set.rt->ray_generator(set.rgen);
		}
		dap->set(set);
		dap->run();
		if (cmdline.outfile != "" && cmdline.sphere_series)
			dap->mod_and_save_ply(cmdline.outfile);
	}
	else if (cmdline.positional_series) {
		uint res_x = rta::res_x, res_y = rta::res_y;
		cam_ray_generator_shirley *crgs = new cam_ray_generator_shirley(res_x, res_y);
		cout << "cmdline.pos: " << cmdline.pos << endl;
		cout << "cmdline.dir: " << cmdline.dir << endl;
		cout << "cmdline.up:  " << cmdline.up << endl;
		
		/*

		// 	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::object_median);
		bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::spatial_median);
		bvh_t *bvh = ctor.build(&triangle_lists.front());
		
		binary_png_tester<box_t, tri_t> coll(res_x, res_y);
		// 	bbvh_direct_is_tracer<box_t, tri_t> rt(&crgs, bvh, &coll);
		bbvh_child_is_tracer<box_t, tri_t> rt(&crgs, bvh, &coll);
		rt.trace();
		coll.save("/tmp/blub.png");
		cout << "stored result to /tmp/blub.png" << endl;

		*/
		

		direct_diffuse_illumination<box_t, tri_t> coll(res_x, res_y);
		set.rt->ray_bouncer(&coll);
		if (set.rgen)
			crgs = dynamic_cast<cam_ray_generator_shirley*>(set.rgen);
		set.rt->ray_generator(crgs);
		crgs->setup(&cmdline.pos, &cmdline.dir, &cmdline.up, 45);
		crgs->generate_rays();

		set.rt->trace();
		coll.triangle_ptr(set.as->triangle_ptr());
		coll.add_pointlight(cmdline.pos, {1,1,1});
		coll.reset(cmdline.back_col);
		coll.shade();
		coll.save("/tmp/blub.png");
		cout << "stored result to /tmp/blub.png" << endl;
	}
	else {
		cerr << "Don't know what to do. Try --help." << endl;
		exit(EXIT_FAILURE);
	}

	/*
	if (0)
	{
		bbvh_constructor_using_binning<bvh_t> ctor;
		flat_triangle_list tris;
		bvh_t *my_bbvh = ctor.build(&tris);
	}

	if (0)
	{
		mbvh_sse_contructor<mbvh_t, bbvh_constructor_using_median<bvh_t>> ctor;
		flat_triangle_list tris;
		bbvh_constructor_using_median<bvh_t> bvhctor(bbvh_constructor_using_median<bvh_t>::object_median);
		mbvh_t *my_mbvh = ctor.build(&tris, bvhctor);
	}
	*/


	return 0;
}

/* vim: set foldmethod=marker: */

