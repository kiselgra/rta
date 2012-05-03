#include "image.h"
#include "basic_types.h"
#include "cmdline.h"
#include "wall-timer.h"
#include "intersect.h"

#include <libobjloader/default.h>
#include <libplyloader/plyloader.h>

#include <string>
#include <list>
#include <float.h>

/*
todo:
	- got to work for flat-tri-list and ifs
	- bbvh ctor must work for om/sm
	- got to work for simple-tri, tex-tri
	- gotta work for two aabb types (not fully implemented, but conceptually)
*/

using namespace std;

namespace rta {


////////////////////
#define box_t__and__tri_t typename _box_t, typename _tri_t
#define forward_traits _box_t, _tri_t
#define declare_traits_types typedef _box_t box_t; typedef _tri_t tri_t;
#define traits_of(X) typename X::box_t, typename X::tri_t

////////////////////

struct flat_triangle_list {
	uint triangles;
	simple_triangle *triangle;
	flat_triangle_list() : triangles(0), triangle(0) {}
	flat_triangle_list(int size) : triangles(size), triangle(0) { triangle = new simple_triangle[size]; }
};

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

#include "raytrav.h"
#include "bbvh.h"


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

class directional_analysis_pass {
		ply sphere;
		vec3f *vertex;
		int vertices;
		rta::cam_ray_generator_shirley crgs;
		rta::raytracer *rt;
		rta::binary_png_tester coll;
	public:
		directional_analysis_pass(const std::string sphere_file, int res_x, int res_y) 
		: vertex(0), vertices(0), crgs(res_x, res_y), rt(0), coll(res_x, res_y) {
			load_ply_file(sphere_file.c_str(), &sphere);
			auto elem = sphere.element("vertex");
			vertices = elem->count;
			vertex = new vec3f[vertices];
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
		rta::cam_ray_generator_shirley* ray_gen() { 
			return &crgs; 
		}
		void tracer(rta::raytracer *tracer) { 
			rt = tracer; 
		}
		rta::bouncer* bouncer() { 
			return &coll; 
		}
		void run() {
			for (int i = 0; i < vertices; ++i) {
				crgs.setup(&cmdline.pos, &cmdline.dir, &cmdline.up, 45);
				crgs.generate_rays();

				rt->trace();

				coll.save("/tmp/blub.png");
			}
		}
};

int main(int argc, char **argv) {
	parse_cmdline(argc, argv);

	using namespace rta;
	using namespace std;

	typedef simple_triangle tri_t;
	typedef simple_aabb box_t;
	typedef binary_bvh<box_t, tri_t> bvh_t;
	typedef multi_bvh_sse<box_t, tri_t> mbvh_t;

	cout << "loading object" << endl;
	auto triangle_lists = load_objfile_to_flat_tri_list("/home/kai/render-data/models/drache.obj");
	
	/*
	uint res_x = 512, 
		 res_y = 512;
	cout << "initializing rays" << endl;
	cam_ray_generator_shirley crgs(res_x, res_y);
	crgs.setup(&cmdline.pos, &cmdline.dir, &cmdline.up, 45);
	crgs.generate_rays();
	
	cout << "building bvh" << endl;
// 	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::object_median);
	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::spatial_median);
	bvh_t *bvh = ctor.build(&triangle_lists.front());
	cout << "trace" << endl;
	binary_png_tester coll(res_x, res_y);
// 	bbvh_direct_is_tracer<box_t, tri_t> rt(&crgs, bvh, &coll);
	bbvh_child_is_tracer<box_t, tri_t> rt(&crgs, bvh, &coll);
	rt.trace();
	cout << "save" << endl;
	coll.save("/tmp/blub.png");
	cout << "done" << endl;
	*/


	{
	directional_analysis_pass dap("/home/kai/sphere0.ply", 1024, 512);
	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::spatial_median);
	bvh_t *bvh = ctor.build(&triangle_lists.front());
	bbvh_child_is_tracer<box_t, tri_t> rt(dap.ray_gen(), bvh, dap.bouncer());
	dap.tracer(&rt);
	dap.run();
	}

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


	return 0;
}

/* vim: set foldmethod=marker: */

