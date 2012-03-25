#include "image.h"

#include <libmcm/vectors.h>
#include <libobjloader/default.h>

#include <string>
#include <list>

/*
todo:
	- got to work for flat-tri-list and ifs
	- bbvh ctor must work for om/sm
	- got to work for simple-tri, tex-tri
	- gotta work for two aabb types (not fully implemented, but conceptually)
*/

using namespace std;

namespace rta {

	typedef unsigned int uint;
	typedef float float_t;
	typedef vec3f vec3_t;

	
	struct simple_triangle {
		vec3_t a, b, c;
		vec3_t na, nb, nc;
	};

	struct simple_aabb {
		vec3_t min, max;
	};


class binary_bvh_facade {
	public:
		// node adding, etc?
		void reserver_node_storage(unsigned int nodes);
};

////////////////////
#define box_t__and__tri_t typename _box_t, typename _tri_t
#define forward_traits _box_t, _tri_t
#define declare_traits_types typedef _box_t box_t; typedef _tri_t tri_t;
#define traits_of(X) typename X::box_t, typename X::tri_t

template<box_t__and__tri_t> class binary_bvh : public binary_bvh_facade {
	public:
		declare_traits_types;
		typedef uint32_t link_t;
		struct node {
			link_t type_left_elems; // contains: bit0=inner/leaf >>1=(left-child/tris-in-leaf)
			link_t right_tris;      // contains: right-child/link-to-tris
			box_t box;
			bool inner()         { return (type_left_elems&0x01)==1; }
			void make_inner()    { type_left_elems |= 0x01; }
			void make_leaf()     { type_left_elems &= (~1); }
			void left(link_t n)  { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }
			link_t left()        { return type_left_elems>>1; }
			void elems(uint n)   { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }
			uint elems()         { return type_left_elems>>1; }
			//! link to the right.
			void right(link_t n) { right_tris = n; }
			link_t right()       { return right_tris; }
			//! an index into an array of triangles holding elems() successive triangles enclosed by this node.
			void tris(link_t n)  { right_tris = n; }
			uint tris()          { return right_tris; }
		};
};

template<box_t__and__tri_t> class stackess_binary_bvh : public binary_bvh_facade {
	public:
		declare_traits_types;
		struct node {
		};
};

template<box_t__and__tri_t> class multi_bvh_sse {
	public:
		declare_traits_types;
};

template<box_t__and__tri_t> class multi_bvh_avx {
	public:
		declare_traits_types;
};

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

struct bbvh_no_bias {
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> 
class bbvh_constructor_using_median {
	protected:
		typedef typename bvh_t::node node_t;
		std::vector<node_t> nodes;

		enum axes { X = 0, Y, Z };

		bvh_t* build_om(flat_triangle_list *tris, uint begin, uint end) {
			/*
			int id = nodes.size();
			nodes.push_back(node_t());
			node_t *n = &nodes[id];
			n->box = compute_aabb<vec3f>(tris, begin, end);

			// 	cout << n->box.min.x << "\t" << n->box.min.y << "\t" << n->box.min.z << endl;
			// 	cout << n->box.max.x << "\t" << n->box.max.y << "\t" << n->box.max.z << endl;

			uint elems = end-begin;
			if (elems > max_tris_per_node) 
			{
				vec3f dists; sub_components_vec3f(&dists, &n->box.max, &n->box.min);
				if (dists.x > dists.y)
					if (dists.x > dists.z)	{	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_x);	}
					else                    {	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_z);	}
				else
					if (dists.y > dists.z)	{	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_y);	}
					else                    {	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_z);	}

				uint mid = begin + (end-begin)/2;
				if (sse_compatible) adjust_for_sse(begin, mid, end);

				link_t left  = build_object_median_bvh_from(tris, begin, mid);
				link_t right = build_object_median_bvh_from(tris, mid, end);
				n = &nodes[id]; // refresh pointer!
				n->left(left);
				n->right(right);
				n->make_inner();
			}
			else 
			{
				int idx = tri_lines.size();
				tri_t *leaf_tris = new tri_t[elems];
				tri_lines.push_back(leaf_tris);
				for (int i = 0; i < elems; ++i)
					leaf_tris[i] = tris[begin+i];
				n->elems(elems);
				n->tris(idx);
				n->make_leaf();
			}

			return id;
			*/
		}
	public:
		enum median_t { object_median, spatial_median };
		median_t median;
		bbvh_constructor_using_median(median_t median) : median(median) {
		}
		bvh_t* build(flat_triangle_list *tris) {
// 			bvh_t 
		}
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> class bbvh_constructor_using_sah {
	public:
		bvh_t* build(flat_triangle_list *tris) {
		}
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> class bbvh_constructor_using_binning {
	public:
		bvh_t* build(flat_triangle_list *tris) {
		}
};

template<typename mbvh_t, typename bbvh_ctor_t> class mbvh_sse_contructor {
	public:
		class mbvh_bbvh_building_bias {
		};
		typedef binary_bvh<traits_of(mbvh_t)> bbvh_t;
		mbvh_t* build(flat_triangle_list *tris, bbvh_ctor_t &bbvhctor) {
			bbvh_t *bbvh = bbvhctor.build(tris);
		}
};

////////////////////

class bouncer { // sequential calls to raytrace
	public:
		virtual void trace_single_bounce() = 0;
		virtual bool trace_further_bounces() = 0;
		virtual void new_pass() {}
};

class primary_intersection_collector : public bouncer {
		bool done;
	public:
		primary_intersection_collector() : done(false) {
		}
		virtual void trace_single_bounce() {
			done = true;
		}
		virtual bool trace_further_bounces() {
			return !done;
		}
};

////////////////////

class ray_generator {
	protected:
		image<vec3f, 2> raydata;
	public:
		ray_generator(unsigned int res_x, unsigned int res_y) : raydata(res_x, res_y) {
		}
		virtual void generate_rays() = 0;
		inline uint res_x() const                           { return raydata.w; }
		inline uint res_y() const                           { return raydata.h; }
		inline vec3f* origin(uint x, uint y)                { return &raydata.pixel(x, y, 0); }
		inline const vec3f* origin(uint x, uint y) const    { return &raydata.pixel(x, y, 0); }
		inline vec3f* direction(uint x, uint y)             { return &raydata.pixel(x, y, 1); }
		inline const vec3f* direction(uint x, uint y) const { return &raydata.pixel(x, y, 1); }
};

class cam_ray_generator_shirley : public ray_generator {
	protected:
		float aspect, fovy;
		vec3f position;
		vec3f dir;
		vec3f up;

		void generate_ray_dir(vec3f *dir, float plane_w, float plane_h, unsigned int x, unsigned int y, unsigned int w, unsigned h, const vec3f *view_dir, const vec3f *view_up) { // {{{
			float u_s = ((float)x+0.5f)/(float)w * 2.0f - 1.0f;	// \in (-1,1)
			float v_s = ((float)y+0.5f)/(float)h * 2.0f - 1.0f;
			u_s = plane_w * u_s / 2.0f;	// \in (-pw/2, pw/2)
			v_s = plane_h * v_s / 2.0f;

			vec3f W, TxW, U, V;
			div_vec3f_by_scalar(&W, view_dir, length_of_vec3f(view_dir));
			cross_vec3f(&TxW, view_up, &W);
			div_vec3f_by_scalar(&U, &TxW, length_of_vec3f(&TxW));
			cross_vec3f(&V, &W, &U);

			mul_vec3f_by_scalar(dir, &U, u_s);
			vec3f tmp;
			mul_vec3f_by_scalar(&tmp, &V, v_s);
			add_components_vec3f(dir, dir, &tmp);
			add_components_vec3f(dir, dir, &W);
			normalize_vec3f(dir);
		} // }}}

		void generate_ray_dir_from_cam_parameters(vec3f *dir, float fovy, float aspect, unsigned int x, unsigned int y, unsigned int w, unsigned int h, const vec3f *view_dir, const vec3f *view_up) {
			fovy /= 2.0;
			float height = tanf(M_PI * fovy / 180.0f);
			float width = aspect * height;
			generate_ray_dir(dir, 2*width, 2*height, x, y, w, h, view_dir, view_up);
		}

	public:
		cam_ray_generator_shirley(uint res_x, uint res_y) : ray_generator(res_x, res_y), aspect(float(res_x)/float(res_y)) {
		}
		void setup(vec3f *new_position, vec3f *new_dir, vec3f *new_up, float new_fovy) {
			position = *new_position;
			dir = *new_dir;
			up = *new_up;
			fovy = new_fovy;
		}
		virtual void generate_rays() {
			for (uint y = 0; y < res_y(); ++y)
				for (uint x = 0; x < res_x(); ++x) {
					*origin(x, y) = position;
					generate_ray_dir_from_cam_parameters(direction(x, y), fovy, aspect, x, y, res_x(), res_y(), &dir, &up);
				}
		}
};

////////////////////

class raytracer {
		ray_generator *raygen;
		class bouncer *bouncer;
		virtual void trace_rays() = 0;
	public:
		raytracer(ray_generator *raygen, class bouncer *bouncer) : raygen(raygen), bouncer(bouncer) {
		}
		virtual void setup_rays() { // potentially uploads ray data to the gpu
		}
		virtual void prepare_bvh_for_tracing() { // potentially uploads the bvh to the gpu? will ich das?
		}
		virtual void trace() {
			raygen->generate_rays();
			do {
				trace_rays();		
				// bouncer->bounce
			} while (bouncer->trace_further_bounces());
		}
};

template<box_t__and__tri_t> class bbvh_tracer : public raytracer {
	public:
		declare_traits_types;
		typedef binary_bvh<box_t, tri_t> bbvh_t;
	protected:
		bbvh_t *bvh;
	public:
		bbvh_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : raytracer(gen, b), bvh(bvh) {
		}
// 		void trace() {
// 		}
};

template<box_t__and__tri_t> class bbvh_direct_is_tracer : public bbvh_tracer<forward_traits> {
	public:
		declare_traits_types;
		typedef binary_bvh<box_t, tri_t> bbvh_t;

		bbvh_direct_is_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : bbvh_tracer<forward_traits>(gen, bvh, b) {
		}
		void trace_rays() {
		}
};

}


////////////////////

// typedef auto function;

auto foo(int x, int y) -> int {
	return x + y;
}

int main() {
	using namespace rta;
	using namespace std;

	typedef int box_t;
	typedef simple_triangle tri_t;
	typedef binary_bvh<box_t, tri_t> bvh_t;
	typedef multi_bvh_sse<box_t, tri_t> mbvh_t;

	cout << "loading object" << endl;
	auto triangle_lists = load_objfile_to_flat_tri_list("/home/kai/render-data/models/bunny-70k.obj");
	
	cout << "initializing rays" << endl;
	cam_ray_generator_shirley crgs(1024, 768);
	vec3f pos = {0,0,0}, dir = {0,0,1}, up = {0,1,0};
	crgs.setup(&pos, &dir, &up, 35);
	crgs.generate_rays();
	
	cout << "building bvh" << endl;
	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::object_median);
	ctor.build(&triangle_lists.front());
	cout << "done" << endl;

	{
		bbvh_constructor_using_binning<bvh_t> ctor;
		flat_triangle_list tris;
		bvh_t *my_bbvh = ctor.build(&tris);
	}

	{
		mbvh_sse_contructor<mbvh_t, bbvh_constructor_using_median<bvh_t>> ctor;
		flat_triangle_list tris;
		bbvh_constructor_using_median<bvh_t> bvhctor(bbvh_constructor_using_median<bvh_t>::object_median);
		mbvh_t *my_mbvh = ctor.build(&tris, bvhctor);
	}


	return 0;
}

/* vim: set foldmethod=marker: */

