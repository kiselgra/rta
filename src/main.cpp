#include "image.h"
#include "basic_types.h"
#include "cmdline.h"
#include "wall-timer.h"
#include "intersect.h"

#include <libobjloader/default.h>

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


class binary_bvh_facade {
	public:
		// node adding, etc?
		void reserve_node_storage(unsigned int nodes);
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
		typedef node node_t;

		std::vector<node> nodes;
		std::vector<tri_t> triangles;

		//! take the nodes stored in the array. \attention does so destructively!
		void take_node_array(std::vector<node> &n) {
			nodes.swap(n);
		}
		
		//! take the triangles stored in the array. \attention does so destructively!
		void take_triangle_array(std::vector<tri_t> &t) {
			triangles.swap(t);
		}

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
	static void apply(uint &begin, uint &mid, uint &end) {
	}
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> 
class bbvh_constructor_using_median {
	protected:
		typedef typename bvh_t::node node_t;
		typedef typename bvh_t::box_t box_t;
		typedef typename bvh_t::tri_t tri_t;
		typedef typename bvh_t::link_t link_t;

		std::vector<node_t> nodes;
		const uint max_tris_per_node;
		std::vector<tri_t> triangles;

		enum axes { X = 0, Y, Z };
		
		template<unsigned int N> static int tri_sort(const tri_t *a, const tri_t *b) { // optimize: component-cog.
			vec3_t cog = center_of_gravity(*a);
			float cog_a = comp<N>(cog);
			cog = center_of_gravity(*b);
			float cog_b = comp<N>(cog);
			if (cog_a < cog_b) return -1;
			else if (cog_a == cog_b) return 0;
			return 1;
		}
		
		static int tri_sort_x(const tri_t *a, const tri_t *b) { return tri_sort<X>(a, b); }
		static int tri_sort_y(const tri_t *a, const tri_t *b) { return tri_sort<Y>(a, b); }
		static int tri_sort_z(const tri_t *a, const tri_t *b) { return tri_sort<Z>(a, b); }

		// build o.m. for triangles in [begin, end)
		uint build_om(tri_t *tris, uint begin, uint end) {
			uint id = nodes.size();
			nodes.push_back(node_t());
			node_t *n = &nodes[id];
			n->box = compute_aabb<box_t>(tris, begin, end);

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
				bias_t::apply(begin, mid, end);

				link_t left  = build_om(tris, begin, mid);
				link_t right = build_om(tris, mid, end);
				n = &nodes[id]; // refresh pointer!
				n->left(left);
				n->right(right);
				n->make_inner();
			}
			else 
			{
				int idx = triangles.size();
				for (int i = 0; i < elems; ++i)
					triangles.push_back(tris[begin+i]);
				n->elems(elems);
				n->tris(idx);
				n->make_leaf();
			}

			return id;
		}

		uint compute_spatial_median(tri_t *sorted, uint begin, uint end, const float_t&(*comp)(const vec3_t&)) {
			// determine the interval of the centers of the boxes.
			// the bb of the container cannot be used, as the midpoints of the boxes might cluster.
			float min_cog = comp(center_of_gravity(sorted[begin])),
				  max_cog = comp(center_of_gravity(sorted[begin]));
			for (uint it = begin; it != end; ++it) {
				float_t tmp = comp(center_of_gravity(sorted[it]));
				if (tmp < min_cog)	min_cog = tmp;
				if (tmp > max_cog)	max_cog = tmp;
			}

			// divide.
			float_t s = (max_cog - min_cog) / 2 + min_cog;
			uint it = begin;
			while (it != end && comp(center_of_gravity(sorted[it])) < s)		
				++it;
			return it;
		}

		uint build_sm(tri_t *tris, uint begin, uint end) {
			uint id = nodes.size();
			nodes.push_back(node_t());
			node_t *n = &nodes[id];
			n->box = compute_aabb<box_t>(tris, begin, end);

			uint elems = end-begin;
			if (elems > max_tris_per_node) 
			{
				vec3f dists; sub_components_vec3f(&dists, &n->box.max, &n->box.min);
				const float_t& (*comp_n)(const vec3_t&) = x_comp;
				if (dists.x > dists.y)
					if (dists.x > dists.z)	{	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_x); comp_n=x_comp; }
					else                    {	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_z); comp_n=z_comp; }
				else
					if (dists.y > dists.z)	{	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_y); comp_n=y_comp; }
					else                    {	qsort(tris+begin, end-begin, sizeof(tri_t), (int(*)(const void*,const void*))tri_sort_z); comp_n=z_comp; }

				uint mid = compute_spatial_median(tris, begin, end, comp_n);
				if (begin == mid || mid == end)
					mid = begin + (end-begin)/2;
				bias_t::apply(begin, mid, end);

				link_t left  = build_om(tris, begin, mid);
				link_t right = build_om(tris, mid, end);
				n = &nodes[id]; // refresh pointer!
				n->left(left);
				n->right(right);
				n->make_inner();
			}
			else 
			{
				int idx = triangles.size();
				for (int i = 0; i < elems; ++i)
					triangles.push_back(tris[begin+i]);
				n->elems(elems);
				n->tris(idx);
				n->make_leaf();
			}

			return id;
		}
	public:
		enum median_t { object_median, spatial_median };
		median_t median;
		static const uint default_max_tris_per_node = 4;

		bbvh_constructor_using_median(median_t median, uint max_tris_per_node = default_max_tris_per_node) 
		: median(median), max_tris_per_node(max_tris_per_node) {
		}

		bvh_t* build(flat_triangle_list *tris) {
			std::cout << "building bvh for triangle list of " << tris->triangles << " triangles" << std::endl;
			uint root = 0;
			if (median == object_median)
				root = build_om(tris->triangle, 0, tris->triangles);
			else
				root = build_sm(tris->triangle, 0, tris->triangles);

			assert(root == 0);
			
			std::cout << "done building bvh (" << nodes.size() << " nodes, " << triangles.size() << " triangles)" << std::endl;


			bvh_t *bvh = new bvh_t;
			bvh->take_node_array(nodes);
			bvh->take_triangle_array(triangles);

			return bvh;
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

//! state of a ray traversal as executed by a trace thread. not nested in raytracer because of forward-delcaration conflicts.
struct traversal_state {
	enum { stack_size = 128 };
	uint stack[stack_size];
	int sp; //!< -1 -> stack empty.
	uint x, y;
	triangle_intersection intersection;
	void reset(uint x, uint y) {
		this->x = x, this->y = y;
		sp = 0;
		intersection.reset();
	}
	uint pop() { return stack[sp--]; }
	void push(uint node) { stack[++sp] = node; }
};

////////////////////

class bouncer { // sequential calls to raytrace
	public:
		virtual void bounce() = 0;
		virtual bool trace_further_bounces() = 0;
		virtual void new_pass() {}
};

/*! commonly, cpu ray bouncers will require the last triangle intersections.
 *  the ray data is supposed to be stored in the ray generator's structure until the bouncer replaces it.
 */
class cpu_ray_bouncer : public bouncer {
	protected:
		image<triangle_intersection, 1> last_intersection;
	public:
		cpu_ray_bouncer(uint w, uint h) : last_intersection(w, h) {
		}
		void save_intersection(uint x, uint y, const triangle_intersection &ti) {
			last_intersection.pixel(x,y) = ti;
		}
};

class primary_intersection_collector : public cpu_ray_bouncer {
	public:
		primary_intersection_collector(uint w, uint h) : cpu_ray_bouncer(w,h) {
		}
		virtual void bounce_ray(const traversal_state &state, vec3_t *origin, vec3_t *dir) {
		}
		virtual bool trace_further_bounces() {
			return false;
		}
};

class binary_png_tester : public cpu_ray_bouncer {
		image<unsigned char, 3> res;
	public:
		binary_png_tester(uint w, uint h) : cpu_ray_bouncer(w,h), res(w,h) {
			for (int y = 0; y < h; ++y)
				for (int x = 0; x < w; ++x)
					for (int c = 0; c < 3; ++c)
						res.pixel(x,y,c) = 0;
		}
// 		virtual void bounce_ray(const traversal_state &state, vec3_t *origin, vec3_t *dir) {
// 			res.pixel(state.x,state.y,1) = (state.intersection.valid() ? 255 : 0);
// 		}
		virtual void bounce() {
			for (int y = 0; y < res.h; ++y)
				for (int x = 0; x < res.w; ++x)
					res.pixel(x,y,1) = last_intersection.pixel(x,y).valid() ? 255 : 0;
		}
		virtual bool trace_further_bounces() {
			return false;
		}
		void save() {
			res.save_png("/tmp/blub.png");
		}
};

////////////////////

class ray_generator {
	public:
		image<vec3f, 2> raydata;
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
// 		struct trace_thread {
// 		};
	protected:
		ray_generator *raygen;
		class bouncer *bouncer;
		cpu_ray_bouncer *cpu_bouncer;
		virtual void trace_rays() = 0;
	public:
		raytracer(ray_generator *raygen, class bouncer *bouncer) : raygen(raygen), bouncer(bouncer), cpu_bouncer(dynamic_cast<cpu_ray_bouncer*>(bouncer)) {
		}
		virtual void setup_rays() { // potentially uploads ray data to the gpu
		}
		virtual void prepare_bvh_for_tracing() { // potentially uploads the bvh to the gpu? will ich das?
		}
		virtual void trace() {
			std::cout << "setting up rays for first bounce" << std::endl;
			raygen->generate_rays();
			do {
				std::cout << "tracing rays" << std::endl;
				trace_rays();
				bouncer->bounce();
			} while (bouncer->trace_further_bounces());
			std::cout << "trace done" << std::endl;
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
};

template<box_t__and__tri_t> class bbvh_direct_is_tracer : public bbvh_tracer<forward_traits> {
	public:
		declare_traits_types;
		typedef binary_bvh<box_t, tri_t> bbvh_t;
		typedef typename bbvh_t::node_t node_t;
		using raytracer::raygen;
		using raytracer::cpu_bouncer;

		bbvh_direct_is_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : bbvh_tracer<forward_traits>(gen, bvh, b) {
		}
		virtual void trace_rays() {
			std::cout << "trace rays" << std::endl;
			wall_time_timer wtt; wtt.start();
			traversal_state state;
			for (uint y = 0; y < raygen->res_y(); ++y) {
				for (uint x = 0; x < raygen->res_x(); ++x) {
					state.reset(x,y);
					trace_ray(state, raygen->origin(x,y), raygen->direction(x,y));
					cpu_bouncer->save_intersection(x,y,state.intersection);
				}
			}
			float ms = wtt.look();
			cout << "took " << ms << " ms." << endl;
		}
		void trace_ray(traversal_state &state, const vec3_t *origin, const vec3_t *dir) {
			state.stack[0] = 0;
			state.sp = 0;
			node_t *curr = 0;
			while (state.sp >= 0) {
				uint node = state.pop();
				curr = &bbvh_tracer<forward_traits>::bvh->nodes[node];
				if (curr->inner()) {
					float dist;
					if (intersect_aabb(curr->box, origin, dir, dist))
						if (dist < state.intersection.t) {
							state.push(curr->right());
							state.push(curr->left());
						}
				}
				else {
					int elems = curr->elems();
					int offset = curr->tris();
					for (int i = 0; i < elems; ++i) {
						triangle_intersection is;
						if (intersect_tri_opt(bbvh_tracer<forward_traits>::bvh->triangles[offset+i], origin, dir, is)) {
							if (is.t < state.intersection.t)
								state.intersection = is;
						}
					}
				}
			}
		}
};

}


////////////////////

// typedef auto function;

#define defun auto
auto foo(int x, int y) -> int {
	return x + y;
}

defun foo2 (int x, int y) -> int {
	return x + y;
}

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
	
	uint res_x = 512, 
		 res_y = 512;
	cout << "initializing rays" << endl;
	cam_ray_generator_shirley crgs(res_x, res_y);
	crgs.setup(&cmdline.pos, &cmdline.dir, &cmdline.up, 35);
	crgs.generate_rays();
	
	cout << "building bvh" << endl;
// 	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::object_median);
	bbvh_constructor_using_median<bvh_t> ctor(bbvh_constructor_using_median<bvh_t>::spatial_median);
	bvh_t *bvh = ctor.build(&triangle_lists.front());
	cout << "trace" << endl;
	binary_png_tester coll(res_x, res_y);
	bbvh_direct_is_tracer<box_t, tri_t> rt(&crgs, bvh, &coll);
	rt.trace();
	cout << "save" << endl;
	coll.save();
	cout << "done" << endl;

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

