#ifndef __RTA_BBVH_H__ 
#define __RTA_BBVH_H__ 

#include "librta/librta.h"
#include "librta/wall-timer.h"
#include "bbvh-node.h"

#include <ostream>

namespace rta {

template<box_t__and__tri_t, typename node = bbvh_node<_box_t>> class binary_bvh : public basic_acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		typedef node node_t;
		typedef typename node_t::link_t link_t;

		std::vector<node_t> nodes;
		std::vector<tri_t> triangles;

		//! take the nodes stored in the array. \attention does so destructively!
		virtual void take_node_array(std::vector<node_t> &n) {
			nodes.swap(n);
		}
		
		//! take the triangles stored in the array. \attention does so destructively!
		virtual void take_triangle_array(std::vector<tri_t> &t) {
			triangles.swap(t);
		}

		tri_t* triangle_ptr() { return &triangles[0]; }
		int triangle_count() { return triangles.size(); }
		
		virtual std::string identification() { return "binary_bvh"; }
		
		virtual void dump_acceleration_structure(const std::string &filename) {
			std::ofstream out(filename.c_str());
			out << "bvh\n" << nodes.size() << "\n";
			for (int i = 0; i < nodes.size(); ++i) {
				out << i << " " << (nodes[i].inner() ? "I " : "L ")
					<< nodes[i].box_min_x() << " " << nodes[i].box_min_y() << " " << nodes[i].box_min_z() << " "
					<< nodes[i].box_max_x() << " " << nodes[i].box_max_y() << " " << nodes[i].box_max_z() << " ";
				if (nodes[i].inner())
					out << nodes[i].split_axis() << " " << nodes[i].left() << " " << nodes[i].right()  << "\n"; 
				else {
					out << nodes[i].elems();
					for (int j = 0; j < nodes[i].elems(); ++j)
						out << " " << nodes[i].tris() + j;
					out << "\n";
				}
			}
		}
		virtual void dump_primitives(const std::string &filename) {
			std::ofstream out(filename.c_str());
			out << "tri\n" << triangles.size() << "\n";
			for (int i = 0; i < triangles.size(); ++i) {
				out << i 
					<< " " << triangles[i].a.x << " " << triangles[i].a.y << " " << triangles[i].a.z
					<< " " << triangles[i].b.x << " " << triangles[i].b.y << " " << triangles[i].b.z
					<< " " << triangles[i].c.x << " " << triangles[i].c.y << " " << triangles[i].c.z
					<< " " << triangles[i].na.x << " " << triangles[i].na.y << " " << triangles[i].na.z
					<< " " << triangles[i].nb.x << " " << triangles[i].nb.y << " " << triangles[i].nb.z
					<< " " << triangles[i].nc.x << " " << triangles[i].nc.y << " " << triangles[i].nc.z << "\n";
			}
		}
};

/*! a binary bvh like \ref binary_bvh which stores the split axis, too.
 *  \note the split axis is not stored compactly, (yet?).
 *  
 *  this structure is intended for use in the contruction of other acceleration structures for
 *  which we wan't to take advantage of the already implented space subdivision.
 */
template<box_t__and__tri_t> class binary_bvh_with_split_axis : public basic_acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		typedef uint32_t link_t;
		struct node : public binary_bvh<forward_traits>::node_t {
			uint axis;
			void split_axis(uint a) { axis = a; }
			uint split_axis()       { return axis; }
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

		tri_t* triangle_ptr() { return &triangles[0]; }
		int triangle_count() { return triangles.size(); }
		
		virtual std::string identification() { return "binary_bvh"; }
};


struct bbvh_no_bias {
	static void apply(uint &begin, uint &mid, uint &end) {
	}
};

template<typename bvh_t_, typename bias_t = bbvh_no_bias> 
class bbvh_constructor_using_median : public basic_acceleration_structure_constructor<typename bvh_t_::box_t, typename bvh_t_::tri_t> {
	public:
		typedef bvh_t_ bvh_t;
		typedef typename bvh_t::node_t node_t;
		typedef typename bvh_t::box_t box_t;
		typedef typename bvh_t::tri_t tri_t;
		typedef typename bvh_t::link_t link_t;
		typedef typename tri_t::vec3_t vec3_t;

	protected:
		std::vector<node_t> nodes;
		const uint max_tris_per_node;
		std::vector<tri_t> triangles;

		enum axes { X = 0, Y, Z };
		
		std::string median_used;
		
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
		typedef int (*qsort_pred_t)(const void *a, const void *b);

		// build o.m. for triangles in [begin, end)
		uint build_om(tri_t *tris, uint begin, uint end) {
			uint id = nodes.size();
			nodes.push_back(node_t());
			node_t *n = &nodes[id];
			box_t box;
			box = compute_aabb<box_t>(tris, begin, end);
			n->volume(box);

			uint elems = end-begin;
			if (elems > max_tris_per_node) 
			{
				vec3_t dists; sub_components_vec3f(&dists, &box.max, &box.min);
				if (dists.x > dists.y)
					if (dists.x > dists.z)  { n->split_axis(X); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_x);	}
					else                    { n->split_axis(Z); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_z);	}
				else
					if (dists.y > dists.z)  { n->split_axis(Y); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_y);	}
					else                    { n->split_axis(Z); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_z);	}

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
			box_t box = compute_aabb<box_t>(tris, begin, end);
			n->volume(box);

			uint elems = end-begin;
			if (elems > max_tris_per_node) 
			{
				vec3_t dists; sub_components_vec3f(&dists, &box.max, &box.min);
				const float_t& (*comp_n)(const vec3_t&) = x_comp;
				if (dists.x > dists.y)
					if (dists.x > dists.z)  { n->split_axis(X); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_x); comp_n=x_comp; }
					else                    { n->split_axis(Z); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_z); comp_n=z_comp; }
				else
					if (dists.y > dists.z)  { n->split_axis(Y); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_y); comp_n=y_comp; }
					else                    { n->split_axis(Z); qsort(tris+begin, end-begin, sizeof(tri_t), (qsort_pred_t)tri_sort_z); comp_n=z_comp; }

				uint mid = compute_spatial_median(tris, begin, end, comp_n);
				if (begin == mid || mid == end)
					mid = begin + (end-begin)/2;
				bias_t::apply(begin, mid, end);

				link_t left  = build_sm(tris, begin, mid); //!!!!!!
				link_t right = build_sm(tris, mid, end);
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
			median_used = median==object_median ? "OM" : "SM";
		}

		bvh_t* build(typename tri_t::input_flat_triangle_list_t *tris) {
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
		virtual std::string identification() { return "bbvh_constructor_using_median: " + median_used; }
};

/*
template<typename bvh_t, typename bias_t = bbvh_no_bias> class bbvh_constructor_using_sah {
	public:
		declare_traits_types;
		bvh_t* build(typename tri_t::input_flat_triangle_list *tris) {
		}
		virtual std::string identification() { return "not implemented, yet."; }
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> class bbvh_constructor_using_binning {
	public:
		declare_traits_types;
		bvh_t* build(typename tri_t::input_flat_triangle_list *tris) {
		}
		virtual std::string identification() { return "not implemented, yet."; }
};
*/

template<box_t__and__tri_t, typename bvh_t_> class bbvh_tracer : public cpu_raytracer<forward_traits> {
	public:
		declare_traits_types;
		typedef bvh_t_ bbvh_t;
	protected:
		/*! A downcast version of basic_raytracer::accel_struct.
		 *  \note If you keep such a pointer, make sure to keep it up to date with the base version 
		 *        so you don't end up tracing a different acceleration structure than set via the base's
		 *        interface. See \ref basic_raytracer::basic_accelration_structure.
		 */
		bbvh_t *bvh;
	public:
		bbvh_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : cpu_raytracer<forward_traits>(gen, b, bvh), bvh(bvh) {
		}
		virtual void acceleration_structure(rta::basic_acceleration_structure<forward_traits> *as) {
			bvh = dynamic_cast<bvh_t_*>(as);
			basic_raytracer<forward_traits>::acceleration_structure(as);
		}
		virtual bool supports_max_t() { return true; }
};

template<box_t__and__tri_t, typename bvh_t_> class bbvh_direct_is_tracer : public bbvh_tracer<forward_traits, bvh_t_> {
	public:
		declare_traits_types;
		typedef bvh_t_ bbvh_t;
		typedef typename bbvh_t::node_t node_t;
		using basic_raytracer<forward_traits>::raygen;
		using cpu_raytracer<forward_traits>::cpu_bouncer;

		bbvh_direct_is_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : bbvh_tracer<forward_traits, bbvh_t>(gen, bvh, b) {
		}
		virtual float trace_rays() {
			wall_time_timer wtt; wtt.start();
			traversal_state<tri_t> state;
			#pragma omp parallel for schedule(dynamic, 32) private(state)
			for (uint y = 0; y < raygen->res_y(); ++y) {
				for (uint x = 0; x < raygen->res_x(); ++x) {
					state.reset(x,y);
					trace_ray(state, raygen->origin(x,y), raygen->direction(x,y));
					cpu_bouncer->save_intersection(x,y,state.intersection);
				}
			}
			float ms = wtt.look();
			return ms;
		}
		void trace_ray(traversal_state<tri_t> &state, const vec3_t *origin, const vec3_t *dir) {
			state.stack[0] = 0;
			state.sp = 0;
			node_t *curr = 0;
			float max_t = raygen->max_t(state.x, state.y);
			state.intersection.t = FLT_MAX;
			while (state.sp >= 0) {
				uint node = state.pop();
				curr = &bbvh_tracer<forward_traits, bbvh_t>::bvh->nodes[node];
				if (curr->inner()) {
					float dist;
					if (intersect_aabb(curr->box, origin, dir, dist))
						if (dist < state.intersection.t && dist <= max_t) {
							state.push(curr->right());
							state.push(curr->left());
						}
				}
				else {
					int elems = curr->elems();
					int offset = curr->tris();
					for (int i = 0; i < elems; ++i) {
						tri_t *t = &bbvh_tracer<forward_traits, bbvh_t>::bvh->triangles[offset+i];
						triangle_intersection<tri_t> is(offset+i);
						if (intersect_tri_opt(*t, origin, dir, is)) {
							if (is.t < state.intersection.t && is.t <= max_t)
								state.intersection = is;
						}
					}
				}
			}
		}
		virtual std::string identification() { return "bbvh_direct_is_tracer"; }
		virtual bbvh_direct_is_tracer* copy() {
			return new bbvh_direct_is_tracer(*this);
		}
};


template<box_t__and__tri_t, typename bvh_t_> class bbvh_child_is_tracer : public bbvh_tracer<forward_traits, bvh_t_> {
	public:
		declare_traits_types;
		typedef bvh_t_ bbvh_t;
		typedef typename bbvh_t::node_t node_t;
		using basic_raytracer<forward_traits>::raygen;
		using cpu_raytracer<forward_traits>::cpu_bouncer;

		bbvh_child_is_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : bbvh_tracer<forward_traits, bvh_t_>(gen, bvh, b) {
		}
		virtual float trace_rays() {
			this->bvh = dynamic_cast<bbvh_t*>(this->accel_struct);
			wall_time_timer wtt; wtt.start();
			traversal_state<tri_t> state;
			#pragma omp parallel for schedule(dynamic, 32) private(state)
			for (uint y = 0; y < raygen->res_y(); ++y) {
				for (uint x = 0; x < raygen->res_x(); ++x) {
					state.reset(x,y);
					trace_ray(state, raygen->origin(x,y), raygen->direction(x,y));
					cpu_bouncer->save_intersection(x,y,state.intersection);
				}
			}
			float ms = wtt.look();
			return ms;
		}
		void trace_ray(traversal_state<tri_t> &state, const vec3_t *origin, const vec3_t *dir) {
			state.stack[0] = 0;
			state.sp = 0;
			node_t *curr = 0;
			float max_t = raygen->max_t(state.x, state.y);
			state.intersection.t = raygen->max_t(state.x, state.y);
			while (state.sp >= 0) {
				uint node = state.pop();
				curr = &bbvh_tracer<forward_traits, bbvh_t>::bvh->nodes[node];
				if (curr->inner()) {
					float dist_l, dist_r;
					uint n_left   = curr->left();
					node_t *left  = &bbvh_tracer<forward_traits, bbvh_t>::bvh->nodes[n_left];
					bool do_left  = intersect_aabb(left->box, origin, dir, dist_l);
					
					uint n_right  = curr->right();
					node_t *right = &bbvh_tracer<forward_traits, bbvh_t>::bvh->nodes[n_right];
					bool do_right = intersect_aabb(right->box, origin, dir, dist_r);

					do_left  = do_left  && dist_l < state.intersection.t && dist_l <= max_t;
					do_right = do_right && dist_r < state.intersection.t && dist_r <= max_t;
					if (do_left && do_right)
						if (dist_l <= dist_r) {
							state.push(n_right);
							state.push(n_left);
						}
						else {
							state.push(n_left);
							state.push(n_right);
						}
					else if (do_left)
						state.push(n_left);
					else if (do_right)
						state.push(n_right);
				}
				else {
					int elems = curr->elems();
					int offset = curr->tris();
					for (int i = 0; i < elems; ++i) {
						tri_t *t = &bbvh_tracer<forward_traits, bbvh_t>::bvh->triangles[offset+i];
						triangle_intersection<tri_t> is(offset+i);
						if (intersect_tri_opt(*t, origin, dir, is)) {
							if (is.t < state.intersection.t && is.t <= max_t)
								state.intersection = is;
						}
					}
				}
			}
		}
		virtual std::string identification() { return "bbvh_child_is_tracer"; }
		virtual bbvh_child_is_tracer* copy() {
			return new bbvh_child_is_tracer(*this);
		}
};



//////////////
//////////////
//////////////
//////////////

template<box_t__and__tri_t> class multi_bvh_sse : public basic_acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		virtual std::string identification() { return "not implemented, yet"; }
};

template<box_t__and__tri_t> class multi_bvh_avx : public basic_acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		virtual std::string identification() { return "not implemented, yet"; }
};


/*
template<typename mbvh_t, typename bbvh_ctor_t> class mbvh_sse_contructor : public basic_acceleration_structure_constructor<typename mbvh_t::box_t, typename mbvh_t::tri_t> {
	public:
		class mbvh_bbvh_building_bias {
		};
		typedef binary_bvh<traits_of(mbvh_t)> bbvh_t;
		mbvh_t* build(typename tri_t::input_flat_triangle_list *tris, bbvh_ctor_t &bbvhctor) {
			bbvh_t *bbvh = bbvhctor.build(tris);
		}
		virtual std::string identification() { return "not implemented, yet"; }
};
*/




}


#endif

