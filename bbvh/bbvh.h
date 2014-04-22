#ifndef __RTA_BBVH_H__ 
#define __RTA_BBVH_H__ 

#include "librta/librta.h"
#include "librta/wall-timer.h"

namespace rta {

template<box_t__and__tri_t> class binary_bvh : public acceleration_structure<forward_traits> {
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
			void left(link_t n)  { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }  //!< erases leaf/inner bit.
			link_t left()        { return type_left_elems>>1; }
			void elems(uint n)   { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }
			uint elems()         { return type_left_elems>>1; }
			//! link to the right.
			void right(link_t n) { right_tris = n; }
			link_t right()       { return right_tris; }
			//! an index into an array of triangles holding elems() successive triangles enclosed by this node.
			void tris(link_t n)  { right_tris = n; }
			uint tris()          { return right_tris; }
			void split_axis(uint a) {} //!< not implemented for default bbvh nodes.
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

/*! a binary bvh like \ref binary_bvh which stores the split axis, too.
 *  \note the split axis is not stored compactly, (yet?).
 *  
 *  this structure is intended for use in the contruction of other acceleration structures for
 *  which we wan't to take advantage of the already implented space subdivision.
 */
template<box_t__and__tri_t> class binary_bvh_with_split_axis : public acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		typedef uint32_t link_t;
		struct node : public binary_bvh<forward_traits>::node {
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
class bbvh_constructor_using_median : public acceleration_structure_constructor<typename bvh_t_::box_t, typename bvh_t_::tri_t> {
	public:
		typedef bvh_t_ bvh_t;
		typedef typename bvh_t::node node_t;
		typedef typename bvh_t::box_t box_t;
		typedef typename bvh_t::tri_t tri_t;
		typedef typename bvh_t::link_t link_t;

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
			n->box = compute_aabb<box_t>(tris, begin, end);

			uint elems = end-begin;
			if (elems > max_tris_per_node) 
			{
				vec3f dists; sub_components_vec3f(&dists, &n->box.max, &n->box.min);
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
			n->box = compute_aabb<box_t>(tris, begin, end);

			uint elems = end-begin;
			if (elems > max_tris_per_node) 
			{
				vec3f dists; sub_components_vec3f(&dists, &n->box.max, &n->box.min);
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
		virtual std::string identification() { return "bbvh_constructor_using_median: " + median_used; }
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> class bbvh_constructor_using_sah {
	public:
		bvh_t* build(flat_triangle_list *tris) {
		}
		virtual std::string identification() { return "not implemented, yet."; }
};

template<typename bvh_t, typename bias_t = bbvh_no_bias> class bbvh_constructor_using_binning {
	public:
		bvh_t* build(flat_triangle_list *tris) {
		}
		virtual std::string identification() { return "not implemented, yet."; }
};

template<box_t__and__tri_t, typename bvh_t_> class bbvh_tracer : public basic_raytracer<forward_traits> {
	public:
		declare_traits_types;
		typedef bvh_t_ bbvh_t;
	protected:
		/*! A downcast version of basic_raytracer::accel_struct.
		 *  \note If you keep such a pointer, make sure to keep it up to date with the base version 
		 *        so you don't end up tracing a different acceleration structure than set via the base's
		 *        interface. See \ref basic_raytracer::accelration_structure.
		 */
		bbvh_t *bvh;
	public:
		bbvh_tracer(ray_generator *gen, bbvh_t *bvh, class bouncer *b) : basic_raytracer<forward_traits>(gen, b, bvh), bvh(bvh) {
		}
		virtual void acceleration_structure(rta::acceleration_structure<forward_traits> *as) {
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
		using basic_raytracer<forward_traits>::cpu_bouncer;

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
			state.intersection.t = raygen->max_t(state.x, state.y);
			while (state.sp >= 0) {
				uint node = state.pop();
				curr = &bbvh_tracer<forward_traits, bbvh_t>::bvh->nodes[node];
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
						tri_t *t = &bbvh_tracer<forward_traits, bbvh_t>::bvh->triangles[offset+i];
						triangle_intersection<tri_t> is(offset+i);
						if (intersect_tri_opt(*t, origin, dir, is)) {
							if (is.t < state.intersection.t)
								state.intersection = is;
						}
					}
				}
			}
			if (!state.intersection.ref)
				state.intersection.t = FLT_MAX;
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
		using basic_raytracer<forward_traits>::cpu_bouncer;

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

					do_left  = do_left  && dist_l < state.intersection.t;
					do_right = do_right && dist_r < state.intersection.t;
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
							if (is.t < state.intersection.t)
								state.intersection = is;
						}
					}
				}
			}
			if (!state.intersection.ref)
				state.intersection.t = FLT_MAX;
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

template<box_t__and__tri_t> class multi_bvh_sse : public acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		virtual std::string identification() { return "not implemented, yet"; }
};

template<box_t__and__tri_t> class multi_bvh_avx : public acceleration_structure<forward_traits> {
	public:
		declare_traits_types;
		virtual std::string identification() { return "not implemented, yet"; }
};


template<typename mbvh_t, typename bbvh_ctor_t> class mbvh_sse_contructor : public acceleration_structure_constructor<typename mbvh_t::box_t, typename mbvh_t::tri_t> {
	public:
		class mbvh_bbvh_building_bias {
		};
		typedef binary_bvh<traits_of(mbvh_t)> bbvh_t;
		mbvh_t* build(flat_triangle_list *tris, bbvh_ctor_t &bbvhctor) {
			bbvh_t *bbvh = bbvhctor.build(tris);
		}
		virtual std::string identification() { return "not implemented, yet"; }
};




}


#endif

