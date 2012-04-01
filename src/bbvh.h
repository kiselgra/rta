#ifndef __RTA_BBVH_H__ 
#define __RTA_BBVH_H__ 


namespace rta {


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


#endif

