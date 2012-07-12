#ifndef __SBVH_H__ 
#define __SBVH_H__ 

#include "bbvh/bbvh.h"

namespace rta {

	/*! stackless bvh in the general form requiering back-traversal.
	 *  can be used with bbvh ray tracer, too.
	 *  i.e. this is not really a stackless bvh ;)
	 *
	 *  the node's incorporate function is used to transfer additional data of the base bvh node
	 *  into a "derived" stackless node.
	 *
	 *  \note derived is not meant in the object oriented sense!
	 *  the interface is merely syntactic, i.e. this is the template function you would define
	 *  in a node class you can subsitute for this node type as template parameter to the sbvh ctor.
	 *
	 *  you may also derive, but take care that no virtual stuff is involved, otherwise things may 
	 *  go wrong becase we rely on the syntactic interface, only.
	 *
	 *  furthermore, this is a requirement for node types constructed by the \ref sbvh_constructor.
	 *  if you generate your nodes based on this ctor's output such a function is, of course, not 
	 *  required.
	 *
	 */
	template<box_t__and__tri_t> class stackless_bvh : public acceleration_structure<forward_traits> {
		public:
			declare_traits_types;
			typedef uint32_t link_t;

			struct node {
				link_t type_parent;         // contains: bit0=inner/leaf >>1=(parent_link|tris-in-leaf)
				link_t children_tris;       // contains: left-link(right-link: ll+1)/link-to-tris+tris
				box_t box;

				// parent_link contains
				// 1. type information,
				bool inner()            { return (type_parent&0x01)==1; }
				void make_inner()       { type_parent|= 0x01; }
				void make_leaf()        { type_parent&= (~1); }
				// 2. parent link
				void parent(link_t n)   { type_parent = ((type_parent&(0x1)) | (n<<1)); }
				link_t parent()         { return type_parent>>1; }
				// child_link contains
				// 1. link to the two children (the right child has index+1)
				void children(link_t n) { children_tris = n; }
				link_t children()       { return children_tris; }
				link_t left()           { return children_tris; }
				link_t right()          { return children_tris+1; }
				// 2. link to the triangle data
				void tris(link_t n)     { children_tris = ((children_tris&(255)) | (n<<8)); }
				link_t tris()           { return children_tris>>8; }
				// 3. triangles in leaf
				void elems(link_t n)    { children_tris = ((children_tris&(~255)) | n); }
				link_t elems()          { return children_tris&255; }
				template<typename base_node> void incorporate(base_node *n) {}
			};
			typedef node node_t;

			std::vector<node> nodes;
			std::vector<tri_t> triangles;
		
			tri_t* triangle_ptr() { return &triangles[0]; }
			int triangle_count() { return triangles.size(); }
		
			virtual std::string identification() { return "generic_stackless_binary_bvh"; }
			
			//! take the triangles stored in the array. \attention does so destructively!
			void take_triangle_array(std::vector<tri_t> &t) {
				triangles.swap(t);
			}
	};

	/*! ctor for the general sbvh version. see stackless_bvh. */
	template<typename sbvh_t, typename bvh_ctor_t, typename bias_t = bbvh_no_bias> 
	class sbvh_constructor : public acceleration_structure_constructor<typename sbvh_t::box_t, typename sbvh_t::tri_t> {
		public:
			typedef typename bvh_ctor_t::bvh_t bbvh_t;
			typedef sbvh_t bvh_t;
			typedef typename sbvh_t::box_t box_t;
			typedef typename sbvh_t::tri_t tri_t;
			typedef typename sbvh_t::link_t link_t;

		protected:
			bvh_ctor_t bbvh_ctor;

			int next_free_node;

			void convert_node(bbvh_t *binary_bvh, sbvh_t *stackless_bvh, int bvh_idx, int to, int parent) {
				typename bbvh_t::node_t *bvh_node = &binary_bvh->nodes[bvh_idx];
				typename sbvh_t::node_t *stackless_node = &stackless_bvh->nodes[to];

				stackless_node->box = bvh_node->box;
				stackless_node->parent(parent);
				stackless_node->incorporate(bvh_node);
				if (bvh_node->inner()) {
					int left = next_free_node;
					int right = left + 1;
					next_free_node += 2;
					stackless_node->children(left);
					stackless_node->make_inner();

					convert_node(binary_bvh, stackless_bvh, bvh_node->left(), left, to);
					convert_node(binary_bvh, stackless_bvh, bvh_node->right(), right, to);
				}
				else {
					stackless_node->elems(bvh_node->elems());
					stackless_node->tris(bvh_node->tris());
					stackless_node->make_leaf();
				}
			}

		public:
			static const uint default_max_tris_per_node = bvh_ctor_t::default_max_tris_per_node;

			sbvh_constructor(typename bvh_ctor_t::median_t median, uint max_tris_per_node = default_max_tris_per_node) 
			: bbvh_ctor(median, max_tris_per_node) {
			}
	
			virtual std::string identification() { return "stackless bvh ctor (based on " + bbvh_ctor.identification() + ")"; }
		
			sbvh_t* build(flat_triangle_list *tris) {
				bbvh_t *bbvh = bbvh_ctor.build(tris);
				sbvh_t *stackless_bvh = new sbvh_t;

				stackless_bvh->nodes.resize(bbvh->nodes.size());

				next_free_node = 1;
				convert_node(bbvh, stackless_bvh, 0, 0, 0);
				stackless_bvh->nodes[0].parent(0);

				stackless_bvh->take_triangle_array(bbvh->triangles);

				return stackless_bvh;
			}
	};

	/*! preoder sbvh.
	 *  this is the classical stackless bvh implementation with direction dependent performance.
	 *  the creation of the structure is split into three parts.
	 *  1. we create a generic bvh (\ref sbvh_constructor), but with an extended strucutre (\ref fat_sbvh) for the bbvh nodes.
	 *     this enables us to annotate the bbvh nodes with the indices the corresponding (reordered) sbvh nodes are stored at.
	 *     \question i'm not sure why the other fields are not shared. maybe convenience?
	 *  2. we convert the fat_sbvh nodes to preorder_stackless_bvh::nodes, the memory layout as dictated by the stackless method.
	 *
	 *  this bvh has to be traversed with a traverser able to handle the structure (not like the generic (s)bvh above,
	 *  which can be traversed with the standard bbvh traverser. \ref preorder_sbvh_tracer.
	 */
	
	/*! stackless bvh stored in pre order, for classical and 8sbvh traversal. */
	template<box_t__and__tri_t> class preorder_stackless_bvh : public acceleration_structure<forward_traits> {
		public:
			declare_traits_types;
			typedef uint32_t link_t;

			//! this is exactly the same as binary_bvh::node, ignoring renaming of right->skip. maybe try to do something clever here?
			struct node {
				link_t type_elems; // contains: bit0=inner/leaf >>1=(left-child/tris-in-leaf)
				link_t skip_tris;       // contains: skip-child/link-to-tris
				box_t box;
				bool inner()         { return (type_elems&0x01)==1; }
				void make_inner()    { type_elems |= 0x01; }
				void make_leaf()     { type_elems &= (~1); }
				void elems(uint n)   { type_elems = ((type_elems&(~1)) | (n<<1)); }
				uint elems()         { return type_elems>>1; }
				//! link to the skip.
				void skip(link_t n) { skip_tris = n; }
				link_t skip()       { return skip_tris; }
				//! an index into an array of triangles holding elems() successive triangles enclosed by this node.
				void tris(link_t n)  { skip_tris = n; }
				uint tris()          { return skip_tris; }
			};

			typedef node node_t;

			std::vector<node> nodes;
			std::vector<tri_t> triangles;
		
			tri_t* triangle_ptr() { return &triangles[0]; }
			int triangle_count() { return triangles.size(); }
		
			virtual std::string identification() { return "preorder_stackless_binary_bvh"; }
			
			//! take the triangles stored in the array. \attention does so destructively!
			void take_triangle_array(std::vector<tri_t> &t) {
				triangles.swap(t);
			}

			link_t end() { return nodes.size(); }
	};

	
	template<typename sbvh_t, typename bvh_ctor_t, typename bias_t = bbvh_no_bias> 
	class sbvh_preorder_constructor : public acceleration_structure_constructor<typename sbvh_t::box_t, typename sbvh_t::tri_t> {
		public:
			typedef typename sbvh_t::box_t box_t;
			typedef typename sbvh_t::tri_t tri_t;
			typedef sbvh_t bvh_t;
		private:
			/*! we use this fat implementation of the template interface defined in stackless_bvh<>
			 *  to construct a sbvh (via the generic sbvh ctor) which we can extend to insert a preoder
			 *  numbering to ease the conversion to a preoder sbvh.
			 */
			class fat_sbvh : public acceleration_structure<box_t, tri_t> {
				public:
					typedef uint32_t link_t;
					struct node {
						bool inner_;
						link_t parent_, left_, right_, tris_, elems_, id;
						box_t box;

						bool inner()            { return inner_; }
						void make_inner()       { inner_ = true; }
						void make_leaf()        { inner_ = false; }
						void parent(link_t n)   { parent_ = n; }
						link_t parent()         { return parent_; }
						void children(link_t n) { left_ = n; right_ = n+1; }
						link_t children()       { return left_; }
						link_t left()           { return left_; }
						link_t right()          { return right_; }
						void tris(link_t n)     { tris_ = n; }
						link_t tris()           { return tris_; }
						void elems(link_t n)    { elems_ = n; }
						link_t elems()          { return elems_; }
						template<typename base_node> void incorporate(base_node *n) {}
					};
					typedef node node_t;

					std::vector<node> nodes;
					std::vector<tri_t> triangles;

					tri_t* triangle_ptr() { return &triangles[0]; }
					int triangle_count() { return triangles.size(); }

					virtual std::string identification() { return "temporary bvh for conversion to preoder sbvh"; }

					//! take the triangles stored in the array. \attention does so destructively!
					void take_triangle_array(std::vector<tri_t> &t) { triangles.swap(t); }
			};


		public:
			typedef typename bvh_ctor_t::bvh_t bbvh_t;
			typedef typename sbvh_t::link_t link_t;
			typedef stackless_bvh<typename sbvh_t::box_t, typename sbvh_t::tri_t> gen_sbvh_t;
			typedef sbvh_constructor<fat_sbvh, bvh_ctor_t, bias_t> fat_sbvh_ctor_t;

		protected:
			fat_sbvh_ctor_t fat_sbvh_ctor;

			int next_id;
			link_t exit_node;
			/*! augment the fat sbvh with the preoder numbers which correspond to the array positions 
			 *  of those nodes in the preorder sbvh.
			 */
			void set_preorder_ids(fat_sbvh *f, int node) {
				typename fat_sbvh::node *n = &f->nodes[node];
				n->id = next_id++;
				if (n->inner()) {
					set_preorder_ids(f, n->left());
					set_preorder_ids(f, n->right());
				}
			}
			
			/*! this is the basic scheme of the skip pointer determination from the generic sbvh.
			 */
			uint32_t skip_of(fat_sbvh *f, int node) {
				uint32_t x = skip_of(f, node, 0);
				if (x == node) {
					std::cout << "ops" << std::endl;
				}
				if (x == 0) {
					std::cout << "ops2 (" << node << ")" << std::endl;
				}
				return x;
			}
			uint32_t skip_of(fat_sbvh *f, int node, int) {
				if (node == 0)
					return exit_node;
				typename fat_sbvh::node *n = &f->nodes[node];
				typename fat_sbvh::node *p = &f->nodes[n->parent()];
				typename fat_sbvh::node *r = &f->nodes[p->right()];
				if (r == n)
					return skip_of(f, n->parent(), 0);
				return p->right();
			}

			void convert_node(fat_sbvh  *base_sbvh, sbvh_t *stackless_bvh, int base_sbvh_idx, int to) {
				typename fat_sbvh::node_t *fat_node = &base_sbvh->nodes[base_sbvh_idx];
				typename sbvh_t::node_t *stackless_node = &stackless_bvh->nodes[to];

				stackless_node->box = fat_node->box;
				if (fat_node->inner()) {
					uint32_t skip = skip_of(base_sbvh, base_sbvh_idx);    // get skip pointer in the original structure
					if (skip < exit_node)
						skip = base_sbvh->nodes[skip].id;               // get skip pointer in preorder
					stackless_node->skip(skip);
					stackless_node->make_inner();

					convert_node(base_sbvh, stackless_bvh, fat_node->left(), base_sbvh->nodes[fat_node->left()].id);
					convert_node(base_sbvh, stackless_bvh, fat_node->right(), base_sbvh->nodes[fat_node->right()].id);
				}
				else {
					stackless_node->elems(fat_node->elems());
					stackless_node->tris(fat_node->tris());
					stackless_node->make_leaf();
				}
			}

		public:
			static const uint default_max_tris_per_node = bvh_ctor_t::default_max_tris_per_node;

			sbvh_preorder_constructor(typename bvh_ctor_t::median_t median, uint max_tris_per_node = default_max_tris_per_node) 
			: fat_sbvh_ctor(median, max_tris_per_node) {
			}
	
			virtual std::string identification() { return "preorder stackless bvh ctor (based on " + fat_sbvh_ctor.identification() + ")"; }
		
			sbvh_t* build(flat_triangle_list *tris) {
				std::cout << "preorder ctor" << std::endl;
				fat_sbvh *fbvh = fat_sbvh_ctor.build(tris);
				std::cout << "built fat sbvh" << std::endl;
				sbvh_t *stackless_bvh = new sbvh_t;

				stackless_bvh->nodes.resize(fbvh->nodes.size());

				next_id = 0;
				set_preorder_ids(fbvh, 0);
				exit_node = next_id;

				std::cout << "converting nodes" << std::endl;
				convert_node(fbvh, stackless_bvh, 0, 0);

				std::cout << "copying triangles" << std::endl;
				stackless_bvh->take_triangle_array(fbvh->triangles);

				return stackless_bvh;
			}
	};

	template<box_t__and__tri_t, typename sbvh_t_> class preorder_sbvh_tracer : public basic_raytracer<forward_traits> {
		public:
			declare_traits_types;
			typedef sbvh_t_ sbvh_t;
			typedef typename sbvh_t::node_t node_t;
			using basic_raytracer<forward_traits>::raygen;
			using basic_raytracer<forward_traits>::cpu_bouncer;
			sbvh_t *sbvh;

			preorder_sbvh_tracer(ray_generator *gen, sbvh_t *bvh, class bouncer *b) : basic_raytracer<forward_traits>(gen, b, bvh), sbvh(bvh) {
			}
			virtual float trace_rays() {
				wall_time_timer wtt; wtt.start();
				traversal_state<tri_t> state;
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
				uint32_t curr = 0;
				uint32_t end = sbvh->end();
				node_t *node = 0;
				while (curr < end) {
					node = &sbvh->nodes[curr];
					if (node->inner()) {
						float dist = 0;
						bool hit = intersect_aabb(node->box, origin, dir, dist);
						if (hit && dist < state.intersection.t)
							++curr;
						else
							curr = node->skip();
					}
					else {
						int elems = node->elems();
						int offset = node->tris();
						for (int i = 0; i < elems; ++i) {
							tri_t *t = &sbvh->triangles[offset+i];
							triangle_intersection<tri_t> is(offset+i);
							if (intersect_tri_opt(*t, origin, dir, is)) {
								if (is.t < state.intersection.t)
									state.intersection = is;
							}
						}
						++curr;
					}
				}
			}
			virtual std::string identification() { return "preorder sbvh tracer"; }
	};


	/*! stackless, direction independent bvh.
	 * 	there is no need for a special o.i. ctor as the \ref sbvh_constructor can be used to construct this kind of sbvh
	 * 	(since it implements the syntactical interface of \ref stackless_bvh).
	 * 	the only augmentation required is to incorporate the split axis information into ray direction codes,
	 * 	which is done at construction time per node (in the incorporate function).
	 *
	 * 	\note only the information about the split axis is used. therefore the construction relies on the fact
	 * 	that for each split the volume "more negative" according to the split axis is stored as the left child
	 * 	in the tree.
	 */

	template<box_t__and__tri_t> class order_independent_sbvh : public acceleration_structure<forward_traits> {
		public:
			declare_traits_types;
			typedef uint32_t link_t;

			typedef uint32_t code_t;
			code_t compute_ray_code(const vec3_t &dir) {
				code_t code = 0;
				if (x_comp(dir) < 0) code += 1;
				if (y_comp(dir) < 0) code += 2;
				if (z_comp(dir) < 0) code += 4;
				return code;
			}

			struct node : public stackless_bvh<forward_traits>::node {
				// we just recycle the 8 bit used to store the triangle count
				void raycodes(uint8_t c)   { this->children_tris = ((this->children_tris&(~255)) | (link_t)c); }
				uint8_t raycodes()         { return (uint8_t)(this->children_tris&255); }
				// we'll have to fix this, too.
				void children(link_t n) { this->children_tris = (this->children_tris&255) | (n<<8); }
				link_t children()       { return this->children_tris>>8; }
				link_t left()           { return this->children_tris>>8; }
				link_t right()          { return (this->children_tris>>8)+1; }

				//! this is done at construction time, only.
				template<typename base_node> void incorporate(base_node *n) {
					uint axis = n->split_axis(); // 0=X ...
					uint8_t code;
					if (axis == 0)      code = 0xaa;
					else if (axis == 1) code = 0xcc;
					else                code = 0xf0;
					raycodes(code);
				}
			};
			typedef node node_t;

			std::vector<node> nodes;
			std::vector<tri_t> triangles;
		
			tri_t* triangle_ptr() { return &triangles[0]; }
			int triangle_count() { return triangles.size(); }
		
			virtual std::string identification() { return "order_independent_stackless_binary_bvh"; }
			
			//! take the triangles stored in the array. \attention does so destructively!
			void take_triangle_array(std::vector<tri_t> &t) {
				triangles.swap(t);
			}
	};

	/*! sbvh traverser for the \ref order_independent_sbvh. */
	template<box_t__and__tri_t, typename sbvh_t_> class order_independent_sbvh_tracer : public basic_raytracer<forward_traits> {
		public:
			declare_traits_types;
			typedef sbvh_t_ sbvh_t;
			typedef typename sbvh_t::node_t node_t;
			using basic_raytracer<forward_traits>::raygen;
			using basic_raytracer<forward_traits>::cpu_bouncer;
			sbvh_t *sbvh;

			order_independent_sbvh_tracer(ray_generator *gen, sbvh_t *bvh, class bouncer *b) : basic_raytracer<forward_traits>(gen, b, bvh), sbvh(bvh) {
			}
			virtual float trace_rays() {
				wall_time_timer wtt; wtt.start();
				traversal_state<tri_t> state;
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
			uint next(node_t *node, const uint8_t code) {
				uint8_t c = 1<<code;
				c = (node->raycodes() & c) >> code;
				return node->children() + c;
			}
			// TODO: check for performance bug? unnecessary stuff... {{{
			uint skip_(node_t *node, int node_id, const uint8_t code) {
				std::cout << "NOT this one" << std::endl;
				uint parent_id = node->parent();
				node_t *parent = &sbvh->nodes[parent_id];
				if (parent_id == node_id)
					return 0;
				
				uint8_t c = 1<<code;
				c = (node->raycodes() & c) >> code; // bug: parent->rc...

				uint parent_right = parent->children() + 1 - c;
				if (parent_right == node_id)
					return skip_(parent, parent_id, code);
				else
					return parent_right;
			} // }}}
			uint skip(node_t *node, int node_id, const uint8_t code) {
				if (node_id == 0) return 0;
				uint parent_id = node->parent();
				node_t *parent = &sbvh->nodes[parent_id];

				uint8_t c = 1<<code;
				c = (parent->raycodes() & c) >> code;

				uint parent_right = parent->children() + 1 - c;
				if (parent_right == node_id)
					return skip(parent, parent_id, code);
				else
					return parent_right;
			}
			void trace_ray(traversal_state<tri_t> &state, const vec3_t *origin, const vec3_t *dir) {
				uint curr = 0;
				node_t *node = 0;
				uint8_t ray_code = sbvh->compute_ray_code(*dir);
				
				while (1) {
					node = &sbvh->nodes[curr];
					if (node->inner()) {
						float dist = 0;
						bool hit = intersect_aabb(node->box, origin, dir, dist);
						if (hit && dist < state.intersection.t)
							curr = next(node, ray_code);
						else
							curr = skip(node, curr, ray_code);
					}
					else {
						int elems = node->elems();
						int offset = node->tris();
						for (int i = 0; i < elems; ++i) {
							tri_t *t = &sbvh->triangles[offset+i];
							triangle_intersection<tri_t> is(offset+i);
							if (intersect_tri_opt(*t, origin, dir, is)) {
								if (is.t < state.intersection.t)
									state.intersection = is;
							}
						}
						curr = skip(node, curr, ray_code);
					}
					if (curr == 0)
						break;
				}
			}
			virtual std::string identification() { return "order independent sbvh tracer"; }
	};


}

#endif

/* vim: set foldmethod=marker: */

