#ifndef __SBVH_H__ 
#define __SBVH_H__ 

#include "bbvh/bbvh.h"

namespace rta {

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
			};
			typedef node node_t;

			std::vector<node> nodes;
			std::vector<tri_t> triangles;
		
			tri_t* triangle_ptr() { return &triangles[0]; }
			int triangle_count() { return triangles.size(); }
		
			virtual std::string identification() { return "stackless_binary_bvh"; }
	};

	template<typename sbvh_t, typename bvh_ctor_t, typename bias_t = bbvh_no_bias> 
	class sbvh_constructor : public acceleration_structure_constructor<typename sbvh_t::box_t, typename sbvh_t::tri_t> {
		protected:
			typedef typename bvh_ctor_t::bvh_t bbvh_t;
	// 		typedef typename bvh_t::node node_t;
			typedef typename sbvh_t::box_t box_t;
			typedef typename sbvh_t::tri_t tri_t;
			typedef typename sbvh_t::link_t link_t;

			bvh_ctor_t bbvh_ctor;

			int next_free_node;

			void convert_node(bbvh_t *binary_bvh, sbvh_t *stackless_bvh, int bvh_idx, int to) {
				typename bbvh_t::node_t *bvh_node = &binary_bvh->nodes[bvh_idx];
				typename sbvh_t::node_t *stackless_node = &stackless_bvh->nodes[to];

				stackless_node->box = bvh_node->box;
				stackless_node->parent(to);
				if (bvh_node->inner()) {
					int left = next_free_node;
					int right = left + 1;
					next_free_node += 2;
					stackless_node->children(left);

					convert_node(binary_bvh, stackless_bvh, bvh_node->left(), left);
					convert_node(binary_bvh, stackless_bvh, bvh_node->right(), right);
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
				convert_node(bbvh, stackless_bvh, 0, 0);
			}
	};

}

#endif

