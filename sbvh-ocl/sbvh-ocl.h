#ifndef __SBVH_OCL_H__ 
#define __SBVH_OCL_H__ 

#include "sbvh/sbvh.h"
#include "bbvh-ocl/bbvh-ocl.h"

#include "librta/ocl.h"

#include <typeinfo>

namespace rta {
	namespace ocl {


		template<box_t__and__tri_t, typename sbvh_base_t> class sbvh : public sbvh_base_t {
			public:
				declare_traits_types;
				typedef uint32_t link_t;
				typedef sbvh_base_t parent_t;

				cl::buffer *tri_buffer, *node_buffer;

				sbvh() : tri_buffer(0), node_buffer(0) {
				}
				
				virtual std::string identification() { return "sbvh using ocl buffers (based on " + parent_t::identification() + ")"; }
				void fill_buffers(cl::context &c) {
					const size_t node_size = sizeof(typename parent_t::node);
					if (!tri_buffer)  tri_buffer = new cl::buffer(c, CL_MEM_READ_ONLY, sizeof(tri_t) * this->triangles.size());
					if (!node_buffer) node_buffer = new cl::buffer(c,CL_MEM_READ_ONLY, node_size * this->nodes.size());
					static_assert(node_size == 32, 
					              "The node size is not as expected here. Should this be so, how is it transferred to CL?");
					tri_buffer->copy_to_buffer_blocking(&this->triangles[0], 0, sizeof(tri_t) * this->triangles.size());
					node_buffer->copy_to_buffer_blocking(&this->nodes[0], 0, node_size * this->nodes.size());
					std::cout << "---> " << typeid(this->nodes[0]).name() << std::endl;
// 					typename order_independent_sbvh<forward_traits>::node n = this->nodes[0];
// 					std::cout << "inner? " << n.inner() << std::endl;
// 					std::cout << "children? " << n.children() << std::endl;
// 					std::cout << "code? " << n.raycodes() << std::endl;
				}
		};
		
		template<typename sbvh_ctor_t, typename bbvh_ctor_t, typename bias_t = bbvh_no_bias>
		class sbvh_constructor : public basic_acceleration_structure_constructor<typename sbvh_ctor_t::box_t, typename sbvh_ctor_t::tri_t> {
			public:
				typedef typename sbvh_ctor_t::bvh_t bvh_t;
				typedef typename sbvh_ctor_t::box_t box_t;
				typedef typename sbvh_ctor_t::tri_t tri_t;
				typedef typename sbvh_ctor_t::link_t link_t;

			protected:
				sbvh_ctor_t sbvh_ctor;
				cl::context &ctx;

			public:
				static const uint default_max_tris_per_node = bbvh_ctor_t::default_max_tris_per_node;

				sbvh_constructor(cl::context &c, typename bbvh_ctor_t::median_t median, uint max_tris_per_node = default_max_tris_per_node) 
				: sbvh_ctor(median, max_tris_per_node), ctx(c) {
				}
	
				virtual std::string identification() { return "sbvh ctor to ocl buffer (based on " + sbvh_ctor.identification() + ")"; }

				bvh_t* build(typename tri_t::input_flat_triangle_list_t *tris) {
					std::cout << "ocl sbvh build" << std::endl;
					bvh_t *sbvh = sbvh_ctor.build(tris);
					std::cout << "done" << std::endl;
					std::cout << "fill buffers" << std::endl;
					sbvh->fill_buffers(ctx);
					std::cout << "done" << std::endl;
					return sbvh;
				}
		};

		template<box_t__and__tri_t, typename sbvh_t> class sbvh_oi_gpu_tracer : public bbvh_gpu_tracer<forward_traits, sbvh_t> {
				sbvh_t *sbvh;
			public:
				sbvh_oi_gpu_tracer(rta::ray_generator *gen, sbvh_t *bvh, class bouncer *b, cl::context &c, 
				                const std::string &sourcefile, const std::string &kernelname)
				: bbvh_gpu_tracer<forward_traits, sbvh_t>(gen, bvh, b, c, sourcefile, kernelname), sbvh(bvh) {
				}
				virtual std::string identification() { return "sbvh_tracer using ocl (based on " + bbvh_gpu_tracer<forward_traits, sbvh_t>::identification() + ")"; }
				virtual bool supports_max_t() { return false; }
		};


		template<box_t__and__tri_t, typename sbvh_t> class sbvh_po_gpu_tracer : public bbvh_gpu_tracer<forward_traits, sbvh_t> {
				sbvh_t *sbvh;
			public:
				sbvh_po_gpu_tracer(rta::ray_generator *gen, sbvh_t *bvh, class bouncer *b, cl::context &c, 
				                const std::string &sourcefile, const std::string &kernelname)
				: bbvh_gpu_tracer<forward_traits, sbvh_t>(gen, bvh, b, c, sourcefile, kernelname), sbvh(bvh) {
				}
				virtual std::string identification() { return "sbvh_tracer using ocl (based on " + bbvh_gpu_tracer<forward_traits, sbvh_t>::identification() + ")"; }
				virtual void kernel_args() {
					uint32_t end = sbvh->end();
					this->kernel->add_param(&end, "the intersection output buffer");
				}
				virtual bool supports_max_t() { return false; }
		};




	}
}

#endif
/* vim: set foldmethod=marker: */

