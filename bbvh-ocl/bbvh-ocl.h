#ifndef __BBVH_OCL_H__ 
#define __BBVH_OCL_H__ 

#include "bbvh/bbvh.h"

#include "librta/ocl.h"

namespace rta {
	namespace ocl {
		
		template<box_t__and__tri_t> class binary_bvh : public rta::binary_bvh<forward_traits> {
			public:
				declare_traits_types;
				typedef uint32_t link_t;
				typedef rta::binary_bvh<forward_traits> parent_t;

				cl::buffer *tri_buffer, *node_buffer;

				binary_bvh() : tri_buffer(0), node_buffer(0) {
				}
				
				virtual std::string identification() { return "bbvh using ocl buffers (based on " + parent_t::identification() + ")"; }
				void fill_buffers(cl::context &c) {
					const size_t node_size = sizeof(typename parent_t::node);
					if (!tri_buffer)  tri_buffer = new cl::buffer(c, CL_MEM_READ_ONLY, sizeof(tri_t) * this->triangles.size());
					if (!node_buffer) node_buffer = new cl::buffer(c,CL_MEM_READ_ONLY, node_size * this->nodes.size());
					static_assert(node_size == 32, 
					              "The node size is not as expected here. Should this be so, how is it transferred to CL?");
					tri_buffer->copy_to_buffer_blocking(&this->triangles[0], 0, sizeof(tri_t) * this->triangles.size());
					node_buffer->copy_to_buffer_blocking(&this->nodes[0], 0, node_size * this->nodes.size());
				}
		};

		template<typename bvh_ctor_t, typename bias_t = bbvh_no_bias>
		class bbvh_constructor : public acceleration_structure_constructor<typename bvh_ctor_t::box_t, typename bvh_ctor_t::tri_t> {
			public:
				typedef typename bvh_ctor_t::bvh_t bbvh_t;
				typedef typename bvh_ctor_t::box_t box_t;
				typedef typename bvh_ctor_t::tri_t tri_t;
				typedef typename bvh_ctor_t::link_t link_t;

			protected:
				bvh_ctor_t bbvh_ctor;
				cl::context &ctx;

			public:
				static const uint default_max_tris_per_node = bvh_ctor_t::default_max_tris_per_node;

				bbvh_constructor(cl::context &c, typename bvh_ctor_t::median_t median, uint max_tris_per_node = default_max_tris_per_node) 
				: bbvh_ctor(median, max_tris_per_node), ctx(c) {
				}
	
				virtual std::string identification() { return "bbvh ctor to ocl buffer (based on " + bbvh_ctor.identification() + ")"; }

				bbvh_t* build(flat_triangle_list *tris) {
					bbvh_t *bbvh = bbvh_ctor.build(tris);
					bbvh->fill_buffers(ctx);
					return bbvh;
				}
		};

		template<box_t__and__tri_t, typename bvh_t_> 
		class bbvh_gpu_tracer : public basic_raytracer<forward_traits>, public ocl::raytracer_ocl_addon<forward_traits> {
			public:
				typedef bvh_t_ bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				using basic_raytracer<forward_traits>::raygen;
				using basic_raytracer<forward_traits>::cpu_bouncer;
				// required because of template base:
				using ocl::raytracer_ocl_addon<forward_traits>::kernel;
				using ocl::raytracer_ocl_addon<forward_traits>::opencl;
				using ocl::raytracer_ocl_addon<forward_traits>::rba;
				using ocl::raytracer_ocl_addon<forward_traits>::bba;
				using ocl::raytracer_ocl_addon<forward_traits>::stack;
				bbvh_t *bbvh;
		
				bbvh_gpu_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b, cl::context &c, 
				                const std::string &sourcefile, const std::string &kernelname)
				: basic_raytracer<forward_traits>(gen, b, bvh), 
				  ocl::raytracer_ocl_addon<forward_traits>(c, sourcefile, kernelname),
				  bbvh(bvh) {
					if (b) ray_bouncer(b);
					if (gen) ray_generator(gen);
				}

				virtual void ray_bouncer(bouncer *rb) {
					basic_raytracer<forward_traits>::ray_bouncer(rb);
					raytracer_ocl_addon<forward_traits>::ray_bouncer(rb);
				}
				virtual void ray_generator(rta::ray_generator *rg) { 
					basic_raytracer<forward_traits>::ray_generator(rg);
					raytracer_ocl_addon<forward_traits>::ray_generator(rg);
				}
			
				virtual void kernel_args() {}

				virtual float trace_rays() {
					clFinish(opencl.command_queue);
					wall_time_timer wtt; wtt.start();

					kernel->start_params();
					kernel->add_param(rba->buffer(), "the ray buffer");
					kernel->add_param(bbvh->node_buffer, "the bvh nodes");
					kernel->add_param(bbvh->tri_buffer, "the triangle array");
					kernel->add_param(stack->buffer(), "the stack");
					kernel->add_param(bba->buffer(), "the intersection output buffer");
					kernel_args();
					kernel->run(cl::work_size(raygen->res_x(), raygen->res_y()), 
					            cl::work_size(16, 16));

					int err = clFinish(opencl.command_queue);
					check_for_cl_error(err, "after kernel");
					float ms = wtt.look();
					std::cout << "< " << ms << " ms >" << std::endl;
					return ms;
				}
				virtual std::string identification() { return "bbvh_tracer using ocl (kernel: " + kernel->name + ")"; }

		};


	}
}

#endif

