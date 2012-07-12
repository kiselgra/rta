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

		template<box_t__and__tri_t, typename bvh_t_> class bbvh_direct_is_tracer : public basic_raytracer<forward_traits> {
			public:
				typedef bvh_t_ bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				typedef typename ocl::bouncer_buffer_addon<forward_traits> bouncer_buffer_addon_t;
				typedef typename ocl::raygen_buffer_addon raygen_buffer_addon_t;
				using basic_raytracer<forward_traits>::raygen;
				using basic_raytracer<forward_traits>::cpu_bouncer;
				cl::context &ctx;
				cl::program *test_prog;
				cl::kernel *test_kernel;
				cl::kernel *test_kernel2;
				cl::buffer *test_buf;
				cl::buffer *is_buf;
				bouncer_buffer_addon_t *bba;
				raygen_buffer_addon_t *rba;
				bbvh_t *bbvh;
				ocl::stackspace *stack;
		
				/* remove me!!  TODO
				 */
				std::string read_file(const std::string &filename)
				{
					std::ifstream in(filename.c_str());
					std::string str;
					while (in) 
					{
						std::string line;
						std::getline(in, line);
						str += line + "\n";
					}
					return str;
				}

				bbvh_direct_is_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b, cl::context &c) 
				: basic_raytracer<forward_traits>(gen, b, bvh), bbvh(bvh), ctx(c), stack(0) {
					test_prog = new cl::program(read_file("test.ocl"), c, "-cl-nv-verbose");
					test_kernel = new cl::kernel(test_prog->kernel("test"));
					test_kernel2 = new cl::kernel(test_prog->kernel("test2"));
					test_buf = new cl::buffer(c, CL_MEM_WRITE_ONLY, 512*sizeof(float));

// 					test_kernel->start_params();
// 					test_kernel->add_param(test_buf, "bla");
// 					test_kernel->run(cl::work_size(512), cl::work_size(64));
// 					clFinish(c.command_queue);
// 					float bu[512];
// 					test_buf->copy_from_buffer_blocking(bu, 0, 512*sizeof(float));
// 					for (int i = 0; i < 32; ++i) {
// 						for (int j = 0; j < 16; ++j) {
// 							std::cout << "\t" << bu[16*i + j];
// 						}
// 						std::cout << std::endl;
// 					}

					if (b) ray_bouncer(b);
					if (gen) ray_generator(gen);
				}

				virtual void ray_bouncer(bouncer *rb) {
					basic_raytracer<forward_traits>::ray_bouncer(rb);
					bba = dynamic_cast<bouncer_buffer_addon_t*>(rb);
					if (bba == 0)
						throw std::logic_error("while creating ray tracer \"" + identification() + "\": bouncer \"" + 
						                       rb->identification() + "\" does not have an opencl buffer attachment.");
				}

				virtual void ray_generator(rta::ray_generator *rg) { 
					basic_raytracer<forward_traits>::ray_generator(rg);
					rba = dynamic_cast<raygen_buffer_addon_t*>(rg);
					if (rba == 0)
						throw std::logic_error("while creating ray tracer \"" + identification() + "\": ray generator \"" + 
						                       rg->identification() + "\" does not have an opencl buffer attachment.");
					stack = new ocl::stackspace(rg->res_x(), rg->res_y(), 64, ctx);
				}
				
				virtual float trace_rays() {
					clFinish(ctx.command_queue);
					wall_time_timer wtt; wtt.start();

					test_kernel2->start_params();
					test_kernel2->add_param(rba->buffer(), "the ray buffer");
					test_kernel2->add_param(bbvh->node_buffer, "the bvh nodes");
					test_kernel2->add_param(bbvh->tri_buffer, "the triangle array");
					test_kernel2->add_param(stack->buffer(), "the stack");
					test_kernel2->add_param(bba->buffer(), "the intersection output buffer");
					test_kernel2->run(cl::work_size(raygen->res_y(), raygen->res_x()), 
					                  cl::work_size(16, 16));

					int err = clFinish(ctx.command_queue);
					check_for_cl_error(err, "after kernel");
					float ms = wtt.look();
					return ms;
				}
				virtual std::string identification() { return "bbvh_direct_is_tracer using ocl"; }

		};


	}
}

#endif

