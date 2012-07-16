#ifndef __OCL_H__ 
#define __OCL_H__ 

#include "librta.h"

#include <liblumocl/lumocl.h>

namespace rta {
	namespace ocl {

		class ocl_support {
		protected:
			cl::context &opencl;
			ocl_support(cl::context &ocl) : opencl(ocl) {}
		};

		template<box_t__and__tri_t> class bouncer_buffer_addon : public ocl_support {
			public:
				declare_traits_types;
			protected:
				uint w, h;
				cl::buffer *gpu_intersections;
			public:
				bouncer_buffer_addon(uint w, uint h, cl::context &c) : w(w), h(h), ocl_support(c), gpu_intersections(0) {
					gpu_intersections = new cl::buffer(c, CL_MEM_READ_WRITE, sizeof(triangle_intersection<tri_t>) * w * h);
				}
				cl::buffer* buffer() { return gpu_intersections; }
			protected:
				void download_intersection_info(void *to) {
// 					std::cout << "downloading intersection data " << w << " " << h << std::endl;
					gpu_intersections->copy_from_buffer_blocking(to, 0, sizeof(triangle_intersection<tri_t>) * w * h);
// 					std::cout << "done" << std::endl;
// 					triangle_intersection<tri_t> x = ((triangle_intersection<tri_t>*)to)[0];
// 					std::cout << "t: " << x.t << "\tb: " << x.beta << "\tg: " << x.gamma << "\tr: " << x.ref << std::endl;
// 					triangle_intersection<tri_t> y = ((triangle_intersection<tri_t>*)to)[1];
// 					std::cout << "t: " << y.t << "\tb: " << y.beta << "\tg: " << y.gamma << "\tr: " << y.ref << std::endl;
// 					exit(0);
					clFinish(opencl.command_queue);
				}
		};
			
		template<box_t__and__tri_t> 
		class direct_diffuse_illumination : public bouncer, public lighting_collector<forward_traits>, public bouncer_buffer_addon<forward_traits> {
				declare_traits_types;
				typedef lighting_collector<forward_traits> coll_t;
				image<triangle_intersection<tri_t>, 1> last_intersection;
			public:
				direct_diffuse_illumination(uint w, uint h, cl::context &c) 
				: lighting_collector<forward_traits>(w, h, 0), bouncer_buffer_addon<forward_traits>(w, h, c), last_intersection(w, h) {
					coll_t::last_intersection = & last_intersection;
				}
				virtual void add_pointlight(const vec3_t &at, const vec3_t &col) {
					coll_t::add_pointlight(at, col);
				}
				virtual void shade() {
					download_intersection_info(last_intersection.data);
					coll_t::shade();
				}
				virtual void bounce() {
				}
				virtual bool trace_further_bounces() {
					return false;
				}
				virtual void save(const std::string &filename) {
					coll_t::save(filename);
				}
				virtual std::string identification() { return "ocl ray bouncer producing a png with diffuse shading."; }
		};
		
		class raygen_buffer_addon  : public ocl_support {
				cl::buffer *ray_buffer;
				uint w, h;
			public:
				raygen_buffer_addon(uint w, uint h, cl::context &c)
				:  ocl_support(c), ray_buffer(0), w(w), h(h) {
					ray_buffer = new cl::buffer(c, CL_MEM_READ_WRITE, sizeof(float)*3*2 * w * h, 0);
				}

				cl::buffer* buffer() { return ray_buffer; }
					
				void upload_buffer(void *from) {
// 					std::cout << "uploading ray buffer " << w << " " << h << std::endl;
					ray_buffer->copy_to_buffer_blocking(from, 0, sizeof(float)*3*2 * w * h);
// 					std::cout << "done" << std::endl;
				}
		};

		template<typename raygen_t> class raygen_with_buffer : public raygen_t, public raygen_buffer_addon {
			public:
				raygen_with_buffer(uint res_x, uint res_y, cl::context &c) 
				:  raygen_t(res_x, res_y), raygen_buffer_addon(res_x, res_y, c) {
				}

				virtual void generate_rays() {
					raygen_t::generate_rays();
					upload_buffer(this->raydata.data);
				}

				virtual std::string identification() { return "ray generator with opencl buffer (based on " + raygen_t::identification() + ")"; }
		};

		typedef raygen_with_buffer<cam_ray_generator_shirley> cam_ray_buffer_generator_shirley  ;

		class stackspace {
				cl::buffer *stack;
				int n;
			public:
				stackspace(uint w, uint h, uint n, cl::context &c) : stack(0), n(n) {
					stack = new cl::buffer(c, CL_MEM_READ_WRITE, sizeof(cl_uint) * w*h*n, 0);
				}
				cl::buffer* buffer() { return stack; }
				int depth() { return n; }
		};

		template<box_t__and__tri_t> class raytracer_ocl_addon : public ocl_support {
				typedef typename ocl::bouncer_buffer_addon<forward_traits> bouncer_buffer_addon_t;
				typedef typename ocl::raygen_buffer_addon raygen_buffer_addon_t;
			protected:
				cl::program *prog;
				cl::kernel *kernel;
				cl::buffer *is_buf;
				bouncer_buffer_addon_t *bba;
				raygen_buffer_addon_t *rba;
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

			public:
				raytracer_ocl_addon(cl::context &c, const std::string &sourcefile, const std::string &kernelname) 
				: ocl_support(c), prog(0), kernel(0), is_buf(0), bba(0), rba(0), stack(0) {
					prog = new cl::program(read_file(sourcefile), c, "-cl-nv-verbose");
					kernel = new cl::kernel(prog->kernel(kernelname));
				}

				void ray_bouncer(rta::bouncer  *rb) {
					bba = dynamic_cast<bouncer_buffer_addon_t*>(rb);
					if (bba == 0)
						throw std::logic_error("while setting ray bouncer to cl rt addon: bouncer \"" + 
						                       rb->identification() + "\" does not have an opencl buffer attachment.");
				}

				void ray_generator(rta::ray_generator *rg) {
					rba = dynamic_cast<raygen_buffer_addon_t*>(rg);
					if (rba == 0)
						throw std::logic_error("while setting ray generator to cl rt addon: ray generator \"" + 
						                       rg->identification() + "\" does not have an opencl buffer attachment.");
					if (!stack) stack = new ocl::stackspace(rg->res_x(), rg->res_y(), 64, opencl);
				}
	
		};

		bool using_ocl();
		extern cl::context *context;
	
		/*! Round up to next higher power of 2 (return x if it's already a power of 2). 
		 * [http://stackoverflow.com/questions/364985/algorithm-for-finding-the-smallest-power-of-two-thats-greater-or-equal-to-a-giv]
		 */
		inline uint32_t pow2roundup(uint32_t x)
		{
			if (x <= 0)
				return 0;
			--x;
			x |= x >> 1;
			x |= x >> 2;
			x |= x >> 4;
			x |= x >> 8;
			x |= x >> 16;
			return x+1;
		}

	}
}


#endif

