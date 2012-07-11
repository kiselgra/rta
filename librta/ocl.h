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
					gpu_intersections->copy_from_buffer_blocking(to, 0, sizeof(triangle_intersection<tri_t>) * w * h);
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

