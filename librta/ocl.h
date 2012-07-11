#ifndef __OCL_H__ 
#define __OCL_H__ 

#include <liblumocl/lumocl.h>

namespace rta {
	namespace ocl {

		class ocl_support {
		protected:
			cl::context &opencl;
			ocl_support(cl::context &ocl) : opencl(ocl) {}
		};

			
		template<box_t__and__tri_t> class direct_diffuse_illumination : public bouncer, public lighting_collector<forward_traits> {
				declare_traits_types;
				typedef lighting_collector<forward_traits> coll_t;
				image<triangle_intersection<tri_t>, 1> last_intersection;
			public:
				direct_diffuse_illumination(uint w, uint h) : lighting_collector<forward_traits>(w, h, 0), last_intersection(w, h) {
					coll_t::last_intersection = & last_intersection;
				}
				virtual void add_pointlight(const vec3_t &at, const vec3_t &col) {
					coll_t::add_pointlight(at, col);
				}
				virtual void shade() {
					download_intersection_info();
					coll_t::shade();
				}
				void download_intersection_info() {
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


	
	}
}


#endif

