#ifndef __BBVH_OCL_H__ 
#define __BBVH_OCL_H__ 

#include "bbvh/bbvh.h"

#include <liblumocl/lumocl.h>

namespace rta {
	namespace ocl {
		
		class ocl_support {
		protected:
			cl::context &opencl;
			ocl_support(cl::context &ocl) : opencl(ocl) {}
		};

		class cam_ray_buffer_generator_shirley : public cam_ray_generator_shirley, public ocl_support {
				cl::buffer ray_buffer;
			public:
				cam_ray_buffer_generator_shirley(uint res_x, uint res_y, cl::context &c) 
				: cam_ray_generator_shirley(res_x, res_y), ocl_support(c), ray_buffer(c, CL_MEM_READ_WRITE, sizeof(float)*3*2, 0) {
				}

				void generate_rays() {
					cam_ray_generator_shirley::generate_rays();
					ray_buffer.copy_to_buffer_blocking(raydata.data, 0, sizeof(float)*3*2);
				}
		};


	}
}

#endif

