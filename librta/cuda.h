#ifndef __LIBRTA_CUDA_H__ 
#define __LIBRTA_CUDA_H__ 

#include "rta-config.h"
#include "basic_types.h"

#if RTA_HAVE_LIBCUDART == 1
#define WITH_CUDA
#endif

#ifdef WITH_CUDA

#include <iostream>	//temp
#include <cuda.h>
#include <cuda_runtime_api.h>

namespace rta {
	namespace cuda {
		class raygen_buffer_addon {
			public:
				float *gpu_origin, *gpu_direction;
				float *gpu_maxt;

				raygen_buffer_addon(int w, int h) : gpu_origin(0), gpu_direction(0),  gpu_maxt(0) {
					std::cout << "allocating " << ((w*h*7*sizeof(float))>>20) << "M of ray storage on GPU" << std::endl;
					cudaMalloc((void**)&gpu_origin, w*h*3*sizeof(float));
					cudaMalloc((void**)&gpu_direction, w*h*3*sizeof(float));
					cudaMalloc((void**)&gpu_maxt, w*h*sizeof(float));
				}
				~raygen_buffer_addon() {
					cudaFree(gpu_origin);
					cudaFree(gpu_direction);
					cudaFree(gpu_maxt);
				}
		};

		template<typename base_rgen> class raygen_with_buffer : public base_rgen, protected raygen_buffer_addon {
			public:
				vec3_t *tmp0, *tmp1;

				raygen_with_buffer(int w, int h) : base_rgen(w, h), raygen_buffer_addon(w, h), tmp0(0), tmp1(0) {
					tmp0 = new vec3_t[3*w*h];
					tmp1 = new vec3_t[3*w*h];
				}
				~raygen_with_buffer() {
					delete [] tmp0;
					delete [] tmp1;
				}
				virtual void generate_rays() {
					base_rgen::generate_rays();
					for (int y = 0; y < base_rgen::raydata.h; ++y)
						for (int x = 0; x < base_rgen::raydata.w; ++x)
							tmp0[y*base_rgen::raydata.w+x] = *base_rgen::origin(x,y),
							tmp1[y*base_rgen::raydata.w+x] = *base_rgen::direction(x,y);
					cudaMemcpy(gpu_origin, tmp0, base_rgen::raydata.w*base_rgen::raydata.h*3*sizeof(float), cudaMemcpyHostToDevice);
					cudaMemcpy(gpu_direction, tmp1, base_rgen::raydata.w*base_rgen::raydata.h*3*sizeof(float), cudaMemcpyHostToDevice);
				}
				virtual std::string identification() {
					return "cuda raygen adaptor for " + base_rgen::identification();
				}
				virtual void dont_forget_to_initialize_max_t() {}
		};

		template<box_t__and__tri_t> class gpu_ray_bouncer {
			public:
				forward_traits;
				triangle_intersection<tri_t> *gpu_last_intersection;
		};

		
		/*! \brief A <b>stupid hack</b> to tell the rta program that the plugin uses cuda so that it can setup the ray bouncer to collect intersections appropriately.
		 * 	
		 * 	For general plugin implementers this is not nice, sorry. Please take care of setting this up in \ref initialize by assigning to rta::cuda::plugin_with_cuda_loaded.
		 */
		bool using_cuda();
		extern bool plugin_with_cuda_loaded;
	}
}

#else
// for debugging cuda, to avoid stupid mistakes...
#error NO CUDA
#endif

#endif

