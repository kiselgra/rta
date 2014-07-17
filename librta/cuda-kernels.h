#ifndef __CUDA_KERNELS_H__ 
#define __CUDA_KERNELS_H__ 

#include "rta-config.h"

#if RTA_HAVE_LIBCUDART == 1

#include <cuda_runtime.h>
#include <stdio.h>

namespace rta {
	namespace cuda {
			
		#ifndef checked_cuda
		#define checked_cuda(ans) { rta::cuda::gpu_assert((ans), (const char*)__FILE__, __LINE__); }
		inline void gpu_assert(cudaError_t code, const char *file, int line, bool abort=true) {
			if (code != cudaSuccess) {
				fprintf(stderr,(char*)"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
				if (abort) exit(code);
			}
		}
		#endif

	
		inline dim3 block_configuration_2d(int w, int h, dim3 threads) {
			int x = w / threads.x + ((w%threads.x) ? 1 : 0);
			int y = h / threads.y + ((h%threads.y) ? 1 : 0);
			return dim3(x, y);
		}

		void reset_intersections(rta::triangle_intersection<rta::cuda::simple_triangle> *last_intersection, uint w, uint h);
	
	}
}



#endif

#endif

