#ifndef __CUDA_KERNELS_H__ 
#define __CUDA_KERNELS_H__ 

#include "rta-config.h"

#if RTA_HAVE_LIBCUDART == 1

namespace rta {
	namespace cuda {
		
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

