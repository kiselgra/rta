#include "basic_types.h"
#include "cuda-kernels.h"

#include <iostream>
#include <stdio.h>
using namespace std;

namespace rta {
	namespace cuda {

		namespace k {

			#define ray_x (blockIdx.x * blockDim.x + threadIdx.x)
			#define ray_y (blockIdx.y * blockDim.y + threadIdx.y)

			__global__ void reset_intersections(rta::triangle_intersection<rta::cuda::simple_triangle> *last_intersection, uint w, uint h) {
				if (ray_x < w && ray_y < h) {
					last_intersection[ray_y * w + ray_x].t = FLT_MAX;
					last_intersection[ray_y * w + ray_x].beta = 1;
					last_intersection[ray_y * w + ray_x].gamma = 2;
				}
			}

		}

		#define checked_cuda(ans) { gpu_assert((ans), __FILE__, __LINE__); }
		inline void gpu_assert(cudaError_t code, char *file, int line, bool abort=true) {
			if (code != cudaSuccess) {
				fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
				if (abort) exit(code);
			}
		}

		void reset_intersections(rta::triangle_intersection<rta::cuda::simple_triangle> *last_intersection, uint w, uint h) {
			checked_cuda(cudaGetLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::reset_intersections<<<blocks,threads>>>(last_intersection, w, h);
			checked_cuda(cudaGetLastError());
			checked_cuda(cudaDeviceSynchronize());
		}

	}
}

