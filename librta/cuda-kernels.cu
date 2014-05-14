#include "basic_types.h"
#include "cuda-kernels.h"

#include <iostream>
using namespace std;

namespace rta {
	namespace cuda {

		namespace k {

			#define ray_x (blockIdx.x * blockDim.x + threadIdx.x)
			#define ray_y (blockIdx.y * blockDim.y + threadIdx.y)

			__global__ void reset_intersections(rta::triangle_intersection<rta::cuda::simple_triangle> *last_intersection, uint w, uint h) {
				if (ray_x < w && ray_y < h) {
					last_intersection[ray_y * w + ray_x].t = FLT_MAX;
				}
			}

		}

		void reset_intersections(rta::triangle_intersection<rta::cuda::simple_triangle> *last_intersection, uint w, uint h) {
			cout << "resetting intersections!" << endl;
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			cout << "T: " << threads.x << " " << threads.y << " " << threads.z << endl;
			cout << "B: " << blocks.x << " " << blocks.y << " " << blocks.z << endl;
			k::reset_intersections<<<threads,blocks>>>(last_intersection, w, h);
			cout << "waiting" << endl;
			cudaDeviceSynchronize();
			cout << "done" << endl;
		}

	}
}

