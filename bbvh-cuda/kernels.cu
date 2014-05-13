#include "librta/basic_types.h"
#include "bbvh-cuda-node.h"

#include <librta/cuda-kernels.h>
#include <librta/cuda-vec.h>
#include <librta/intersect.h>

#include <iostream>
using namespace std;

namespace rta {
	namespace cuda {
			
		#define ray_x (blockIdx.x * blockDim.x + threadIdx.x)
		#define ray_y (blockIdx.y * blockDim.y + threadIdx.y)

		namespace k {
			__global__ void trace_dis(simple_triangle *triangles, int n, bbvh::node<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
			               int w, int h, triangle_intersection<simple_triangle> *is) {
				uint tid = ray_y*w+ray_x;
				vec3f orig = (ray_orig)[tid];
				vec3f dir = (ray_dir)[tid];
// 				float t_max = max_t[tid];
				simple_aabb box = nodes[0].box;
				float dist;
				if (intersect_aabb(box, &orig, &dir, dist))
					is[tid].t = 0;
			}
		}

		void trace_dis(simple_triangle *triangles, int n, bbvh::node<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
		               int w, int h, triangle_intersection<simple_triangle> *is) {
			cout << "kernel!" << endl;
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::trace_dis<<<threads, blocks>>>(triangles, n, nodes, ray_orig, ray_dir, max_t, w, h, is);
			cudaDeviceSynchronize();
		}

	}
}
