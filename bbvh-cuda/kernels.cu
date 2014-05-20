// cuda can't handle gcc 4.7 includes...
// https://bugs.archlinux.org/task/29359
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

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
			__global__ void trace_dis(cuda::simple_triangle *triangles, int n, bbvh::node<cuda::simple_aabb> *nodes, 
			                          vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
			                          int w, int h, triangle_intersection<simple_triangle> *is) {
					/*
				uint32_t stack[16];
				stack[0] = 0;
				int sp = 0;
				uint tid = ray_y*w+ray_x;
				vec3f orig = (ray_orig)[tid];
				vec3f dir = (ray_dir)[tid];
				float t_max = max_t[tid];
				simple_aabb box;
				float dist;
				triangle_intersection<simple_triangle> closest;
				closest.t = FLT_MAX;
				while (sp >= 0) {
					uint node = stack[sp--];
					bbvh::node<simple_aabb> curr = nodes[node];
					if (curr.inner()) {
						if (intersect_aabb(curr.box, &orig, &dir, dist))
							if (dist < closest.t && dist <= t_max) {
								stack[++sp] = curr.right();
								stack[++sp] = curr.left();
							}
					}
					else {
						uint elems = curr.elems();
						uint offset = curr.tris();
						for (int i = 0; i < elems; ++i) {
							triangle_intersection<simple_triangle> is(offset+i);
							if (intersect_tri_opt(triangles[offset+i], &orig, &dir, is)) {
								if (is.t < closest.t)
									closest = is;
							}
						}
					}
				}
					*/
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
