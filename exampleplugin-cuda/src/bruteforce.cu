#include <librta/basic_types.h>
#include <librta/cuda-kernels.h>
#include <librta/cuda-vec.h>
#include <librta/intersect.h>

#include <iostream>
using namespace std;

namespace rta {
	namespace cuda {
		namespace example {

			#define ray_x (blockIdx.x * blockDim.x + threadIdx.x)
			#define ray_y (blockIdx.y * blockDim.y + threadIdx.y)

			__global__ void bruteforce(simple_triangle *triangles, int n, vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<simple_triangle> *intersections) {
				if (ray_x < w & ray_y < h) {
					uint tid = ray_y*w+ray_x;
					vec3f orig = (ray_orig)[tid];
					vec3f dir = (ray_dir)[tid];
					float t_max = max_t[tid];
					triangle_intersection<simple_triangle> is = intersections[tid];
					triangle_intersection<simple_triangle> tmp;
					for (int i = 0; i < n; ++i) {
						tmp.t = FLT_MAX;
						if (intersect_tri_opt(triangles[i], &orig, &dir, tmp))
							if (tmp.t < is.t && tmp.t <= t_max) {
								is = tmp;
								is.ref = i;
							}
					}
// 					is.t=0;
					intersections[tid] = is;
				}
			}

			void trace_bruteforce(simple_triangle *triangles, int n, vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<simple_triangle> *intersection) {
				cout << "trace call!" << endl;
				dim3 threads(16, 16);
				dim3 blocks = block_configuration_2d(w, h, threads);
				bruteforce<<<threads, blocks>>>(triangles, n, ray_orig, ray_dir, max_t, w, h, intersection);
				cudaDeviceSynchronize();
			}

		}
	}
}

