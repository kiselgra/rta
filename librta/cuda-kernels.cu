// cuda can't handle gcc 4.7 includes...
// https://bugs.archlinux.org/task/29359
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "basic_types.h"
#include "cuda-kernels.h"
#include "cuda-vec.h"

#include <iostream>
#include <stdio.h>
using namespace std;

namespace rta {
	namespace cuda {

		// ray bouncer

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

			#undef ray_x
			#undef ray_y

		}

		void reset_intersections(rta::triangle_intersection<rta::cuda::simple_triangle> *last_intersection, uint w, uint h) {
			checked_cuda(cudaGetLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::reset_intersections<<<blocks,threads>>>(last_intersection, w, h);
			checked_cuda(cudaGetLastError());
			checked_cuda(cudaDeviceSynchronize());
		}


		// ray generator

		namespace k {
			__global__ void setup_shirley_rays(float *dirs, float *orgs, float *maxts, 
											   float fovy, float aspect, int w, int h, float3 view_dir, float3 pos, float3 up, float maxt) {
				int2 gid = make_int2(blockIdx.x * blockDim.x + threadIdx.x, blockIdx.y * blockDim.y + threadIdx.y);
				if (gid.x >= w || gid.y >= h)
					return;
				orgs[3*(gid.y * w + gid.x)+0] = pos.x;
				orgs[3*(gid.y * w + gid.x)+1] = pos.y;
				orgs[3*(gid.y * w + gid.x)+2] = pos.z;
				maxts[gid.y * w + gid.x] = maxt;
				fovy /= 2.0;
				float height = tanf(M_PI * fovy / 180.0f);
				float width = aspect * height;
				
				float u_s = (((float)gid.x+0.5f)/(float)w) * 2.0f - 1.0f;	// \in (-1,1)
				float v_s = (((float)gid.y+0.5f)/(float)h) * 2.0f - 1.0f;
				u_s = width * u_s;	// \in (-pw/2, pw/2)
				v_s = height * v_s;
			
				/*
				float3 W, TxW, U, V;
				div_vec3f_by_scalar(&W, &view_dir, length_of_vec3f(&view_dir));
				cross_vec3f(&TxW, &up, &W);
				div_vec3f_by_scalar(&U, &TxW, length_of_vec3f(&TxW));
				cross_vec3f(&V, &W, &U);

				float3 dir;
				mul_vec3f_by_scalar(&dir, &U, u_s);
				float3 tmp;
				mul_vec3f_by_scalar(&tmp, &V, v_s);
				add_components_vec3f(&dir, &dir, &tmp);
				add_components_vec3f(&dir, &dir, &W);
				normalize_vec3f(&dir);
				*/
			float3 vd = view_dir;
			float3 vu = up;
			float3 W, TxW, U, V;
			div_vec3f_by_scalar(&W, &vd, length_of_vec3f(&vd));
			cross_vec3f(&TxW, &vu, &W);
			div_vec3f_by_scalar(&U, &TxW, length_of_vec3f(&TxW));
			cross_vec3f(&V, &W, &U);

			float3 dir = make_float3(0,0,0), tmp;
			mul_vec3f_by_scalar(&dir, &U, u_s);
			mul_vec3f_by_scalar(&tmp, &V, v_s);
			add_components_vec3f(&dir, &dir, &tmp);
			add_components_vec3f(&dir, &dir, &W);
			normalize_vec3f(&dir);

				dirs[3*(gid.y * w + gid.x)+0] = dir.x;
				dirs[3*(gid.y * w + gid.x)+1] = dir.y;
				dirs[3*(gid.y * w + gid.x)+2] = dir.z;
			}
		}

		void setup_shirley_rays(float *dirs, float *orgs, float *maxts, 
								float fovy, float aspect, int w, int h, float3 *dir, float3 *pos, float3 *up, float maxt) {
			checked_cuda(cudaGetLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::setup_shirley_rays<<<blocks,threads>>>(dirs, orgs, maxts, fovy, aspect, w, h, *dir, *pos, *up, maxt);
			checked_cuda(cudaGetLastError());
			checked_cuda(cudaDeviceSynchronize());
		}

	}
}

