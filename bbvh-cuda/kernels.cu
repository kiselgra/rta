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
#include <stdio.h>
using namespace std;

namespace rta {
	namespace cuda {
			
		#define ray_x (blockIdx.x * blockDim.x + threadIdx.x)
		#define ray_y (blockIdx.y * blockDim.y + threadIdx.y)

		namespace k {
			__global__ void trace_dis(cuda::simple_triangle *triangles, int n, bbvh_node<cuda::simple_aabb> *nodes, 
			                          vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
			                          int w, int h, triangle_intersection<simple_triangle> *intersections) {
				if (ray_x < w && ray_y < h) {
					uint tid = ray_y*w+ray_x;
					uint32_t stack[32];
					stack[0] = 0;
					int sp = 0;
					vec3f orig = (ray_orig)[tid];
					vec3f dir = (ray_dir)[tid];
					float t_max = max_t[tid];
					simple_aabb box;
					float dist;
					triangle_intersection<simple_triangle> closest = intersections[tid];
					closest.t = FLT_MAX;
					while (sp >= 0) {
						uint node = stack[sp--];
						bbvh_node<simple_aabb> curr = nodes[node];
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
									if (is.t < closest.t && is.t <= t_max)
										closest = is;
								}
							}
						}
					}
					intersections[tid] = closest;
				}
			}
				
			__global__ void trace_shadow_dis(cuda::simple_triangle *triangles, int n, bbvh_node<cuda::simple_aabb> *nodes, 
											 vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
											 int w, int h, triangle_intersection<simple_triangle> *intersections) {
				if (ray_x < w && ray_y < h) {
					uint tid = ray_y*w+ray_x;
					uint32_t stack[32];
					stack[0] = 0;
					int sp = 0;
					vec3f orig = (ray_orig)[tid];
					vec3f dir = (ray_dir)[tid];
					float t_max = max_t[tid];
					simple_aabb box;
					float dist;
					triangle_intersection<simple_triangle> closest = intersections[tid];
					closest.t = FLT_MAX;
					while (sp >= 0) {
						uint node = stack[sp--];
						bbvh_node<simple_aabb> curr = nodes[node];
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
									if (is.t < closest.t && is.t <= t_max) {
										closest = is;
										break;
									}
								}
							}
							if (closest.t != FLT_MAX)
								break;
						}
					}
					intersections[tid] = closest;
				}
			}
			
			
			#define is_inner(F)       ((__float_as_int(F.x)&0x01)==1)
			#define extract_left(F)   (__float_as_int(F.x)>>1)
			#define extract_right(F)  (__float_as_int(F.y))
			#define extract_count(F)  (__float_as_int(F.x)>>1)
			#define extract_offset(F) (__float_as_int(F.y))
			#define box_min_x(nodes)  (nodes[0]).z
			#define box_min_y(nodes)  (nodes[0]).w
			#define box_min_z(nodes)  (nodes[1]).x
			#define box_max_x(nodes)  (nodes[1]).y
			#define box_max_y(nodes)  (nodes[1]).z
			#define box_max_z(nodes)  (nodes[1]).w
			/*! A CIS intersection method according to sijehara's master thesis.
			 *  
			 *  Differences to the naive version above:
			 *  - Only push nodes when necessary (i.e. assign curr if possible)
			 *  - Memory layout using 2 float4, which we hope yields better access patterns
			 *  - For two child intersections we use one box
			 */
			__global__ void trace_dis(cuda::simple_triangle *triangles, int n, /*bbvh_node_float4<cuda::simple_aabb> *nodes*/float4 *nodes, 
			                          vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
			                          int w, int h, triangle_intersection<simple_triangle> *intersections) {
				if (ray_x < w && ray_y < h) {
					uint tid = ray_y*w+ray_x;
					// ray data
					vec3f orig = (ray_orig)[tid];
					vec3f dir = (ray_dir)[tid];
					float t_max = max_t[tid];
					// stack mgmt
					uint32_t stack[32];
					int sp = -1;
					// cis info
					bool hit_left, hit_right;
					float dist_left = FLT_MAX, dist_right = FLT_MAX;;
					// triangle intersection
					triangle_intersection<simple_triangle> closest = intersections[tid];
					closest.t = FLT_MAX;
					float4 curr = nodes[0];

					while (true) {
						// fetch only first part, we don't need the box-only part.
						if (is_inner(curr)) {
							// load nodes
							float4 left_node[2], right_node[2];
							left_node[0] = nodes[2*extract_left(curr)+0];
							left_node[1] = nodes[2*extract_left(curr)+1];
							right_node[0] = nodes[2*extract_right(curr)+0];
							right_node[1] = nodes[2*extract_right(curr)+1];
							// extract data & intersect left
							cuda::simple_aabb bb;
							bb.min.x = box_min_x(left_node); bb.min.y = box_min_y(left_node); bb.min.z = box_min_z(left_node);
							bb.max.x = box_max_x(left_node); bb.max.y = box_max_y(left_node); bb.max.z = box_max_z(left_node);
							hit_left = intersect_aabb(bb, &orig, &dir, dist_left);
							// extract data & intersect right
							bb.min.x = box_min_x(right_node); bb.min.y = box_min_y(right_node); bb.min.z = box_min_z(right_node);
							bb.max.x = box_max_x(right_node); bb.max.y = box_max_y(right_node); bb.max.z = box_max_z(right_node);
							hit_right = intersect_aabb(bb, &orig, &dir, dist_right);
							if (dist_left >= closest.t || dist_left > t_max) hit_left = false;
							if (dist_right >= closest.t || dist_right > t_max) hit_right = false;
							// eval
							if (hit_left)
								if (hit_right) // note how we re-use the already loaded nodes.
									if (dist_left <= dist_right) {
										stack[++sp] = extract_right(curr);
										curr = left_node[0];
									}
									else {
										stack[++sp] = extract_left(curr);
										curr = right_node[0];
									}
								else // only hit the left node
									curr = left_node[0];
							else if (hit_right)
								curr = right_node[0];
							else if (sp >= 0)
								curr = nodes[2*stack[sp--]];
							else
								break;
						}
						else {
							uint elems = extract_count(curr);
							uint offset = extract_offset(curr);
							for (int i = 0; i < elems; ++i) {
								triangle_intersection<simple_triangle> is(offset+i);
								if (intersect_tri_opt(triangles[offset+i], &orig, &dir, is)) {
									if (is.t < closest.t && is.t <= t_max)
										closest = is;
								}
							}
							if (sp >= 0)
								curr = nodes[2*stack[sp--]];
							else
								break;
						}
					}
					intersections[tid] = closest;
				}
			}

			__global__ void trace_shadow_dis(cuda::simple_triangle *triangles, int n, /*bbvh_node_float4<cuda::simple_aabb> *nodes*/float4 *nodes, 
											 vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
											 int w, int h, triangle_intersection<simple_triangle> *intersections) {
				if (ray_x < w && ray_y < h) {
					uint tid = ray_y*w+ray_x;
					// ray data
					vec3f orig = (ray_orig)[tid];
					vec3f dir = (ray_dir)[tid];
					float t_max = max_t[tid];
					// stack mgmt
					uint32_t stack[32];
					int sp = -1;
					// cis info
					bool hit_left, hit_right;
					float dist_left = FLT_MAX, dist_right = FLT_MAX;;
					// triangle intersection
					triangle_intersection<simple_triangle> closest = intersections[tid];
					closest.t = FLT_MAX;
					float4 curr = nodes[0];

					while (true) {
						// fetch only first part, we don't need the box-only part.
						if (is_inner(curr)) {
							// load nodes
							float4 left_node[2], right_node[2];
							left_node[0] = nodes[2*extract_left(curr)+0];
							left_node[1] = nodes[2*extract_left(curr)+1];
							right_node[0] = nodes[2*extract_right(curr)+0];
							right_node[1] = nodes[2*extract_right(curr)+1];
							// extract data & intersect left
							cuda::simple_aabb bb;
							bb.min.x = box_min_x(left_node); bb.min.y = box_min_y(left_node); bb.min.z = box_min_z(left_node);
							bb.max.x = box_max_x(left_node); bb.max.y = box_max_y(left_node); bb.max.z = box_max_z(left_node);
							hit_left = intersect_aabb(bb, &orig, &dir, dist_left);
							// extract data & intersect right
							bb.min.x = box_min_x(right_node); bb.min.y = box_min_y(right_node); bb.min.z = box_min_z(right_node);
							bb.max.x = box_max_x(right_node); bb.max.y = box_max_y(right_node); bb.max.z = box_max_z(right_node);
							hit_right = intersect_aabb(bb, &orig, &dir, dist_right);
							if (dist_left >= closest.t || dist_left > t_max) hit_left = false;
							if (dist_right >= closest.t || dist_right > t_max) hit_right = false;
							// eval
							if (hit_left)
								if (hit_right) // note how we re-use the already loaded nodes.
									if (dist_left <= dist_right) {
										stack[++sp] = extract_right(curr);
										curr = left_node[0];
									}
									else {
										stack[++sp] = extract_left(curr);
										curr = right_node[0];
									}
								else // only hit the left node
									curr = left_node[0];
							else if (hit_right)
								curr = right_node[0];
							else if (sp >= 0)
								curr = nodes[2*stack[sp--]];
							else
								break;
						}
						else {
							uint elems = extract_count(curr);
							uint offset = extract_offset(curr);
							for (int i = 0; i < elems; ++i) {
								triangle_intersection<simple_triangle> is(offset+i);
								if (intersect_tri_opt(triangles[offset+i], &orig, &dir, is)) {
									if (is.t < closest.t && is.t <= t_max) {
										closest = is;
										break;
									}
								}
							}
							if (closest.t != FLT_MAX)
								break;
							if (sp >= 0)
								curr = nodes[2*stack[sp--]];
							else
								break;
						}
					}
					intersections[tid] = closest;
				}
			}

			#undef is_inner
			#undef extract_left
			#undef extract_right
			#undef extract_count
			#undef extract_offset
			#undef box_min_x
			#undef box_min_y
			#undef box_min_z
			#undef box_max_x
			#undef box_max_y
			#undef box_max_z
		}
		

		void trace_dis(simple_triangle *triangles, int n, bbvh_node<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
		               int w, int h, triangle_intersection<simple_triangle> *is) {
			checked_cuda(cudaPeekAtLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::trace_dis<<<blocks, threads>>>(triangles, n, nodes, ray_orig, ray_dir, max_t, w, h, is);
			checked_cuda(cudaPeekAtLastError());
			checked_cuda(cudaDeviceSynchronize());
		}
			
		void trace_shadow_dis(simple_triangle *triangles, int n, bbvh_node<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
							  int w, int h, triangle_intersection<simple_triangle> *is) {
			checked_cuda(cudaPeekAtLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::trace_shadow_dis<<<blocks, threads>>>(triangles, n, nodes, ray_orig, ray_dir, max_t, w, h, is);
			checked_cuda(cudaPeekAtLastError());
			checked_cuda(cudaDeviceSynchronize());
		}
		
		void trace_dis(simple_triangle *triangles, int n, bbvh_node_float4<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
		               int w, int h, triangle_intersection<simple_triangle> *is) {
			checked_cuda(cudaPeekAtLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::trace_dis<<<blocks, threads>>>(triangles, n, (float4*)nodes, ray_orig, ray_dir, max_t, w, h, is);
			checked_cuda(cudaPeekAtLastError());
			checked_cuda(cudaDeviceSynchronize());
		}

		void trace_shadow_dis(simple_triangle *triangles, int n, bbvh_node_float4<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
							  int w, int h, triangle_intersection<simple_triangle> *is) {
			checked_cuda(cudaPeekAtLastError());
			dim3 threads(16, 16);
			dim3 blocks = block_configuration_2d(w, h, threads);
			k::trace_shadow_dis<<<blocks, threads>>>(triangles, n, (float4*)nodes, ray_orig, ray_dir, max_t, w, h, is);
			checked_cuda(cudaPeekAtLastError());
			checked_cuda(cudaDeviceSynchronize());
		}


	}
}
