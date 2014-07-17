#ifndef __RTA_BBVH_CUDA_H__ 
#define __RTA_BBVH_CUDA_H__ 

#include "bbvh/bbvh.h"
#include "librta/cuda-vec.h"
#include "librta/cuda-kernels.h"
#include "librta/cuda.h"
#include "bbvh-cuda-node.h"

namespace rta {
	namespace cuda {

		/*! \brief Forwarding of an rta::bbvh to cuda.
		 *
		 * 	Same scheme as ocl::binary_bvh.
		 *
		 * 	Now extended to also take already-on-gpu data.
		 */
		template<box_t__and__tri_t> class binary_bvh : public rta::binary_bvh<forward_traits> {
			public:
				declare_traits_types;
				typedef rta::binary_bvh<forward_traits> parent_t;
				typedef typename parent_t::node node_t;
				
				device_array<tri_t> triangle_data;
				device_array<bbvh::node<box_t>> node_data;

				virtual std::string identification() { return "cuda front to " + parent_t::identification() + " (in ::rta)"; }

				//! \attention copies the pointer, not the data pointed to. This means you have to take care to free the data.
				virtual void take_node_array(node_t *array, uint n) {
					node_data.data = (bbvh::node<box_t>*)array;
					node_data.n = n;
					node_data.owns_memory = false;
				}
				//! \attention copies the pointer, not the data pointed to. This means you have to take care to free the data.
				virtual void take_node_array(bbvh::node<box_t> *array, uint n) {
					node_data.data = array;
					node_data.n = n;
					node_data.owns_memory = false;
				}
				//! \attention copies the pointer, not the data pointed to. This means you have to take care to free the data.
				virtual void take_triangle_array(tri_t *array, uint n) {
					triangle_data.data = array;
					triangle_data.n = n;
					triangle_data.owns_memory = false;
				}
				//! \attention triggers an upload of the data to the gpu.
				virtual void take_node_array(std::vector<node_t> &n) {
					parent_t::take_node_array(n);
					static_assert(sizeof(node_t) == 32, 
					              "The node size is not as expected here. Should this be so, how is it transferred to Cuda?");
					static_assert(sizeof(node_t) == sizeof(bbvh::node<box_t>),
								  "The size of rta::binary_bvh::node and rta::cuda::bbvh::node does not match. Read the documentation of the latter!");
					this->node_data.upload((bbvh::node<box_t>*)&this->nodes[0], this->nodes.size());
				}
				//! \attention triggers an upload of the data to the gpu.
				virtual void take_triangle_array(std::vector<tri_t> &t) {
					parent_t::take_triangle_array(t);
					this->triangle_data.upload(&this->triangles[0], this->triangles.size());
				}
				virtual tri_t* canonical_triangle_ptr() {
					return triangle_data.download();
				}
				virtual void free_canonical_triangles(tri_t *data) {
					delete [] data;
				}
		};
			
		template<box_t__and__tri_t>
		class bbvh_gpu_tracer : public cuda::gpu_raytracer<forward_traits> {
			public:
				typedef typename cuda::binary_bvh<forward_traits> bbvh_t;
// 				typedef typename bbvh_t::node_t node_t;
				bbvh_t *bbvh;
				
				bbvh_gpu_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: gpu_raytracer<forward_traits>(gen, b, bvh), bbvh(bvh) {
				}
				virtual void acceleration_structure(rta::basic_acceleration_structure<forward_traits> *as) {
					bbvh = dynamic_cast<bbvh_t*>(as);
					gpu_raytracer<forward_traits>::acceleration_structure(as);
				}
				virtual bool supports_max_t() { return true; }
		};

		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_dis_tracer : public bbvh_gpu_tracer<forward_traits> {
			public:
// 				typedef typename bbvh_gpu_tracer<forward_traits>::bbvh_t bbvh_t;
				typedef bvh_t bbvh_t;
				declare_traits_types;
				bbvh_gpu_dis_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh direct intersection tracer; a cuda version of rta::bbvh_direct_is_tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_dis(tri_t *triangles, int n, bbvh::node<box_t> *nodes, 
					               vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_dis(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
					          (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
					          this->gpu_raygen->w, this->gpu_raygen->h,
					          this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_dis_tracer* copy() {
					return new bbvh_gpu_dis_tracer(*this);
				}

		};

	}
}

#endif

