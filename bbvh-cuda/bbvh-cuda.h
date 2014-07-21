#ifndef __RTA_BBVH_CUDA_H__ 
#define __RTA_BBVH_CUDA_H__ 

#include "bbvh/bbvh.h"
#include "librta/cuda-vec.h"
#include "librta/cuda-kernels.h"
#include "librta/cuda.h"
#include "bbvh-cuda-node.h"

namespace rta {
	namespace cuda {

		/*!	\brief This is just a stub to be used bby cuda::binary_bvh as an alternative to the host-storage requiring rta::binary_bvh.
		 */
		template<box_t__and__tri_t, typename node = bbvh_node<_box_t>> class binary_bvh_gpu_only : public basic_acceleration_structure<forward_traits> {
			public:
				declare_traits_types;
				typedef node node_t;
				typedef typename node_t::link_t link_t;
		};

		/*! \brief Forwarding of an rta::bbvh to cuda.
		 *
		 * 	Same scheme as ocl::binary_bvh.
		 *
		 * 	Now extended to also take already-on-gpu data.
		 *	Takes a Bvh type as parent which must supply a node type according to \ref rta::binary_bvh::node.
		 */
		template<box_t__and__tri_t, typename bvh = binary_bvh_gpu_only<forward_traits>> class binary_bvh : public bvh {
			public:
				declare_traits_types;
				typedef bvh bvh_t;
				typedef bvh_t parent_t;
				typedef typename parent_t::node_t node_t;
				
				device_array<tri_t> triangle_data;
				device_array<node_t> node_data;

				virtual std::string identification() { return "cuda front to " + parent_t::identification() + " (in ::rta)"; }

				//! \attention copies the pointer, not the data pointed to. This means you have to take care to free the data.
				virtual void take_node_array(node_t *array, uint n) {
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
					this->node_data.upload((node_t*)&this->nodes[0], this->nodes.size());
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
				tri_t* triangle_ptr() { return triangle_data.data; }
				int triangle_count() { return triangle_data.n; }
		};
			
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_tracer : public cuda::gpu_raytracer<forward_traits> {
			public:
				typedef bvh_t bbvh_t;
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

		//! Cuda DIS tracer.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_dis_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_dis_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_dis(tri_t *triangles, int n, node_t *nodes, 
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

		//! Cuda DIS tracer using Aila et al's box hack.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_dis_ailabox_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_dis_ailabox_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_dis_ailabox(tri_t *triangles, int n, node_t *nodes, 
										   vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_dis_ailabox(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
									  (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
									  this->gpu_raygen->w, this->gpu_raygen->h,
									  this->gpu_bouncer->gpu_last_intersection);

					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_dis_ailabox_tracer* copy() {
					return new bbvh_gpu_dis_ailabox_tracer(*this);
				}
		};

		//! Cuda DIS shadow ray tracer.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_dis_shadow_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_dis_shadow_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh shadow ray tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_shadow_dis(tri_t *triangles, int n, node_t *nodes, 
										  vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_shadow_dis(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
									 (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
									 this->gpu_raygen->w, this->gpu_raygen->h,
									 this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_dis_shadow_tracer* copy() {
					return new bbvh_gpu_dis_shadow_tracer(*this);
				}
		};

		//! Cuda DIS shadow ray tracer using Aila et al's box hack.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_dis_ailabox_shadow_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_dis_ailabox_shadow_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh shadow ray tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_shadow_dis_ailabox(tri_t *triangles, int n, node_t *nodes, 
												  vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_shadow_dis_ailabox(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
											 (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
											 this->gpu_raygen->w, this->gpu_raygen->h,
											 this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_dis_ailabox_shadow_tracer* copy() {
					return new bbvh_gpu_dis_ailabox_shadow_tracer(*this);
				}
		};


		//! Cuda CIS tracer.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_cis_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_cis_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_cis(tri_t *triangles, int n, node_t *nodes, 
					               vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_cis(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
					          (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
					          this->gpu_raygen->w, this->gpu_raygen->h,
					          this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_cis_tracer* copy() {
					return new bbvh_gpu_cis_tracer(*this);
				}
		};

		//! Cuda CIS tracer using Aila et al's box hack.
		template<box_t__and__tri_t,	typename bvh_t>
		class bbvh_gpu_cis_ailabox_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_cis_ailabox_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_cis_ailabox(tri_t *triangles, int n, node_t *nodes, 
										   vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_cis_ailabox(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
									  (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
									  this->gpu_raygen->w, this->gpu_raygen->h,
									  this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_cis_ailabox_tracer* copy() {
					return new bbvh_gpu_cis_ailabox_tracer(*this);
				}
		};

		//! Cuda CIS shadow ray tracer.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_cis_shadow_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_cis_shadow_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh shadow ray tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_shadow_cis(tri_t *triangles, int n, node_t *nodes, 
										  vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_shadow_cis(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
									 (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
									 this->gpu_raygen->w, this->gpu_raygen->h,
									 this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_cis_shadow_tracer* copy() {
					return new bbvh_gpu_cis_shadow_tracer(*this);
				}
		};

		//! Cuda CIS shadow ray tracer using Aila et al's box hack.
		template<box_t__and__tri_t, typename bvh_t>
		class bbvh_gpu_cis_ailabox_shadow_tracer : public bbvh_gpu_tracer<forward_traits, bvh_t> {
			public:
				typedef bvh_t bbvh_t;
				typedef typename bbvh_t::node_t node_t;
				declare_traits_types;
				bbvh_gpu_cis_ailabox_shadow_tracer(rta::ray_generator *gen, bbvh_t *bvh, class bouncer *b)
				: bbvh_gpu_tracer<forward_traits, bvh_t>(gen, bvh, b) {
				}
				virtual std::string identification() { return "cuda bbvh shadow ray tracer."; }
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					void trace_shadow_cis_ailabox(tri_t *triangles, int n, node_t *nodes, 
												  vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<tri_t> *is);
					trace_shadow_cis_ailabox(this->bbvh->triangle_data.data, this->bbvh->triangle_data.n, this->bbvh->node_data.data,
											 (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
											 this->gpu_raygen->w, this->gpu_raygen->h,
											 this->gpu_bouncer->gpu_last_intersection);
					          
					float ms = wtt.look();
					return ms;
				}
				virtual bbvh_gpu_cis_ailabox_shadow_tracer* copy() {
					return new bbvh_gpu_cis_ailabox_shadow_tracer(*this);
				}
		};

	}
}

#endif

