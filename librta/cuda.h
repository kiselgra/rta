#ifndef __LIBRTA_CUDA_H__ 
#define __LIBRTA_CUDA_H__ 

#include "rta-config.h"
#include "basic_types.h"
#include "raytrav.h"
#include "cuda-kernels.h"
#include "cuda-vec.h"

#if RTA_HAVE_LIBCUDART == 1
#define WITH_CUDA
#endif

#ifdef WITH_CUDA

#include <iostream>	//temp
#include <cuda.h>
#include <cuda_runtime_api.h>

namespace rta {
	namespace cuda {

		/*! \defgroup cuda Cuda Interface
		 *
		 * 	\brief Here we define how cuda is used with our framework.
		 *
		 * 	\attention Cuda never really supported templates in any way beyond
		 * 	loop unrolling, so <b>a lot</b> of supporting functionality is not
		 * 	available.
		 * 	This is also the reason you cannot include raytrav.h in cuda files
		 * 	and you have to implement some kind of separation between kernel
		 * 	code and rta interfaces.
		 *
		 */
		
		///// different memory layouts for cuda data. not done yet.
		
		/*! \defgroup cuda_data_layout Cuda Data Layouts.
		 * 	
		 * 	\brief Different data layouts for information loaded from cuda
		 * 	kernels. We might use these find the best representation for a
		 * 	given kind of data.
		 * 	\attention Note integrated yet - just a sketch.
		 */

		/*! \addtogroup cuda_data_layout
		 * 	@{
		 */
		namespace triangle_layout {
			struct plain_vntm {
				struct tri {
					float3 a, b, c;
					float3 na, nb, nc;
					float2 ta, tb, tc;
					int material_index;
				};
				tri *data;
			};
		}

		namespace raydata_layout {
			struct plain_soa {
				float3 *origin,
					   *direction;
				float *max_t;
			};
		}
		//! @}
		
		#ifndef checked_cuda
		#define checked_cuda(ans) { rta::cuda::gpu_assert((ans), (const char*)__FILE__, __LINE__); }
		inline void gpu_assert(cudaError_t code, const char *file, int line, bool abort=true) {
			if (code != cudaSuccess) {
				fprintf(stderr,(char*)"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
				if (abort) exit(code);
			}
		}
		#endif

		///// rta interface
		
		/*! \addtogroup cuda
		 * 	@{
		 */

		/*! \brief Representation of ray data that is allocated on the GPU.
		 * 	\note Derive from this, as well as from rta::ray_generator for cuda ray generators.
		 * 	\note Due to layouts (\ref cuda_data_layout) this might get extended.
		 */
		class gpu_ray_generator {
			public:
				float *gpu_origin, *gpu_direction;
				float *gpu_maxt;
				uint w, h;

				gpu_ray_generator(int w, int h) : gpu_origin(0), gpu_direction(0),  gpu_maxt(0), w(w), h(h) {
					std::cout << "allocating " << ((w*h*7*sizeof(float))>>20) << "M of ray storage on GPU" << std::endl;
					cudaMalloc((void**)&gpu_origin, w*h*3*sizeof(float));
					cudaMalloc((void**)&gpu_direction, w*h*3*sizeof(float));
					cudaMalloc((void**)&gpu_maxt, w*h*sizeof(float));
				}
				~gpu_ray_generator() {
					cudaFree(gpu_origin);
					cudaFree(gpu_direction);
					cudaFree(gpu_maxt);
				}
		};

		/*! \brief An adaptor for existing cpu ray generators which uploads the
		 * 	ray data to the GPU via \ref gpu_ray_generator.
		 */
		template<typename base_rgen> class raygen_with_buffer : public base_rgen, public gpu_ray_generator {
			public:
				vec3_t *tmp0, *tmp1;
				float *tmp2;

				raygen_with_buffer(int w, int h) : base_rgen(w, h), gpu_ray_generator(w, h), tmp0(0), tmp1(0) {
					tmp0 = new vec3_t[w*h];
					tmp1 = new vec3_t[w*h];
					tmp2 = new float[w*h];
				}
				~raygen_with_buffer() {
					delete [] tmp0;
					delete [] tmp1;
					delete [] tmp2;
				}
				virtual void generate_rays() {
					base_rgen::generate_rays();
					for (int y = 0; y < base_rgen::raydata.h; ++y)
						for (int x = 0; x < base_rgen::raydata.w; ++x)
							tmp0[y*base_rgen::raydata.w+x] = *base_rgen::origin(x,y),
							tmp1[y*base_rgen::raydata.w+x] = *base_rgen::direction(x,y),
							tmp2[y*base_rgen::raydata.w+x] = base_rgen::max_t(x,y);
					cudaMemcpy(gpu_origin, tmp0, base_rgen::raydata.w*base_rgen::raydata.h*3*sizeof(float), cudaMemcpyHostToDevice);
					cudaMemcpy(gpu_direction, tmp1, base_rgen::raydata.w*base_rgen::raydata.h*3*sizeof(float), cudaMemcpyHostToDevice);
					cudaMemcpy(gpu_maxt, tmp2, base_rgen::raydata.w*base_rgen::raydata.h*sizeof(float), cudaMemcpyHostToDevice);
				}
				virtual std::string identification() {
					return "cuda raygen adaptor for " + base_rgen::identification();
				}
				virtual void dont_forget_to_initialize_max_t() {}
		};

		/*! \brief An implementation of \ref rta::cam_ray_generator_shirley that runs on the gpu.
		 */
		class cam_ray_generator_shirley : public rta::cam_ray_generator_shirley, public gpu_ray_generator {
		public:
			cam_ray_generator_shirley(uint res_x, uint res_y) : rta::cam_ray_generator_shirley(res_x, res_y), gpu_ray_generator(res_x, res_y) {
			}
			virtual void generate_rays() {
				void setup_shirley_rays(float *dirs, float *orgs, float *maxts, 
										float fovy, float aspect, int w, int h, float3 *dir, float3 *pos, float3 *up, float maxt);
				setup_shirley_rays(gpu_direction, gpu_origin, gpu_maxt, 
								   fovy, aspect, w, h, (float3*)&dir, (float3*)&position, (float3*)&up, FLT_MAX);
			}
			virtual std::string identification() { return "cuda version of the ray generator according to shirley."; }
			virtual void dont_forget_to_initialize_max_t() {}
		};



		template<typename T> class device_array {
			public:
				T *data;
				uint n;
				device_array() : data(0), n(0) {}
				void upload(T *in, uint elems) {
					n = elems;
					cudaMalloc((void**)&data, n*sizeof(T));
					cudaMemcpy(data, in, n*sizeof(T), cudaMemcpyHostToDevice);
				}
				T* download() {
					T *t = new T[n];
					cudaMemcpy(t, data, sizeof(T)*n, cudaMemcpyDeviceToHost);
					return t;
				}
				~device_array() {
					cudaFree(data);
					data = 0;
					n = 0;
				}
		};

		/*! \brief Basic ray bouncer for cuda tracers.
		 *
		 * 	It maintains the gpu accesible intersection data, its job is the
		 * 	same as that of \ref cpu_ray_bouncer.
		 *
		 * 	\note Due to layouts this might get extended.
		 */
		template<box_t__and__tri_t> class gpu_ray_bouncer : virtual public rta::bouncer {
			public:
				declare_traits_types;
				uint w, h;
				triangle_intersection<tri_t> *gpu_last_intersection;

				gpu_ray_bouncer(uint w, uint h) : w(w), h(h), gpu_last_intersection(0) {
					cudaMalloc((void**)&gpu_last_intersection, w*h*sizeof(triangle_intersection<tri_t>));
				}
				~gpu_ray_bouncer() {
					cudaFree(gpu_last_intersection);
				}
		};

		//! A \ref rta::primary_intersection_collector for cuda.
		template<box_t__and__tri_t> class primary_intersection_collector : public gpu_ray_bouncer<forward_traits> {
			public:
				primary_intersection_collector(uint w, uint h) : gpu_ray_bouncer<forward_traits>(w, h) {
				}
				virtual bool trace_further_bounces() {
					return false;
				}
				virtual void bounce() {
				}
				virtual std::string identification() {
					return "cuda primary intersection collector.";
				}
		};

		//! Supplies primary intersections to host programs, e.g. the standard rta program's shader.
		template<box_t__and__tri_t, typename cpu_bouncer> class primary_intersection_downloader : public primary_intersection_collector<forward_traits>, 
		                                                                                          public cpu_bouncer {
			public:
				declare_traits_types;
				typedef primary_intersection_collector<forward_traits> collector_t;

				primary_intersection_downloader(uint w, uint h) 
				: cuda::primary_intersection_collector<forward_traits>(w, h), cpu_bouncer(w, h) {
				}
				virtual bool trace_further_bounces() {
					return false;
				}
				virtual void bounce() {
					cudaMemcpy(this->last_intersection.data, this->gpu_last_intersection,
							   sizeof(triangle_intersection<tri_t>)*collector_t::w*collector_t::h, cudaMemcpyDeviceToHost);
					cudaDeviceSynchronize();
					cpu_bouncer::bounce();
				}
				virtual std::string identification() {
					return "cuda primary intersection collector providing host data.";
				}
		};

		//! An implementation of rta::direct_diffuse_illumination that downloads the intersection data from cuda.
		template<box_t__and__tri_t, typename shader> 
		class direct_diffuse_illumination : public primary_intersection_downloader<forward_traits, rta::primary_intersection_collector<forward_traits> >, 
		                                    public shader {
			public:
				typedef _tri_t tri_t;
				typedef primary_intersection_downloader<forward_traits, rta::primary_intersection_collector<forward_traits> > bouncer_t;
				direct_diffuse_illumination(uint w, uint h) 
				: primary_intersection_downloader<forward_traits, rta::primary_intersection_collector<forward_traits> >(w,h), 
				  shader(w,h, 0) {
					image<triangle_intersection<tri_t>, 1> *li = &this->bouncer_t::last_intersection;
					shader::last_intersection = li;
				}
				virtual void bounce() {
					bouncer_t::bounce();
// 					this->shade();
				}
				virtual std::string identification() { 
					return "direct_diffuse_illumination from host data provided by cuda (primary_intersection_downloader)";
				}
		};

		/*! \brief The basic cuda ray tracing interface.
		 *
		 * 	In contrast to \ref cpu_raytracer this not only keeps a reference
		 * 	to the ray bouncer, but also to the gpu version of the ray
		 * 	generators. This is because we have to pass all the pointers to the
		 * 	kernel at one place.
		 * 	We'll see how this will work out with different layouts...
		 *
		 *  \note prepare_trace initializes the intersection data and this step
		 *  is not integrated into the \ref basic_raytracer's timings.
		 */
		template<box_t__and__tri_t> class gpu_raytracer : public basic_raytracer<forward_traits> {
			public:
				declare_traits_types;
			
			protected:
				gpu_ray_bouncer<forward_traits> *gpu_bouncer;
				gpu_ray_generator *gpu_raygen;

			public:
				gpu_raytracer(rta::ray_generator *raygen, class bouncer *bouncer, basic_acceleration_structure<forward_traits> *as)
				: basic_raytracer<forward_traits>(raygen, bouncer, as), gpu_bouncer(0), gpu_raygen(0) {
					this->ray_bouncer(bouncer);
					this->ray_generator(raygen);
				}
				virtual void ray_bouncer(rta::bouncer *rb) { 
					basic_raytracer<forward_traits>::ray_bouncer(rb);
					gpu_bouncer = dynamic_cast<gpu_ray_bouncer<forward_traits>*>(rb); 
				}
				virtual void ray_generator(rta::ray_generator *rg) {
					basic_raytracer<forward_traits>::ray_generator(rg);
					gpu_raygen = dynamic_cast<gpu_ray_generator*>(rg);
				}
				virtual void prepare_trace() {
					reset_intersections(gpu_bouncer->gpu_last_intersection, gpu_bouncer->w, gpu_bouncer->h);
				}
		};


		
		/*! \brief A <b>stupid hack</b> to tell the rta program that the plugin uses cuda so that it can setup the ray bouncer to collect intersections appropriately.
		 * 	
		 * 	For general plugin implementers this is not nice, sorry. Please take care of setting this up in \ref initialize by assigning to rta::cuda::plugin_with_cuda_loaded.
		 */
		bool using_cuda();
		extern bool plugin_with_cuda_loaded;

		//! @}
	}
}

#else
// for debugging cuda, to avoid stupid mistakes...
#error NO CUDA
#endif

#endif

