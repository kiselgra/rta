#include <librta/librta.h>
#include <librta/wall-timer.h>

#include </opt/cuda/include/cuda.h>
#include </opt/cuda/include/cuda_runtime_api.h>

using namespace std;
using namespace rta;

//// cuda stuff

namespace rta {
	namespace cuda {
		class raygen_buffer_addon {
			public:
				float *gpu_origin, *gpu_direction;
				float *gpu_maxt;

				raygen_buffer_addon(int w, int h) : gpu_origin(0), gpu_direction(0),  gpu_maxt(0) {
					cout << "allocating " << ((w*h*7*sizeof(float))>>20) << "M of ray storage on GPU" << endl;
					cudaMalloc((void**)&gpu_origin, w*h*3*sizeof(float));
					cudaMalloc((void**)&gpu_direction, w*h*3*sizeof(float));
					cudaMalloc((void**)&gpu_maxt, w*h*sizeof(float));
				}
				~raygen_buffer_addon() {
					cudaFree(gpu_origin);
					cudaFree(gpu_direction);
					cudaFree(gpu_maxt);
				}
		};

		template<typename base_rgen> class raygen_with_buffer : public base_rgen, protected raygen_buffer_addon {
			public:
				vec3_t *tmp0, *tmp1;

				raygen_with_buffer(int w, int h) : base_rgen(w, h), raygen_buffer_addon(w, h), tmp0(0), tmp1(0) {
					tmp0 = new vec3_t[3*w*h];
					tmp1 = new vec3_t[3*w*h];
				}
				~raygen_with_buffer() {
					delete [] tmp0;
					delete [] tmp1;
				}
				virtual void generate_rays() {
					base_rgen::generate_rays();
					for (int y = 0; y < base_rgen::raydata.h; ++y)
						for (int x = 0; x < base_rgen::raydata.w; ++x)
							tmp0[y*base_rgen::raydata.w+x] = *base_rgen::origin(x,y),
							tmp1[y*base_rgen::raydata.w+x] = *base_rgen::direction(x,y);
					cudaMemcpy(gpu_origin, tmp0, base_rgen::raydata.w*base_rgen::raydata.h*3*sizeof(float), cudaMemcpyHostToDevice);
					cudaMemcpy(gpu_direction, tmp1, base_rgen::raydata.w*base_rgen::raydata.h*3*sizeof(float), cudaMemcpyHostToDevice);
				}
				virtual std::string identification() {
					return "cuda raygen adaptor for " + base_rgen::identification();
				}
				virtual void dont_forget_to_initialize_max_t() {}
		};

		
	}
}

//// brute force implementation


namespace rta {
	namespace cuda {
		namespace example {
	
			/*! \brief Implementation of the \ref acceleration_structure interface for brute force ray tracing.
			 *  \note For the BF case this is bogus, of course :)
			 */
			template<box_t__and__tri_t> class bruteforce_dummy_accel_struct : public acceleration_structure<forward_traits> {
			public:
				declare_traits_types;
	
				std::vector<tri_t> triangles;
				tri_t *gpu_triangles;
	
				/*! take the triangles stored in the array. 
				 * 	\attention does so destructively!
				 */
				void take_triangle_array(std::vector<tri_t> &t) {
					triangles.swap(t);
					cout << "allocating " << ((sizeof(tri_t)*triangles.size())>>20) << "M of triangle storage on GPU" << endl;
					cudaMalloc((void**)&gpu_triangles, sizeof(tri_t)*triangles.size());
					cudaMemcpy(gpu_triangles, &triangles[0], sizeof(tri_t)*triangles.size(), cudaMemcpyHostToDevice);
				}
	
				tri_t* triangle_ptr() { return &triangles[0]; }
				int triangle_count() { return triangles.size(); }
	
				virtual std::string identification() { return "cuda dummy accel struct for brute force ray tracing"; }
			};
	
			/*! \brief Constructor for the dummy acceleration structure, \ref bruteforce_dummy_accel_struct, implementing \ref acceleration_structure_constructor.
			 *  \note For the case of BF RT this is really bogus.
			 *  \note We forward our template parameters but it is not really required to happen this way, so feel free.
			 */
			template<box_t__and__tri_t> class bruteforce_dummy_as_ctor : public acceleration_structure_constructor<forward_traits> {
			public:
				declare_traits_types;
	
				/*! We just pass on the triangle list; your version may be permuted or made larger by the introduction of duplicates.
				 * 	The triangle data returned by the acceleration structure must reflect those changes or 
				 * 		return intersection information respecting the original configuration (which would probably be harder to do).
				 */
				bruteforce_dummy_accel_struct<forward_traits>* build(flat_triangle_list *tris) {
					bruteforce_dummy_accel_struct<forward_traits> *as = new bruteforce_dummy_accel_struct<forward_traits>;
					std::vector<tri_t> tmp;
					for (int i = 0; i < tris->triangles; ++i)
						tmp.push_back(tris->triangle[i]);
					as->take_triangle_array(tmp);
					return as;
				}
				
				virtual std::string identification() { return "brute force ray tracer's dummy acceleration structure ctor"; }
			};
	
			/*! \brief Implemantation of the \ref raytracer interface via \ref basic_raytracer to integrate nicely into the evaluation code.
			 */
			template<box_t__and__tri_t> class bruteforce_tracer : public basic_raytracer<forward_traits> {
				declare_traits_types;
	
				// we "use" these here to avoid more ugly syntax in the functions using them.
				using basic_raytracer<forward_traits>::raygen;
				using basic_raytracer<forward_traits>::cpu_bouncer;
	
				bruteforce_dummy_accel_struct<forward_traits> *as;
	
			public:
				bruteforce_tracer(ray_generator *gen, bouncer *b, bruteforce_dummy_accel_struct<forward_traits> *as) : basic_raytracer<forward_traits>(gen, b, as), as(as) {
				}
	
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
// 					trace_bruteforce(as->gpu_triangles, 
					/*
					traversal_state<tri_t> state;
					for (uint y = 0; y < raygen->res_y(); ++y) {
						if (cmdline.verbose)
							cout << "~" << flush;
						for (uint x = 0; x < raygen->res_x(); ++x) {
							state.reset(x,y);
							trace_ray(state, raygen->origin(x,y), raygen->direction(x,y));
							cpu_bouncer->save_intersection(x,y,state.intersection);
						}
					}
					*/
					float ms = wtt.look();
					return ms;
				}
	
				/*! The separation between \ref trace_rays and \ref trace_ray is used throughout the code.
				 *  \ref trace_rays is usually the same code, it is, however, not integrated into the interface
				 *  to prevent the \ref trace_ray from requiring a virtual call for each single ray.
				 *
				 *  We hope this is a price worth paying :)
				 */
				void trace_ray(traversal_state<tri_t> &state, const vec3_t *origin, const vec3_t *dir) {
					tri_t *tri = as->triangle_ptr();
					uint tris = as->triangle_count();
	
					for (int i = 0; i < tris; ++i) {
						triangle_intersection<tri_t> is(i);
						if (intersect_tri_opt(tri[i], origin, dir, is)) {
							if (is.t < state.intersection.t)
								state.intersection = is;
						}
					}
				}
				virtual std::string identification() { return "brute force ray tracer"; }
				virtual bruteforce_tracer* copy() { return new bruteforce_tracer(*this); }
				virtual bool supports_max_t() { return false; }
			};
		}
	}
}


