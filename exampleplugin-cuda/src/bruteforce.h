#include <librta/librta.h>
#include <librta/cuda.h>
#include <librta/wall-timer.h>

using namespace std;
using namespace rta;

//// brute force implementation

namespace rta {
	namespace cuda {
		namespace example {
			
			// see bruteforce.cu
			void trace_bruteforce(simple_triangle *triangles, int n, vec3f *ray_orig, vec3f *ray_dir, float *max_t, int w, int h, triangle_intersection<simple_triangle> *is);
	
			/*! \brief Implementation of the \ref basic_acceleration_structure interface for brute force ray tracing.
			 *  \note For the BF case this is bogus, of course :)
			 */
			template<box_t__and__tri_t> class bruteforce_dummy_accel_struct : public basic_acceleration_structure<forward_traits> {
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
	
			/*! \brief Constructor for the dummy acceleration structure, \ref bruteforce_dummy_accel_struct, implementing \ref basic_acceleration_structure_constructor.
			 *  \note For the case of BF RT this is really bogus.
			 *  \note We forward our template parameters but it is not really required to happen this way, so feel free.
			 */
			template<box_t__and__tri_t> class bruteforce_dummy_as_ctor : public basic_acceleration_structure_constructor<forward_traits> {
			public:
				declare_traits_types;
	
				/*! We just pass on the triangle list; your version may be permuted or made larger by the introduction of duplicates.
				 * 	The triangle data returned by the acceleration structure must reflect those changes or 
				 * 		return intersection information respecting the original configuration (which would probably be harder to do).
				 */
				bruteforce_dummy_accel_struct<forward_traits>* build(typename tri_t::input_flat_triangle_list_t *tris) {
					cuda_ftl tri_data(*tris);
					bruteforce_dummy_accel_struct<forward_traits> *as = new bruteforce_dummy_accel_struct<forward_traits>;
					std::vector<tri_t> tmp;
					for (int i = 0; i < tri_data.triangles; ++i)
						tmp.push_back(tri_data.triangle[i]);
					as->take_triangle_array(tmp);
					return as;
				}
				
				virtual std::string identification() { return "brute force ray tracer's dummy acceleration structure ctor"; }
			};
	
			/*! \brief Implemantation of the \ref raytracer interface via \ref basic_raytracer to integrate nicely into the evaluation code.
			 */
			template<box_t__and__tri_t> class bruteforce_tracer : public gpu_raytracer<forward_traits> {
				declare_traits_types;
	
				// we "use" these here to avoid more ugly syntax in the functions using them.
// 				using basic_raytracer<forward_traits>::raygen;
// 				using basic_raytracer<forward_traits>::cpu_bouncer;
	
				bruteforce_dummy_accel_struct<forward_traits> *as;
	
			public:
				bruteforce_tracer(ray_generator *gen, bouncer *b, bruteforce_dummy_accel_struct<forward_traits> *as)
				: gpu_raytracer<forward_traits>(gen, b, as), as(as) {
				}
	
				virtual float trace_rays() {
					wall_time_timer wtt; wtt.start();
					trace_bruteforce(as->gpu_triangles, as->triangle_count(), 
					                 (vec3f*)this->gpu_raygen->gpu_origin, (vec3f*)this->gpu_raygen->gpu_direction, this->gpu_raygen->gpu_maxt,
									 this->gpu_raygen->w, this->gpu_raygen->h,
					                 this->gpu_bouncer->gpu_last_intersection);
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


