#ifndef __RTA_RAYTRAV_H__ 
#define __RTA_RAYTRAV_H__ 

#include "image.h"

#include <algorithm>  // min,max
#include <list>
#include <ostream>

namespace rta {

/*! \defgroup librta Ray Tracing Interface
 *  
 *  \brief Here we define the classes which act as interfaces to the structures you custom ray tracers will have to provide.
 */

/*! \addtogroup librta
 * 	@{
 */

//! state of a ray traversal as executed by a trace thread. not nested in raytracer because of forward-delcaration conflicts.
template<typename _tri_t> struct traversal_state {
	typedef _tri_t tri_t;
	enum { stack_size = 128 };
	uint stack[stack_size];
	int sp; //!< -1 -> stack empty.
	uint x, y;
	triangle_intersection<tri_t> intersection;
	void reset(uint x, uint y) {
		this->x = x, this->y = y;
		sp = 0;
		intersection.reset();
	}
	uint pop() { return stack[sp--]; }
	void push(uint node) { stack[++sp] = node; }
};

////////////////////

/*! \brief The base of all acceleration structures. This is here only for a generic (tri_t, box_t agnostic) plugin interface.
 * 	\attention Please derive from basic_acceleration_structure.
 */
class acceleration_structure {
	public:
		virtual ~acceleration_structure() {}
		virtual std::string identification() = 0;
		//! \attention this is a temporary precaution to catch old code. will be deleted in some time.
		virtual void please_derive_from_basic_acceleration_structure() = 0;
		virtual void dump_acceleration_structure(const std::string &) {}
		virtual void dump_primitives(const std::string &) {}
};

/*! \brief The real interface for acceleration structures. See \ref triangle_ptr.
 */
template<box_t__and__tri_t> class basic_acceleration_structure : public acceleration_structure {
	public:
		declare_traits_types;
		
		/*! This *must* return an array that can be indexed with the ref value stored in a \ref triangle_intersection instance.
		 */
		virtual tri_t* triangle_ptr() = 0;
		//! The length of the array returned by \ref triangle_ptr.
		virtual int triangle_count() = 0;

		/*! Returns the same s \ref triangle_ptr, but where the data can be accessed in the `canonical' way, 
		 * 	i.e. on the cpu side. This is used to be able to shade and compute sphs data for gpu acceleration
		 * 	structures.
		 * 	\attention On gpu or elaborate triangle setup schemes, this may involve copying all the triangle data.
		 */
		virtual tri_t* canonical_triangle_ptr() { return triangle_ptr(); }
		/*! Always call this on the triangle pointer obtained via \ref canonical_triangle_ptr.
		 *  Data will only be freed if it was allocated by that funciton.
		 */
		virtual void free_canonical_triangles(tri_t *data) {}

		//! \attention this is a temporary precaution to catch old code. will be deleted in some time.
		virtual void please_derive_from_basic_acceleration_structure() {}
};

/*! \brief The base of all acceleration structure constructors. This is here only for a generic (tri_t, box_t agnostic) plugin interface.
 * 	\attention Please derive from basic_acceleration_structure.
 */
class acceleration_structure_constructor {
	public:
		virtual ~acceleration_structure_constructor() {}
		virtual std::string identification() = 0;
		
		//! \attention this is a temporary precaution to catch old code. will be deleted in some time.
		virtual void please_derive_from_basic_acceleration_structure_constructor() = 0;
};

/*! The real interface for acceleration structure constructors.
 * 	
 * 	We have this as a separate entity because there are different ways to obtain an acceleration structure of the same type.
 */
template<box_t__and__tri_t> class basic_acceleration_structure_constructor : public acceleration_structure_constructor {
	public:
		declare_traits_types;
		typedef typename tri_t::input_flat_triangle_list_t flat_triangle_list_t;
		virtual basic_acceleration_structure<box_t, tri_t>* build(flat_triangle_list_t *tris) = 0;

		/*! \brief whether the triangle data to build the AS on is supposed to be in host (i.e. system ram) memory.
		 * 	Host data is the most generic, but potentially slower, choice.
		 */
		virtual bool expects_host_triangles() { return true; }
		
		//! \attention this is a temporary precaution to catch old code. will be deleted in some time.
		virtual void please_derive_from_basic_acceleration_structure_constructor() {}
};

////////////////////

/*! The main interface for ray bouncing.
 * 	
 * 	\note 
 * 	We have not yet done much ray bouncing but were mostly concearned with primary rays.
 * 	Therefore, this concept is not really mature.
 */
class bouncer { // sequential calls to raytrace
	public:
		virtual ~bouncer() {}
		virtual void bounce() = 0;
		virtual bool trace_further_bounces() = 0;
		virtual void new_pass() {}
		virtual std::string identification() = 0;
};

/*! \brief extension of \ref bouncer for the case of cpu ray bouncers.
 *  
 *  \note Note too sure if this is really a good idea.
 *
 *  commonly, cpu ray bouncers will require the last triangle intersections.
 *  the ray data is supposed to be stored in the ray generator's structure until the bouncer replaces it.
 *  \todo does this generalize to gpu tracing?
 */
template<box_t__and__tri_t> class cpu_ray_bouncer : virtual public bouncer {
	public:
		declare_traits_types;
	protected:
		image<triangle_intersection<tri_t>, 1> last_intersection;
	public:
		cpu_ray_bouncer(uint w, uint h) : last_intersection(w, h) {
		}
		void save_intersection(uint x, uint y, const triangle_intersection<tri_t> &ti) {
			last_intersection.pixel(x,y) = ti;
		}
};

/*! \brief An implementation of \ref cpu_ray_bouncer which simply collects the closest intersections.
 *  \todo Why is this restricted to cpu tracers?
 *  \todo Is `primary ray' really the correct word or does it sound too much like `generated by a camera'?
 */
template<box_t__and__tri_t> class primary_intersection_collector : public cpu_ray_bouncer<forward_traits> {
	public:
		declare_traits_types;
		primary_intersection_collector(uint w, uint h) : cpu_ray_bouncer<forward_traits>(w,h) {
		}
// 		virtual void bounce_ray(const traversal_state<tri_t> &state, vec3_t *origin, vec3_t *dir) {
// 		}
		virtual bool trace_further_bounces() {
			return false;
		}
		virtual void bounce() {
		}
		virtual std::string identification() {
			return "not implemented yet.";
		}
		const triangle_intersection<tri_t>& intersection(uint x, uint y) {
			return this->last_intersection.pixel(x,y);
		}
};

//! Applies point lights.
template<box_t__and__tri_t> class lighting_collector {
	public:
		typedef _tri_t tri_t;
		typedef typename tri_t::vec3_t vec3_t;
	protected:
		image<unsigned char, 3> res;
		struct light { vec3_t pos, col ; };
		image<triangle_intersection<tri_t>, 1> *last_intersection;
		tri_t *triangles;
	public:
		std::list<light> lights;

		lighting_collector(uint w, uint h, image<triangle_intersection<tri_t>, 1> *li) : res(w, h), last_intersection(li), triangles(0) {
		}
		virtual void reset(vec3f c) {
			for (int y = 0; y < res.h; ++y)
				for (int x = 0; x < res.w; ++x)
					res.pixel(x,y,0) = std::min(int(c.x*255), 255),
					res.pixel(x,y,1) = std::min(int(c.y*255), 255),
					res.pixel(x,y,2) = std::min(int(c.z*255), 255);
		}
		virtual void shade(bool binary_result_only = false) {
			if (lights.size() == 0) return;
			for (int y = 0; y < res.h; ++y)
				for (int x = 0; x < res.w; ++x) {
					triangle_intersection<tri_t> &is = this->last_intersection->pixel(x,y);
					if (!is.valid()) continue;
					vec3_t col = {0,0,0};
					if (binary_result_only)
						col = lights.front().col;
					else {
						tri_t ref = triangles[is.ref];
						for (light &l : lights) {
							// get normal
							vec3_t bc; 
							is.barycentric_coord(&bc);
							const vec3_t &na = normal_a(ref);
							const vec3_t &nb = normal_b(ref);
							const vec3_t &nc = normal_c(ref);
							vec3_t N, p;
							barycentric_interpolation(&N, &bc, &na, &nb, &nc);
							// get vertex pos
							const vec3_t &va = vertex_a(ref);
							const vec3_t &vb = vertex_b(ref);
							const vec3_t &vc = vertex_c(ref);
							barycentric_interpolation(&p, &bc, &va, &vb, &vc);
							// compute lambert
							vec3_t L = l.pos;
							sub_components_vec3f(&L, &L, &p);
							normalize_vec3f(&L);
							normalize_vec3f(&N);
							float_t dot = std::max(dot_vec3f(&N, &L), (float_t)0);
							vec3_t c;
							mul_vec3f_by_scalar(&c, &l.col, dot);
							add_components_vec3f(&col, &col, &c);
						}
					}
					res.pixel(x,y,0) = std::max(0, std::min(int(col.x*255), 255));
					res.pixel(x,y,1) = std::max(0, std::min(int(col.y*255), 255));
					res.pixel(x,y,2) = std::max(0, std::min(int(col.z*255), 255));
			}
		}
		virtual void add_pointlight(const vec3_t &at, const vec3_t &col) {
			lights.push_back({at,col});
		}
		virtual void save(const std::string &filename) {
			res.save_png(filename);
		}
		void triangle_ptr(tri_t *t) { triangles = t; }
};

/*! \brief An actual \ref bouncer implementation which traces primary rays (see \ref primary_intersection_collector for confusion)
 *  	and can apply point light illumination on the result.
 */
template<box_t__and__tri_t, typename shader> class direct_diffuse_illumination : public cpu_ray_bouncer<forward_traits>, public shader {
	public:
		typedef _tri_t tri_t;
		typedef cpu_ray_bouncer<forward_traits> bouncer_t;
		// on the initialization order, see http://www.informit.com/guides/content.aspx?g=cplusplus&seqNum=169
		direct_diffuse_illumination(uint w, uint h) : cpu_ray_bouncer<forward_traits>(w,h), shader(w,h, 0) {
			image<triangle_intersection<tri_t>, 1> *li = &this->bouncer_t::last_intersection;
			shader::last_intersection = li;
		}
		virtual void bounce() {
// 			for (int y = 0; y < res.h; ++y)
// 				for (int x = 0; x < res.w; ++x)
// 					res.pixel(x,y,1) = this->last_intersection.pixel(x,y).valid() ? 255 : 0;
// 			this->shade();
		}
		virtual bool trace_further_bounces() {
			return false;
		}
		virtual std::string identification() { return "direct_diffuse_illumination"; }
};

////////////////////

//! The interface for ray generators. See \ref generate_rays.
class ray_generator {
	public:
		image<vec3f, 2> raydata;
		//! this stores the maximal length of each ray. initialize to FLT_MAX if you want your rays to extent infinitely.
		image<float, 1> ray_max_t;
		ray_generator(unsigned int res_x, unsigned int res_y) : raydata(res_x, res_y), ray_max_t(res_x, res_y) {
		}
		virtual ~ray_generator() {}
		/*! \brief The actual interface function.
		 * 
		 * 	\note The whole image,x,y stuff is to be understood as a means of storage and 
		 * 		mapping from trace thread pixel index to ray, only.
		 */
		virtual void generate_rays() = 0;
		inline uint res_x() const                           { return raydata.w; }
		inline uint res_y() const                           { return raydata.h; }
		inline vec3f* origin(uint x, uint y)                { return &raydata.pixel(x, y, 0); }
		inline const vec3f* origin(uint x, uint y) const    { return &raydata.pixel(x, y, 0); }
		inline vec3f* direction(uint x, uint y)             { return &raydata.pixel(x, y, 1); }
		inline const vec3f* direction(uint x, uint y) const { return &raydata.pixel(x, y, 1); }
 		inline float& max_t(uint x, uint y)                 { return ray_max_t.pixel(x, y, 0); }
		inline const float& max_t(uint x, uint y) const     { return ray_max_t.pixel(x, y, 0); }
		virtual std::string identification() = 0;
		virtual void dump_rays(const std::string &filename) {}
		//! this is here just to find bugs in legacy ray generators. subject to removal.
		virtual void dont_forget_to_initialize_max_t() = 0;
};

//! A ray generator according to the setup presented in Shirley's book.
class cam_ray_generator_shirley : public ray_generator {
	protected:
		float aspect, fovy;
		vec3f position;
		vec3f dir;
		vec3f up;

		void generate_ray_dir(vec3f *dir, float plane_w, float plane_h, unsigned int x, unsigned int y, unsigned int w, unsigned h, const vec3f *view_dir, const vec3f *view_up) { // {{{
			float u_s = ((float)x+0.5f)/(float)w * 2.0f - 1.0f;	// \in (-1,1)
			float v_s = ((float)y+0.5f)/(float)h * 2.0f - 1.0f;
			u_s = plane_w * u_s / 2.0f;	// \in (-pw/2, pw/2)
			v_s = plane_h * v_s / 2.0f;

			vec3f W, TxW, U, V;
			div_vec3f_by_scalar(&W, view_dir, length_of_vec3f(view_dir));
			cross_vec3f(&TxW, view_up, &W);
			div_vec3f_by_scalar(&U, &TxW, length_of_vec3f(&TxW));
			cross_vec3f(&V, &W, &U);

			mul_vec3f_by_scalar(dir, &U, u_s);
			vec3f tmp;
			mul_vec3f_by_scalar(&tmp, &V, v_s);
			add_components_vec3f(dir, dir, &tmp);
			add_components_vec3f(dir, dir, &W);
			normalize_vec3f(dir);
		} // }}}

		void generate_ray_dir_from_cam_parameters(vec3f *dir, float fovy, float aspect, unsigned int x, unsigned int y, unsigned int w, unsigned int h, const vec3f *view_dir, const vec3f *view_up) {
			fovy /= 2.0;
			float height = tanf(M_PI * fovy / 180.0f);
			float width = aspect * height;
			generate_ray_dir(dir, 2*width, 2*height, x, y, w, h, view_dir, view_up);
		}

	public:
		cam_ray_generator_shirley(uint res_x, uint res_y) : ray_generator(res_x, res_y), aspect(float(res_x)/float(res_y)) {
		}
		void setup(vec3f *new_position, vec3f *new_dir, vec3f *new_up, float new_fovy) {
			position = *new_position;
			dir = *new_dir;
			up = *new_up;
			fovy = new_fovy;
		}
		virtual void generate_rays() {
			for (uint y = 0; y < res_y(); ++y)
				for (uint x = 0; x < res_x(); ++x) {
					*origin(x, y) = position;
					generate_ray_dir_from_cam_parameters(direction(x, y), fovy, aspect, x, y, res_x(), res_y(), &dir, &up);
					max_t(x, y) = FLT_MAX;
				}
		}
		virtual std::string identification() { return "ray generator according to shirley."; }
		virtual void dump_rays(const std::string &filename) {
			std::ofstream out(filename.c_str());
			int w = res_x(), h = res_y();
			out << "ray\n" << w << " " << h << "\n";
			for (int y = 0; y < h; ++y)
				for (int x = 0; x < w; ++x) {
					vec3f *o = origin(x, y),
						  *d = direction(x, y);
					out << y*w+x << " " << 2 << " "
						<< o->x << " " << o->y << " " << o->z << " "
						<< d->x << " " << d->y << " " << d->z << " "
						<< "0 " << max_t(x, y) << "\n";
				}
		}
		virtual void dont_forget_to_initialize_max_t() {}
};

////////////////////

//! The general ray tracing interface.
class raytracer {
	public:
// 		virtual void setup_rays() = 0;
// 		virtual void prepare_bvh_for_tracing() = 0;
		virtual void trace() = 0;
		virtual std::string identification() = 0;
		virtual raytracer* copy() = 0;

		//! returns true if the tracer honores the ray_generator's max_t.
		virtual bool supports_max_t() = 0;
};

/*! \brief A framework ray tracer extension that implements the whole \ref ray_generator, \ref raytracer, \ref bouncer cycle, 
 *		   as well as the taking of  timings.
 */
template<box_t__and__tri_t> class basic_raytracer : public raytracer {
	public:
		declare_traits_types;

	protected:
		rta::ray_generator *raygen;
		rta::bouncer *bouncer;
		virtual float trace_rays() = 0;
		rta::basic_acceleration_structure<forward_traits> *accel_struct;

	public:
		basic_raytracer(rta::ray_generator *raygen, class bouncer *bouncer, basic_acceleration_structure<forward_traits> *as)
		: raygen(raygen), bouncer(bouncer), accel_struct(as) {
			if (bouncer)
				basic_raytracer::ray_bouncer(bouncer);
		}
// 		virtual void setup_rays() { // potentially uploads ray data to the gpu
// 		}
// 		virtual void prepare_bvh_for_tracing() { // potentially uploads the bvh to the gpu? will ich das?
// 		}
		//! \brief this can be used, e.g., to initialize gpu buffers. time spent is not accumulated in \ref timings.
		virtual void prepare_trace() {
		}
		virtual void trace() {
			raygen->generate_rays();
			bouncer->new_pass();
			timings.clear();
			do {
				prepare_trace();
				float ms = trace_rays();
				timings.push_back(ms);
				bouncer->bounce();
			} while (bouncer->trace_further_bounces());
		}
		/*! This holds the timings for each bounce of the last call to \ref trace.
		 *  \attention Each invocation of \ref trace clears this structure, i.e. if you want to accumulate different traces 
		 *  (e.g. primary ray performance) you have to copy the timings after each \ref trace.
		 */
		std::vector<float> timings;
		virtual void ray_generator(rta::ray_generator *rg) { raygen = rg; }
		virtual void ray_bouncer(rta::bouncer *rb) { bouncer = rb; }
		virtual basic_raytracer* copy() = 0;
		
		virtual void acceleration_structure(rta::basic_acceleration_structure<forward_traits> *as) {
			accel_struct = as;
		}
		rta::basic_acceleration_structure<forward_traits>* acceleration_structure() {
			return accel_struct;
		}
};

/*! \brief Extension to basic_raytracer to be able to access cpu_ray_bouncer.
 */
template<box_t__and__tri_t> class cpu_raytracer : public basic_raytracer<forward_traits> {
	public:
		declare_traits_types;
	
	protected:
		cpu_ray_bouncer<forward_traits> *cpu_bouncer;

	public:
		cpu_raytracer(rta::ray_generator *raygen, class bouncer *bouncer, basic_acceleration_structure<forward_traits> *as)
		: basic_raytracer<forward_traits>(raygen, bouncer, as), cpu_bouncer(0) {
			if (bouncer)
				this->ray_bouncer(bouncer);
		}
		virtual void ray_bouncer(rta::bouncer *rb) { 
			basic_raytracer<forward_traits>::ray_bouncer(rb);
			cpu_bouncer = dynamic_cast<cpu_ray_bouncer<forward_traits>*>(rb); 
		}
		virtual void force_cpu_bouncer(rta::cpu_ray_bouncer<forward_traits> *rb) {
			std::cerr << "> > > WARNING > > > you are using cpu_raytracer::force_cpu_bouncer." << std::endl;
			std::cerr << "> > > WARNING > > > please make sure if your code runs without this call." << std::endl;
			std::cerr << "> > > WARNING > > > if not, get back to me!" << std::endl;
			cpu_bouncer = rb;
		}

};

////////////////////

//! @}

}


#endif

