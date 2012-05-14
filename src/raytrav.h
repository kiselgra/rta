#ifndef __RTA_RAYTRAV_H__ 
#define __RTA_RAYTRAV_H__ 

#include <algorithm>  // min,max

namespace rta {


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

template<box_t__and__tri_t> struct acceleraton_structure {
	declare_traits_types;
	virtual ~acceleraton_structure() {}
	virtual tri_t* triangle_ptr() = 0;
	virtual int triangle_count() = 0;
};

template<box_t__and__tri_t> struct acceleration_structure_constructor {
	declare_traits_types;
	acceleraton_structure<box_t, tri_t>* build(flat_triangle_list *tris);
};

////////////////////

class bouncer { // sequential calls to raytrace
	public:
		virtual void bounce() = 0;
		virtual bool trace_further_bounces() = 0;
		virtual void new_pass() {}
};

/*! commonly, cpu ray bouncers will require the last triangle intersections.
 *  the ray data is supposed to be stored in the ray generator's structure until the bouncer replaces it.
 *  todo: does this generalize to gpu tracing?
 */
template<box_t__and__tri_t> class cpu_ray_bouncer : public bouncer {
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

template<box_t__and__tri_t> class primary_intersection_collector : public cpu_ray_bouncer<forward_traits> {
	public:
		primary_intersection_collector(uint w, uint h) : cpu_ray_bouncer<forward_traits>(w,h) {
		}
// 		virtual void bounce_ray(const traversal_state<tri_t> &state, vec3_t *origin, vec3_t *dir) {
// 		}
		virtual bool trace_further_bounces() {
			return false;
		}
};

template<box_t__and__tri_t> class binary_png_tester : public cpu_ray_bouncer<forward_traits> {
		image<unsigned char, 3> res;
		struct light { vec3_t pos, col ; };
	public:
		std::list<light> lights;
		typedef _tri_t tri_t;
		binary_png_tester(uint w, uint h) : cpu_ray_bouncer<forward_traits>(w,h), res(w,h) {
			reset();
		}
		void reset() {
			for (int y = 0; y < res.h; ++y)
				for (int x = 0; x < res.w; ++x)
					for (int c = 0; c < 3; ++c)
						res.pixel(x,y,c) = 0;
		}
		void add_pointlight(const vec3_t &at, const vec3_t &col) {
			lights.push_back({at,col});
		}
		void shade() {
			if (lights.size() == 0) return;
			for (int y = 0; y < res.h; ++y)
				for (int x = 0; x < res.w; ++x) {
					triangle_intersection<tri_t> &is = this->last_intersection.pixel(x,y);
					if (!is.valid()) continue;
					vec3f col = {0,0,0};
					for (light &l : lights) {
						// get normal
						vec3_t bc; 
						is.barycentric_coord(&bc);
						const vec3_t &na = normal_a(*is.ref);
						const vec3_t &nb = normal_b(*is.ref);
						const vec3_t &nc = normal_c(*is.ref);
						vec3_t N, p;
					   	barycentric_interpolation(&N, &bc, &na, &nb, &nc);
						// get vertex pos
						const vec3_t &va = vertex_a(*is.ref);
						const vec3_t &vb = vertex_b(*is.ref);
						const vec3_t &vc = vertex_c(*is.ref);
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
// 						add_components_vec3f(&col, &col, &N);
					}
					res.pixel(x,y,0) = std::min(int(col.x*255), 255);
					res.pixel(x,y,1) = std::min(int(col.y*255), 255);
					res.pixel(x,y,2) = std::min(int(col.z*255), 255);
				}
		}
// 		virtual void bounce_ray(const traversal_state &state, vec3_t *origin, vec3_t *dir) {
// 			res.pixel(state.x,state.y,1) = (state.intersection.valid() ? 255 : 0);
// 		}
		virtual void bounce() {
			for (int y = 0; y < res.h; ++y)
				for (int x = 0; x < res.w; ++x)
					res.pixel(x,y,1) = this->last_intersection.pixel(x,y).valid() ? 255 : 0;
		}
		virtual bool trace_further_bounces() {
			return false;
		}
		void save(const std::string &filename) {
			res.save_png(filename);
		}
};

////////////////////

class ray_generator {
	public:
		image<vec3f, 2> raydata;
		ray_generator(unsigned int res_x, unsigned int res_y) : raydata(res_x, res_y) {
		}
		virtual void generate_rays() = 0;
		inline uint res_x() const                           { return raydata.w; }
		inline uint res_y() const                           { return raydata.h; }
		inline vec3f* origin(uint x, uint y)                { return &raydata.pixel(x, y, 0); }
		inline const vec3f* origin(uint x, uint y) const    { return &raydata.pixel(x, y, 0); }
		inline vec3f* direction(uint x, uint y)             { return &raydata.pixel(x, y, 1); }
		inline const vec3f* direction(uint x, uint y) const { return &raydata.pixel(x, y, 1); }
};

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
				}
		}
};

////////////////////

class raytracer {
	public:
		virtual void setup_rays() = 0;
		virtual void prepare_bvh_for_tracing() = 0;
		virtual void trace() = 0;
};

template<box_t__and__tri_t> class basic_raytracer : public raytracer {
// 		struct trace_thread {
// 		};
	protected:
		ray_generator *raygen;
		class bouncer *bouncer;
		cpu_ray_bouncer<forward_traits> *cpu_bouncer;
		virtual float trace_rays() = 0;
	public:
		declare_traits_types;
		acceleraton_structure<forward_traits> *accel_struct;
		basic_raytracer(ray_generator *raygen, class bouncer *bouncer, acceleraton_structure<forward_traits> *as) : raygen(raygen), bouncer(bouncer), cpu_bouncer(dynamic_cast<cpu_ray_bouncer<forward_traits>*>(bouncer)), accel_struct(as) {
		}
		virtual void setup_rays() { // potentially uploads ray data to the gpu
		}
		virtual void prepare_bvh_for_tracing() { // potentially uploads the bvh to the gpu? will ich das?
		}
		virtual void trace() {
			raygen->generate_rays();
			timings.clear();
			do {
				float ms = trace_rays();
				timings.push_back(ms);
				bouncer->bounce();
			} while (bouncer->trace_further_bounces());
		}
		std::vector<float> timings;
};

////////////////////


}


#endif

