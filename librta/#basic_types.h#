#ifndef __BASIC_TYPES_H__ 
#define __BASIC_TYPES_H__ 

#include <ostream>
#include <float.h>

#include <libmcm/vectors.h>

#include "template-magic.h"

namespace rta {

	typedef unsigned int uint;
	typedef float float_t;
	typedef vec3f vec3_t;
	typedef vec4f vec4_t;

	
	struct simple_triangle {
		vec3_t a, b, c;
		vec3_t na, nb, nc;
	};

	struct simple_aabb {
		vec3_t min, max;
	};

	template<typename _tri_t> struct triangle_intersection {
		typedef _tri_t tri_t;
		float_t t, beta, gamma;
		uint ref;
		triangle_intersection() : t(FLT_MAX), ref(0) {}
		triangle_intersection(uint t) : t(FLT_MAX), ref(t) {}
		bool valid() const { return t != FLT_MAX; }
		void reset() { t = FLT_MAX; ref = 0; }
		void barycentric_coord(vec3_t *to) {
			to->x = 1.0 - beta - gamma;
			to->y = beta;
			to->z = gamma;
		}
	};


	struct flat_triangle_list {
		uint triangles;
		simple_triangle *triangle;
		flat_triangle_list() : triangles(0), triangle(0) {}
		flat_triangle_list(int size) : triangles(size), triangle(0) { triangle = new simple_triangle[size]; }
	};


	// be aware that these are defined everywhere!
	#define box_t__and__tri_t typename _box_t, typename _tri_t
	#define forward_traits _box_t, _tri_t
	#define declare_traits_types typedef _box_t box_t; typedef _tri_t tri_t;
	#define traits_of(X) typename X::box_t, typename X::tri_t



	#define invalid_instantiation_message "Invalid instantiation of a function allowed for certain types, only."
	#define invalid_instantiation(TTT) static_assert(fail<TTT>::result, invalid_instantiation_message)
	#define invalid_t void
	
	
	// accessors

	
	//! access vector components by number
	template<typename T, unsigned int N> struct comp_impl { invalid_instantiation(T); };
	template<typename T> struct comp_impl<T, 0> { 
		static inline float_t& ref(T &t) { return x_comp(t); } 
		static inline const float_t &ref(const T &t) { return x_comp(t); }
	};
	template<typename T> struct comp_impl<T, 1> { 
		static inline float_t& ref(T &t) { return y_comp(t); } 
		static inline const float_t &ref(const T &t) { return y_comp(t); }
	};
	template<typename T> struct comp_impl<T, 2> { 
		static inline float_t& ref(T &t) { return z_comp(t); } 
		static inline const float_t &ref(const T &t) { return z_comp(t); }
	};
	//! access vector components by number
	template<unsigned int N, typename T> inline       float_t& comp(T &t)       { return comp_impl<T, N>::ref(t); }
	template<unsigned int N, typename T> inline const float_t& comp(const T &t) { return comp_impl<T, N>::ref(t); }

	//! acess vector components by name
	template<typename T> inline       float_t& x_comp(T &t)            { invalid_instantiation(T); return 0; }
	template<>           inline       float_t& x_comp(vec3_t &t)       { return t.x; }
	template<>           inline       float_t& x_comp(vec4_t &t)       { return t.x; }
	template<typename T> inline const float_t& x_comp(const T &t)      { invalid_instantiation(T); return 0; }
	template<>           inline const float_t& x_comp(const vec3_t &t) { return t.x; }
	template<>           inline const float_t& x_comp(const vec4_t &t) { return t.x; }
	
	//! acess vector components by name
	template<typename T> inline       float_t& y_comp(T &t)            { invalid_instantiation(T); return 0; }
	template<>           inline       float_t& y_comp(vec3_t &t)       { return t.y; }
	template<>           inline       float_t& y_comp(vec4_t &t)       { return t.y; }
	template<typename T> inline const float_t& y_comp(const T &t)      { invalid_instantiation(T); return 0; }
	template<>           inline const float_t& y_comp(const vec3_t &t) { return t.y; }
	template<>           inline const float_t& y_comp(const vec4_t &t) { return t.y; }
	
	//! acess vector components by name
	template<typename T> inline       float_t& z_comp(T &t)            { invalid_instantiation(T); return 0; }
	template<>           inline       float_t& z_comp(vec3_t &t)       { return t.z; }
	template<>           inline       float_t& z_comp(vec4_t &t)       { return t.z; }
	template<typename T> inline const float_t& z_comp(const T &t)      { invalid_instantiation(T); return 0; }
	template<>           inline const float_t& z_comp(const vec3_t &t) { return t.z; }
	template<>           inline const float_t& z_comp(const vec4_t &t) { return t.z; }
	
	//! acess vector w components by name
	template<typename T> inline       float_t& w_comp(T &t)            { invalid_instantiation(T); return 0; }
	template<>           inline       float_t& w_comp(vec4_t &t)       { return t.w; }
	template<typename T> inline const float_t& w_comp(const T &t)      { invalid_instantiation(T); return 0; }
	template<>           inline const float_t& w_comp(const vec4_t &t) { return t.w; }

	//! access a triangle's vertices by name - a
	template<typename T> inline       vec3_t& vertex_a(T &t)                     { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& vertex_a(simple_triangle &t)       { return t.a; }
	template<typename T> inline const vec3_t& vertex_a(const T &t)               { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& vertex_a(const simple_triangle &t) { return t.a; }

	//! access a triangle's vertices by name - b
	template<typename T> inline       vec3_t& vertex_b(T &t)                     { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& vertex_b(simple_triangle &t)       { return t.b; }
	template<typename T> inline const vec3_t& vertex_b(const T &t)               { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& vertex_b(const simple_triangle &t) { return t.b; }

	//! access a triangle's vertices by name - c
	template<typename T> inline       vec3_t& vertex_c(T &t)                     { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& vertex_c(simple_triangle &t)       { return t.c; }
	template<typename T> inline const vec3_t& vertex_c(const T &t)               { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& vertex_c(const simple_triangle &t) { return t.c; }
	
	//! access a triangle's normal by name - a
	template<typename T> inline       vec3_t& normal_a(T &t)                     { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& normal_a(simple_triangle &t)       { return t.na; }
	template<typename T> inline const vec3_t& normal_a(const T &t)               { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& normal_a(const simple_triangle &t) { return t.na; }

	//! access a triangle's normal by name - b
	template<typename T> inline       vec3_t& normal_b(T &t)                     { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& normal_b(simple_triangle &t)       { return t.nb; }
	template<typename T> inline const vec3_t& normal_b(const T &t)               { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& normal_b(const simple_triangle &t) { return t.nb; }

	//! access a triangle's normal by name - c
	template<typename T> inline       vec3_t& normal_c(T &t)                     { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& normal_c(simple_triangle &t)       { return t.nc; }
	template<typename T> inline const vec3_t& normal_c(const T &t)               { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& normal_c(const simple_triangle &t) { return t.nc; }

	//! access an aabb's vertices by name - min
	template<typename T> inline       vec3_t& min(T &bb)                 { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& min(simple_aabb &bb)       { return bb.min; }
	template<typename T> inline const vec3_t& min(const T &bb)           { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& min(const simple_aabb &bb) { return bb.min; }
	
	//! access an aabb's vertices by name - max
	template<typename T> inline       vec3_t& max(T &bb)                 { invalid_instantiation(T); return vec3_t(); }
	template<>           inline       vec3_t& max(simple_aabb &bb)       { return bb.max; }
	template<typename T> inline const vec3_t& max(const T &bb)           { invalid_instantiation(T); return vec3_t(); }
	template<>           inline const vec3_t& max(const simple_aabb &bb) { return bb.max; }



	// output


	inline std::ostream& operator<<(std::ostream &o, const vec3_t &v) { o << "(vec " << v.x << " " << v.y << " " << v.z << ")"; return o; }
	inline std::ostream& operator<<(std::ostream &o, const simple_aabb &b) { o << "(aabb " << b.min << " " << b.max << ")"; return o; }
	inline std::ostream& operator<<(std::ostream &o, const simple_triangle &t) { o << "(tri " << t.a << " " << t.b << " " << t.c << ")"; return o; }


	#undef invalid_instantiation
	#undef invalid_instantiation_message
}

#include "tri.h"
#include "aabb.h"

#endif

