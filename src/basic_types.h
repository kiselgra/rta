#ifndef __BASIC_TYPES_H__ 
#define __BASIC_TYPES_H__ 

#include <ostream>

#include <libmcm/vectors.h>

#include "template-magic.h"

namespace rta {

	typedef unsigned int uint;
	typedef float float_t;
	typedef vec3f vec3_t;

	
	struct simple_triangle {
		vec3_t a, b, c;
		vec3_t na, nb, nc;
	};

	struct simple_aabb {
		vec3_t min, max;
	};



	#define invalid_instantiation_message "Invalid instantiation of a function allowed for certain types, only."
	#define invalid_instantiation static_assert(fail<T>::result, invalid_instantiation_message)
	
	
	// accessors

	
	//! access vector components by number
	template<typename T, unsigned int N> struct comp_impl { invalid_instantiation; };
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
	template<unsigned int N, typename T> inline       float_t& comp(T &t)       { comp_impl<T, N>::ref(t); }
	template<unsigned int N, typename T> inline const float_t& comp(const T &t) { comp_impl<T, N>::ref(t); }

	//! acess vector components by name
	template<typename T> inline       float_t& x_comp(T &t)            { invalid_instantiation; }
	template<>           inline       float_t& x_comp(vec3_t &t)       { return t.x; }
	template<typename T> inline const float_t& x_comp(const T &t)      { invalid_instantiation; }
	template<>           inline const float_t& x_comp(const vec3_t &t) { return t.x; }
	
	//! acess vector components by name
	template<typename T> inline       float_t& y_comp(T &t)            { invalid_instantiation; }
	template<>           inline       float_t& y_comp(vec3_t &t)       { return t.y; }
	template<typename T> inline const float_t& y_comp(const T &t)      { invalid_instantiation; }
	template<>           inline const float_t& y_comp(const vec3_t &t) { return t.y; }
	
	//! acess vector components by name
	template<typename T> inline       float_t& z_comp(T &t)            { invalid_instantiation; }
	template<>           inline       float_t& z_comp(vec3_t &t)       { return t.z; }
	template<typename T> inline const float_t& z_comp(const T &t)      { invalid_instantiation; }
	template<>           inline const float_t& z_comp(const vec3_t &t) { return t.z; }

	//! access a triangle's vertices by name - a
	template<typename T> inline       vec3_t& vertex_a(T &t)                     { invalid_instantiation; }
	template<>           inline       vec3_t& vertex_a(simple_triangle &t)       { return t.a; }
	template<typename T> inline const vec3_t& vertex_a(const T &t)               { invalid_instantiation; }
	template<>           inline const vec3_t& vertex_a(const simple_triangle &t) { return t.a; }

	//! access a triangle's vertices by name - b
	template<typename T> inline       vec3_t& vertex_b(T &t)                     { invalid_instantiation; }
	template<>           inline       vec3_t& vertex_b(simple_triangle &t)       { return t.b; }
	template<typename T> inline const vec3_t& vertex_b(const T &t)               { invalid_instantiation; }
	template<>           inline const vec3_t& vertex_b(const simple_triangle &t) { return t.b; }

	//! access a triangle's vertices by name - c
	template<typename T> inline       vec3_t& vertex_c(T &t)                     { invalid_instantiation; }
	template<>           inline       vec3_t& vertex_c(simple_triangle &t)       { return t.c; }
	template<typename T> inline const vec3_t& vertex_c(const T &t)               { invalid_instantiation; }
	template<>           inline const vec3_t& vertex_c(const simple_triangle &t) { return t.c; }

	//! access an aabb's vertices by name
	template<typename T> inline vec3_t& min(T &bb) { invalid_instantiation; }
	template<> inline vec3_t& min(simple_aabb &bb) { return bb.min; }
	
	//! access an aabb's vertices by name
	template<typename T> inline vec3_t& max(T &bb) { invalid_instantiation; }
	template<> inline vec3_t& max(simple_aabb &bb) { return bb.max; }



	// output


	std::ostream& operator<<(std::ostream &o, const vec3_t &v) { o << "(vec " << v.x << " " << v.y << " " << v.z << ")"; }
	std::ostream& operator<<(std::ostream &o, const simple_aabb &b) { o << "(aabb " << b.min << " " << b.max << ")"; }
	std::ostream& operator<<(std::ostream &o, const simple_triangle &t) { o << "(tri " << t.a << " " << t.b << " " << t.c << ")"; }


	#undef invalid_instantiation
	#undef invalid_instantiation_message
}

#include "tri.h"
#include "aabb.h"

#endif

