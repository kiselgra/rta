#ifndef __CUDA_VEC_H__ 
#define __CUDA_VEC_H__ 

#include <librta/rta-config.h>

#if RTA_HAVE_LIBCUDART == 1

#include <vector_types.h>

namespace rta {
	heterogenous inline float& x_comp(float3 &f) { return f.x; }
	heterogenous inline float& y_comp(float3 &f) { return f.y; }
	heterogenous inline float& z_comp(float3 &f) { return f.z; }
	
	heterogenous inline const float& x_comp(const float3 &f) { return f.x; }
	heterogenous inline const float& y_comp(const float3 &f) { return f.y; }
	heterogenous inline const float& z_comp(const float3 &f) { return f.z; }
	
	heterogenous inline float& x_comp(float4 &f) { return f.x; }
	heterogenous inline float& y_comp(float4 &f) { return f.y; }
	heterogenous inline float& z_comp(float4 &f) { return f.z; }
	heterogenous inline float& w_comp(float4 &f) { return f.w; }
	
	heterogenous inline const float& x_comp(const float4 &f) { return f.x; }
	heterogenous inline const float& y_comp(const float4 &f) { return f.y; }
	heterogenous inline const float& z_comp(const float4 &f) { return f.z; }
	heterogenous inline const float& w_comp(const float4 &f) { return f.w; }

	#ifdef __CUDACC__
	// actually from basic_types.... :/
	heterogenous inline       float_t& x_comp(vec2_t &t)       { return t.x; }
	heterogenous inline       float_t& x_comp(vec3_t &t)       { return t.x; }
	heterogenous inline       float_t& x_comp(vec4_t &t)       { return t.x; }
	heterogenous inline const float_t& x_comp(const vec3_t &t) { return t.x; }
	heterogenous inline const float_t& x_comp(const vec4_t &t) { return t.x; }
	
	//! acess vector components by name
	heterogenous inline       float_t& y_comp(vec2_t &t)       { return t.y; }
	heterogenous inline       float_t& y_comp(vec3_t &t)       { return t.y; }
	heterogenous inline       float_t& y_comp(vec4_t &t)       { return t.y; }
	heterogenous inline const float_t& y_comp(const vec3_t &t) { return t.y; }
	heterogenous inline const float_t& y_comp(const vec4_t &t) { return t.y; }
	
	//! acess vector components by name
	heterogenous inline       float_t& z_comp(vec3_t &t)       { return t.z; }
	heterogenous inline       float_t& z_comp(vec4_t &t)       { return t.z; }
	heterogenous inline const float_t& z_comp(const vec3_t &t) { return t.z; }
	heterogenous inline const float_t& z_comp(const vec4_t &t) { return t.z; }
	
	//! acess vector w components by name
	heterogenous inline       float_t& w_comp(vec4_t &t)       { return t.w; }
	heterogenous inline const float_t& w_comp(const vec4_t &t) { return t.w; }
	
	//! @}
	
	/*! \defgroup triangle_accessors Triangle Accessors
	 *  \ingroup basic_types
	 *  \addtogroup triangle_accessors
	 *  @{
	 */
	//! access a triangle's vertices by name - a
	heterogenous inline       vec3_t& vertex_a(simple_triangle &t)       { return t.a; }
	heterogenous inline const vec3_t& vertex_a(const simple_triangle &t) { return t.a; }

	//! access a triangle's vertices by name - b
	heterogenous inline       vec3_t& vertex_b(simple_triangle &t)       { return t.b; }
	heterogenous inline const vec3_t& vertex_b(const simple_triangle &t) { return t.b; }

	//! access a triangle's vertices by name - c
	heterogenous inline       vec3_t& vertex_c(simple_triangle &t)       { return t.c; }
	heterogenous inline const vec3_t& vertex_c(const simple_triangle &t) { return t.c; }
	#endif	

}

#endif

#endif

