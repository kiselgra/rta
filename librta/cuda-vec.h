#ifndef __CUDA_VEC_H__ 
#define __CUDA_VEC_H__ 

#include <librta/rta-config.h>

#if RTA_HAVE_LIBCUDART == 1

#include <vector_types.h>

namespace rta {
	namespace cuda {
	
		/*! cuda version of our most primitive triangle type.
		 * 	\note we need this as cuda requries __device__ functions
		 * 	throughout, i.e. we can't use our own math lib that is not written
		 * 	for cuda.
		 */
		struct simple_triangle {
			float3 a, b, c;
			float3 na, nb, nc;
			float2 ta, tb, tc;
			int material_index;
		};

		//! simple aabb, see cuda::simple_triangle
		struct simple_aabb {
			float3 min, max;
			heterogenous simple_aabb() {}
		};

	}

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
	heterogenous inline       float3& vertex_a(cuda::simple_triangle &t)       { return t.a; }
	heterogenous inline const float3& vertex_a(const cuda::simple_triangle &t) { return t.a; }

	//! access a triangle's vertices by name - b
	heterogenous inline       vec3_t& vertex_b(simple_triangle &t)       { return t.b; }
	heterogenous inline const vec3_t& vertex_b(const simple_triangle &t) { return t.b; }
	heterogenous inline       float3& vertex_b(cuda::simple_triangle &t)       { return t.b; }
	heterogenous inline const float3& vertex_b(const cuda::simple_triangle &t) { return t.b; }

	//! access a triangle's vertices by name - c
	heterogenous inline       vec3_t& vertex_c(simple_triangle &t)       { return t.c; }
	heterogenous inline const vec3_t& vertex_c(const simple_triangle &t) { return t.c; }
	heterogenous inline       float3& vertex_c(cuda::simple_triangle &t)       { return t.c; }
	heterogenous inline const float3& vertex_c(const cuda::simple_triangle &t) { return t.c; }

	//! access an aabb's vertices by name - min
	heterogenous inline       vec3_t& min(simple_aabb &bb)       { return bb.min; }
	heterogenous inline const vec3_t& min(const simple_aabb &bb) { return bb.min; }
	heterogenous inline       float3& min(cuda::simple_aabb &bb)       { return bb.min; }
	heterogenous inline const float3& min(const cuda::simple_aabb &bb) { return bb.min; }
	
	//! access an aabb's vertices by name - max
	heterogenous inline       vec3_t& max(simple_aabb &bb)       { return bb.max; }
	heterogenous inline const vec3_t& max(const simple_aabb &bb) { return bb.max; }
	heterogenous inline       float3& max(cuda::simple_aabb &bb)       { return bb.max; }
	heterogenous inline const float3& max(const cuda::simple_aabb &bb) { return bb.max; }
	#endif	

	#ifndef __CUDACC__
	inline       float3& vertex_a(cuda::simple_triangle &t)       { return t.a; }
	inline const float3& vertex_a(const cuda::simple_triangle &t) { return t.a; }
	inline       float3& vertex_b(cuda::simple_triangle &t)       { return t.b; }
	inline const float3& vertex_b(const cuda::simple_triangle &t) { return t.b; }
	inline       float3& vertex_c(cuda::simple_triangle &t)       { return t.c; }
	inline const float3& vertex_c(const cuda::simple_triangle &t) { return t.c; }
	inline       float3& min(cuda::simple_aabb &bb)       { return bb.min; }
	inline const float3& min(const cuda::simple_aabb &bb) { return bb.min; }
	inline       float3& max(cuda::simple_aabb &bb)       { return bb.max; }
	inline const float3& max(const cuda::simple_aabb &bb) { return bb.max; }

	//! a little helper http://stackoverflow.com/questions/7931358/printing-sizeoft-at-compile-time
	template<int N> 
	struct print_number_as_warning { 
		char operator()() { return N + 256; } //deliberately causing overflow
	};

	//! This is rather hacky!
	struct cuda_ftl {
		uint triangles;
		cuda::simple_triangle *triangle;
		cuda_ftl() : triangles(0), triangle(0) {}
		cuda_ftl(int size) : triangles(size), triangle(0) { triangle = new cuda::simple_triangle[size]; }
		cuda_ftl(flat_triangle_list &base) {
			triangles = base.triangles;
			static_assert(sizeof(simple_triangle) == sizeof(cuda::simple_triangle),
			              "Triangle sizes of host vs cuda do not match. Code will break.");
			triangle = reinterpret_cast<cuda::simple_triangle*>(base.triangle);
		}
	};
	#endif
	
}

#endif

#endif

