#ifndef __CUDA_VEC_H__ 
#define __CUDA_VEC_H__ 

#include <librta/rta-config.h>

#if RTA_HAVE_LIBCUDART == 1

#include <vector_types.h>
#include <math.h>

// __host__ inline int __float_as_int(float f) { return *reinterpret_cast<int*>(&f); }
// __host__ inline float __int_as_float(int i) { return *reinterpret_cast<float*>(&i); }

namespace rta {
	struct cuda_ftl;

	template<> struct vector_traits<float3> {
		typedef float2 vec2_t;
		typedef float3 vec3_t;
		typedef float4 vec4_t;
	};
		
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
			typedef cuda_ftl input_flat_triangle_list_t;
			typedef float3 vec3_t;
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

	// where is cgen based code when you need it....
	//#ifndef __CUDACC__	// the indirection might kill us on cuda.
	heterogenous inline void sub_components_vec2f(float2 *out, float2 *lhs, float2 *rhs) {
		out->x = lhs->x - rhs->x;
		out->y = lhs->y - rhs->y;
	}
	heterogenous inline void add_components_vec2f(float2 *out, float2 *lhs, float2 *rhs) {
		out->x = lhs->x + rhs->x;
		out->y = lhs->y + rhs->y;
	}
	heterogenous inline void mul_vec2f_by_scalar(float2 *out, const float2 *lhs, float rhs) {
		out->x = (lhs->x * rhs);
		out->y = (lhs->y * rhs);
	}
	heterogenous inline void div_vec2f_by_scalar(float2 *out, const float2 *lhs, float rhs) {
		out->x = (lhs->x / rhs);
		out->y = (lhs->y / rhs);
	}
	heterogenous inline float dot_vec2f(const float2 *lhs, const float2 *rhs) {
		float sum = (lhs->x * rhs->x);
		sum = (sum + (lhs->y * rhs->y));
		return sum;
	}
	heterogenous inline float length_of_vec2f(const float2 *v) {
		return sqrtf(dot_vec2f(v, v));
	}
	heterogenous inline void normalize_vec2f(float2 *v) {
		div_vec2f_by_scalar(v, v, length_of_vec2f(v));
	}
	heterogenous inline void sub_components_vec3f(float3 *out, float3 *lhs, float3 *rhs) {
		out->x = lhs->x - rhs->x;
		out->y = lhs->y - rhs->y;
		out->z = lhs->z - rhs->z;
	}
	heterogenous inline void add_components_vec3f(float3 *out, float3 *lhs, float3 *rhs) {
		out->x = lhs->x + rhs->x;
		out->y = lhs->y + rhs->y;
		out->z = lhs->z + rhs->z;
	}
	heterogenous inline void mul_vec3f_by_scalar(float3 *out, const float3 *lhs, float rhs) {
		out->x = (lhs->x * rhs);
		out->y = (lhs->y * rhs);
		out->z = (lhs->z * rhs);
	}
	heterogenous inline void div_vec3f_by_scalar(float3 *out, const float3 *lhs, float rhs) {
		out->x = (lhs->x / rhs);
		out->y = (lhs->y / rhs);
		out->z = (lhs->z / rhs);
	}
	heterogenous inline float dot_vec3f(const float3 *lhs, const float3 *rhs) {
		float sum = (lhs->x * rhs->x);
		sum = (sum + (lhs->y * rhs->y));
		sum = (sum + (lhs->z * rhs->z));
		return sum;
	}
	heterogenous inline float length_of_vec3f(const float3 *v) {
		return sqrtf(dot_vec3f(v, v));
	}
	heterogenous inline void normalize_vec3f(float3 *v) {
		div_vec3f_by_scalar(v, v, length_of_vec3f(v));
	}
	heterogenous inline void cross_vec3f(float3 *out, const float3 *lhs, const float3 *rhs) {
		out->x = ((lhs->y * rhs->z) - (lhs->z * rhs->y));
		out->y = ((lhs->z * rhs->x) - (lhs->x * rhs->z));
		out->z = ((lhs->x * rhs->y) - (lhs->y * rhs->x));
	}

	//#endif

	#ifndef __CUDACC__
	inline       float3& vertex_a(cuda::simple_triangle &t)       { return t.a; }
	inline const float3& vertex_a(const cuda::simple_triangle &t) { return t.a; }
	inline       float3& vertex_b(cuda::simple_triangle &t)       { return t.b; }
	inline const float3& vertex_b(const cuda::simple_triangle &t) { return t.b; }
	inline       float3& vertex_c(cuda::simple_triangle &t)       { return t.c; }
	inline const float3& vertex_c(const cuda::simple_triangle &t) { return t.c; }
	inline       float3& normal_a(cuda::simple_triangle &t)       { return t.na; }
	inline const float3& normal_a(const cuda::simple_triangle &t) { return t.na; }
	inline       float3& normal_b(cuda::simple_triangle &t)       { return t.nb; }
	inline const float3& normal_b(const cuda::simple_triangle &t) { return t.nb; }
	inline       float3& normal_c(cuda::simple_triangle &t)       { return t.nc; }
	inline const float3& normal_c(const cuda::simple_triangle &t) { return t.nc; }
	inline       float2& texcoord_a(cuda::simple_triangle &t)       { return t.ta; }
	inline const float2& texcoord_a(const cuda::simple_triangle &t) { return t.ta; }
	inline       float2& texcoord_b(cuda::simple_triangle &t)       { return t.tb; }
	inline const float2& texcoord_b(const cuda::simple_triangle &t) { return t.tb; }
	inline       float2& texcoord_c(cuda::simple_triangle &t)       { return t.tc; }
	inline const float2& texcoord_c(const cuda::simple_triangle &t) { return t.tc; }
	inline       float3& min(cuda::simple_aabb &bb)       { return bb.min; }
	inline const float3& min(const cuda::simple_aabb &bb) { return bb.min; }
	inline       float3& max(cuda::simple_aabb &bb)       { return bb.max; }
	inline const float3& max(const cuda::simple_aabb &bb) { return bb.max; }

	template<> inline       float_t& x_comp(float3 &t)       { return t.x; }
	template<> inline const float_t& x_comp(const float3 &t) { return t.x; }
	template<> inline       float_t& y_comp(float3 &t)       { return t.y; }
	template<> inline const float_t& y_comp(const float3 &t) { return t.y; }
	template<> inline       float_t& z_comp(float3 &t)       { return t.z; }
	template<> inline const float_t& z_comp(const float3 &t) { return t.z; }

	//! a little helper http://stackoverflow.com/questions/7931358/printing-sizeoft-at-compile-time
	template<int N> 
	struct print_number_as_warning { 
		char operator()() { return N + 256; } //deliberately causing overflow
	};

	struct cuda_ftl : public basic_flat_triangle_list<cuda::simple_triangle> {
		cuda_ftl() : basic_flat_triangle_list<cuda::simple_triangle>() {}
		cuda_ftl(int size) : basic_flat_triangle_list<cuda::simple_triangle>(size) {}
		cuda_ftl(basic_flat_triangle_list<rta::simple_triangle> &base) {
			triangles = base.triangles;
			static_assert(sizeof(simple_triangle) == sizeof(cuda::simple_triangle),
			              "Triangle sizes of host vs cuda do not match. Code will break.");
			triangle = reinterpret_cast<cuda::simple_triangle*>(base.triangle);
		}
	};
	#else
	heterogenous inline float3 operator*(const float3 &a, const float3 &b) {
		make_float4(1,2,3,4);
		return make_float3(a.x*b.x, a.y*b.y, a.z*b.z);
	}
	heterogenous inline float3 operator/(const float3 &a, const float3 &b) {
		return make_float3(a.x/b.x, a.y/b.y, a.z/b.z);
	}
	heterogenous inline float3 operator-(const float3 &a, const float3 &b) {
		return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
	}
	#endif
	
	
}

#endif

#endif

