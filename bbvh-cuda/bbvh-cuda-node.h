#ifndef __RTA_BBVH_CUDA_NODE_H__ 
#define __RTA_BBVH_CUDA_NODE_H__ 

#ifdef __CUDACC__
#include <stdint.h>
#endif

#include "bbvh/bbvh-node.h"
#include "librta/cuda-vec.h"

#ifndef __CUDACC__
namespace rta {
	namespace cuda {
		namespace internalstuff {
			union reinterpret_fi {
				float f;
				int i;
			};
		}
	}
}
inline int __float_as_int(float f) {
	rta::cuda::internalstuff::reinterpret_fi uni;
	uni.f = f;
	return uni.i;
}
inline float __int_as_float(int i) {
	rta::cuda::internalstuff::reinterpret_fi uni;
	uni.i = i;
	return uni.f;
}
#endif


namespace rta {
	namespace cuda {
		template<typename _box_t> struct bbvh_node_float4 {
			float4 a, b;
			typedef uint32_t link_t;
			typedef _box_t box_t;
			#define type_left_elems (a.x)
			#define right_tris (a.y)
			heterogenous bool inner()         { return (__float_as_int(a.x)&0x01)==1; }
			heterogenous void make_inner()    { type_left_elems = __int_as_float((uint)__float_as_int(type_left_elems)|0x01); }
			heterogenous void make_leaf()     { type_left_elems = __int_as_float((uint)__float_as_int(type_left_elems)&(~1)); }
			heterogenous void left(link_t n)  { type_left_elems = __int_as_float((__float_as_int(type_left_elems)&(~1)) | (n<<1)); } //!< erases leaf/inner bit.
			heterogenous link_t left()        { return __float_as_int(type_left_elems)>>1; }
			heterogenous void elems(uint n)   { type_left_elems = __int_as_float((__float_as_int(type_left_elems)&(~1)) | (n<<1)); }
			heterogenous uint elems()         { return __float_as_int(type_left_elems)>>1; }
			//! link to the right.
			heterogenous void right(link_t n) { right_tris = n; }
			heterogenous link_t right()       { return right_tris; }
			//! an index into an array of triangles holding elems() successive triangles enclosed by this node.
			heterogenous void tris(link_t n)  { right_tris = n; }
			heterogenous uint tris()          { return right_tris; }
			heterogenous void split_axis(uint a) {} //!< not implemented for default bbvh nodes.
			heterogenous uint split_axis() { return 0; }
			#undef type_left_elems
			#undef right_tris
			heterogenous void volume(box_t &box) {
				a.z = box.min.x;
				a.w = box.min.y;
				b.x = box.min.z;
				b.y = box.max.x;
				b.z = box.max.y;
				b.w = box.max.z;
			}
			//! \attention memory access for those will be bad.
			heterogenous float box_min_x() { return a.z; }
			heterogenous float box_min_y() { return a.w; }
			heterogenous float box_min_z() { return b.x; }
			heterogenous float box_max_x() { return b.y; }
			heterogenous float box_max_y() { return b.z; }
			heterogenous float box_max_z() { return b.w; }
			//! \attention this is expensive!
			heterogenous const box_t& gen_box() {
				box_t box;
				box.min.x = a.z;
				box.min.y = a.w;
				box.min.z = b.x;
				box.max.x = b.y;
				box.max.y = b.z;
				box.max.z = b.w;
				return box;
			}
		};
	}
}

#endif

