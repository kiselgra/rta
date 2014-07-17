#ifndef __RTA_BBVH_CUDA_NODE_H__ 
#define __RTA_BBVH_CUDA_NODE_H__ 

#ifdef __CUDACC__
#include <stdint.h>
#endif

#include "bbvh/bbvh-node.h"
#include "librta/cuda-vec.h"

namespace rta {
	namespace cuda {
		/*
		struct bbvh_node_float4 {
			float4 a, b;
			typedef uint32_t link_t;
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
		};
		*/
	}
}

#endif

