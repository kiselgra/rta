#ifndef __RTA_BBVH_CUDA_NODE_H__ 
#define __RTA_BBVH_CUDA_NODE_H__ 

#ifdef __CUDACC__
#include <stdint.h>
#endif

namespace rta {
	namespace cuda {
		namespace bbvh {

			/*! \attention This is a plain copy of rta::binary_bvh::node.
			 *  The only reason for this hackery is that the cuda compiler does
			 *  not support C++11, preventing us from including the appropriate
			 *  files in the .cu file. Grr.
			 */
			template<typename box_t> struct node {
				typedef uint32_t link_t;
				link_t type_left_elems; // contains: bit0=inner/leaf >>1=(left-child/tris-in-leaf)
				link_t right_tris;      // contains: right-child/link-to-tris
				box_t box;
				bool inner()         { return (type_left_elems&0x01)==1; }
				void make_inner()    { type_left_elems |= 0x01; }
				void make_leaf()     { type_left_elems &= (~1); }
				void left(link_t n)  { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }  //!< erases leaf/inner bit.
				link_t left()        { return type_left_elems>>1; }
				void elems(uint n)   { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }
				uint elems()         { return type_left_elems>>1; }
				//! link to the right.
				void right(link_t n) { right_tris = n; }
				link_t right()       { return right_tris; }
				//! an index into an array of triangles holding elems() successive triangles enclosed by this node.
				void tris(link_t n)  { right_tris = n; }
				uint tris()          { return right_tris; }
				void split_axis(uint a) {} //!< not implemented for default bbvh nodes.
			};

		}
	}
}


#endif

