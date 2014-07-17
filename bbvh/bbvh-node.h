#ifndef __BBVH_NODE_H__ 
#define __BBVH_NODE_H__ 

namespace rta {
	template<typename box_t_> struct bbvh_node {
		typedef uint32_t link_t;
		typedef box_t_ box_t;
		link_t type_left_elems; //!< contains: bit0=inner/leaf >>1=(left-child/tris-in-leaf)
		link_t right_tris;      //!< contains: right-child/link-to-tris
		/*! the handling of bounding boxes in gpu code is a little troublesome.
		 *  tracers (cpu and gpu) should access the box data directly, as any accessors
		 *  might involve conversions from more compact representations (ie nodes as 2 float4).
		 *  this is fine, as tracers will usually know details of the bvh.
		 */
		box_t box;
		heterogenous bool inner()         { return (type_left_elems&0x01)==1; }
		heterogenous void make_inner()    { type_left_elems |= 0x01; }
		heterogenous void make_leaf()     { type_left_elems &= (~1); }
		heterogenous void left(link_t n)  { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }  //!< erases leaf/inner bit.
		heterogenous link_t left()        { return type_left_elems>>1; }
		heterogenous void elems(uint n)   { type_left_elems = ((type_left_elems&(~1)) | (n<<1)); }
		heterogenous uint elems()         { return type_left_elems>>1; }
		//! link to the right.
		heterogenous void right(link_t n) { right_tris = n; }
		heterogenous link_t right()       { return right_tris; }
		//! an index into an array of triangles holding elems() successive triangles enclosed by this node.
		heterogenous void tris(link_t n)  { right_tris = n; }
		heterogenous uint tris()          { return right_tris; }
		heterogenous void split_axis(uint a) {} //!< not implemented for default bbvh nodes.
		heterogenous uint split_axis() { return 0; }
		heterogenous void volume(box_t &b) { box = b; }
		//! those should not be used in device code! just load the box. this is just to be able to output a variety of bounding boxes for mamo data.
		heterogenous float box_max_x() { return box.max.x; }
		heterogenous float box_max_y() { return box.max.y; }
		heterogenous float box_max_z() { return box.max.z; }
		heterogenous float box_min_x() { return box.min.x; }
		heterogenous float box_min_y() { return box.min.y; }
		heterogenous float box_min_z() { return box.min.z; }
		//! this might be expensive for other node types.
		heterogenous const box_t& gen_box() { return box; }
	};
}

#endif

