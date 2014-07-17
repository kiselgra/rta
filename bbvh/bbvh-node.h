#ifndef __BBVH_NODE_H__ 
#define __BBVH_NODE_H__ 

namespace rta {
	template<typename box_t_> struct bbvh_node {
		typedef uint32_t link_t;
		typedef box_t_ box_t;
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
		uint split_axis() { return 0; }
	};
}

#endif

