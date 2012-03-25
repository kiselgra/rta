#ifndef __RTA_AABB_H__ 
#define __RTA_AABB_H__ 

#include "template-magick.h"

#include <ostream>

	//! Axis aligned bounding box. 
	template<typename V> class aabb
	{
	public:
		V min, 	//!< Minimale xyz Komponenten
		  max;	//!< Maximale xyz Komponenten
	};

	template<typename V> inline std::ostream& operator<<(std::ostream &o, const aabb<V> &bb)
	{
		o << "(aabb " << bb.min << " " << bb.max << ")";
		return o;
	}

	template<typename V, typename TRI> void unionize(aabb<V> &bb, const TRI &t)
	{
		const V &a = t.vertex_a();	
		if (x_comp(a) < x_comp(bb.min))		x_comp(bb.min) = x_comp(a);		if (x_comp(a) > x_comp(bb.max)) x_comp(bb.max) = x_comp(a);
		if (y_comp(a) < y_comp(bb.min))		y_comp(bb.min) = y_comp(a);		if (y_comp(a) > y_comp(bb.max)) y_comp(bb.max) = y_comp(a);
		if (z_comp(a) < z_comp(bb.min))		z_comp(bb.min) = z_comp(a);		if (z_comp(a) > z_comp(bb.max)) z_comp(bb.max) = z_comp(a);
		
		const V &b = t.vertex_b();	
		if (x_comp(b) < x_comp(bb.min))	x_comp(bb.min) = x_comp(b);		if (x_comp(b) > x_comp(bb.max)) x_comp(bb.max) = x_comp(b);
		if (y_comp(b) < y_comp(bb.min))	y_comp(bb.min) = y_comp(b);		if (y_comp(b) > y_comp(bb.max)) y_comp(bb.max) = y_comp(b);
		if (z_comp(b) < z_comp(bb.min))	z_comp(bb.min) = z_comp(b);		if (z_comp(b) > z_comp(bb.max)) z_comp(bb.max) = z_comp(b);
		
		const V &c = t.vertex_c();
		if (x_comp(c) < x_comp(bb.min))	x_comp(bb.min) = x_comp(c);		if (x_comp(c) > x_comp(bb.max)) x_comp(bb.max) = x_comp(c);
		if (y_comp(c) < y_comp(bb.min))	y_comp(bb.min) = y_comp(c);		if (y_comp(c) > y_comp(bb.max)) y_comp(bb.max) = y_comp(c);
		if (z_comp(c) < z_comp(bb.min))	z_comp(bb.min) = z_comp(c);		if (z_comp(c) > z_comp(bb.max)) z_comp(bb.max) = z_comp(c);
	}

	template<typename V, typename CONTAINER> aabb<V> compute_aabb(const CONTAINER &container, typename CONTAINER::const_iterator start, typename CONTAINER::const_iterator end)
	{
		aabb<V> bb;
		bb.min = start->vertex_a();
		bb.max = start->vertex_a();
		for (typename CONTAINER::const_iterator it = start; it != end; ++it)
			unionize(bb, *it);

		return bb;
	}

	template<typename V, typename TRI> aabb<V> compute_aabb(const TRI *container, unsigned int start, unsigned int end)
	{
		aabb<V> bb;
		bb.min = container[start].vertex_a();
		bb.max = container[start].vertex_a();
		for (unsigned int it = start; it != end; ++it)
			unionize(bb, container[it]);

		return bb;
	}

	template<typename V, typename CONTAINER> typename enable_if<iscontainer<CONTAINER>::Result, aabb<V> >::Type compute_aabb(const CONTAINER &container)
	{
		return compute_aabb<V>(container, container.begin(), container.end());
	}

	template<typename V, typename TRI> aabb<V> compute_aabb(TRI **triangle, unsigned int n)
	{
		aabb<V> bb;
		bb.min = triangle[0]->vertex_a();
		bb.max = triangle[0]->vertex_a();
		for (int i = 0; i < n; ++i)
			unionize(bb, *triangle[i]);

		return bb;
	}

	template<typename V, typename TRI> typename enable_if<!iscontainer<TRI>::Result, aabb<V> >::Type compute_aabb(const TRI &t)
	{	
		aabb<V> bb;
		bb.min = t.vertex_a();
		bb.max = t.vertex_a();
		unionize(bb, t);
		return bb;
	}

	template<typename V> inline void unionize(aabb<V> &a, const aabb<V> &b)
	{
		x_comp(a.min) = std::min(x_comp(a.min), x_comp(b.min));
		y_comp(a.min) = std::min(y_comp(a.min), y_comp(b.min));
		z_comp(a.min) = std::min(z_comp(a.min), z_comp(b.min));
		x_comp(a.max) = std::max(x_comp(a.max), x_comp(b.max));
		y_comp(a.max) = std::max(y_comp(a.max), y_comp(b.max));
		z_comp(a.max) = std::max(z_comp(a.max), z_comp(b.max));
	}

	template<typename V> inline aabb<V> union_of(const aabb<V> &a, const aabb<V> &b)
	{
		aabb<V> res;
		x_comp(res.min) = std::min(x_comp(a.min), x_comp(b.min));
		y_comp(res.min) = std::min(y_comp(a.min), y_comp(b.min));
		z_comp(res.min) = std::min(z_comp(a.min), z_comp(b.min));
		x_comp(res.max) = std::max(x_comp(a.max), x_comp(b.max));
		y_comp(res.max) = std::max(y_comp(a.max), y_comp(b.max));
		z_comp(res.max) = std::max(z_comp(a.max), z_comp(b.max));
		return res;
	}

	template<typename V> inline V center(const aabb<V> &b)
	{
		return V((b.max - b.min) * 0.5 + b.min);
	}


#endif

