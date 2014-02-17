#ifndef __RTA_AABB_H__ 
#define __RTA_AABB_H__ 

#include "template-magic.h"
#include "basic_types.h"

namespace rta {

	template<typename aabb, typename tri> void merge(aabb &bb, const tri &t)
	{
		typedef vec3_t V;
		const V &a = vertex_a(t);
		if (x_comp(a) < x_comp(min(bb)))	x_comp(min(bb)) = x_comp(a);		if (x_comp(a) > x_comp(max(bb))) x_comp(max(bb)) = x_comp(a);
		if (y_comp(a) < y_comp(min(bb)))	y_comp(min(bb)) = y_comp(a);		if (y_comp(a) > y_comp(max(bb))) y_comp(max(bb)) = y_comp(a);
		if (z_comp(a) < z_comp(min(bb)))	z_comp(min(bb)) = z_comp(a);		if (z_comp(a) > z_comp(max(bb))) z_comp(max(bb)) = z_comp(a);
		
		const V &b = vertex_b(t);	
		if (x_comp(b) < x_comp(min(bb)))	x_comp(min(bb)) = x_comp(b);		if (x_comp(b) > x_comp(max(bb))) x_comp(max(bb)) = x_comp(b);
		if (y_comp(b) < y_comp(min(bb)))	y_comp(min(bb)) = y_comp(b);		if (y_comp(b) > y_comp(max(bb))) y_comp(max(bb)) = y_comp(b);
		if (z_comp(b) < z_comp(min(bb)))	z_comp(min(bb)) = z_comp(b);		if (z_comp(b) > z_comp(max(bb))) z_comp(max(bb)) = z_comp(b);
		
		const V &c = vertex_c(t);
		if (x_comp(c) < x_comp(min(bb)))	x_comp(min(bb)) = x_comp(c);		if (x_comp(c) > x_comp(max(bb))) x_comp(max(bb)) = x_comp(c);
		if (y_comp(c) < y_comp(min(bb)))	y_comp(min(bb)) = y_comp(c);		if (y_comp(c) > y_comp(max(bb))) y_comp(max(bb)) = y_comp(c);
		if (z_comp(c) < z_comp(min(bb)))	z_comp(min(bb)) = z_comp(c);		if (z_comp(c) > z_comp(max(bb))) z_comp(max(bb)) = z_comp(c);
	}

	template<typename aabb, typename CONTAINER> aabb compute_aabb(const CONTAINER &container, typename CONTAINER::const_iterator start, typename CONTAINER::const_iterator end)
	{
		aabb bb;
		min(bb) = start->vertex_a();
		max(bb) = start->vertex_a();
		for (typename CONTAINER::const_iterator it = start; it != end; ++it)
			merge(bb, *it);

		return bb;
	}

	template<typename aabb, typename TRI> aabb compute_aabb(const TRI *container, unsigned int start, unsigned int end)
	{
		aabb bb;
		min(bb) = vertex_a(container[start]);
		max(bb) = vertex_a(container[start]);
		for (unsigned int it = start; it != end; ++it)
			merge(bb, container[it]);

		return bb;
	}

	template<typename aabb, typename CONTAINER> typename enable_if<is_container<CONTAINER>::value, aabb>::Type compute_aabb(const CONTAINER &container)
	{
		return compute_aabb<aabb>(container, container.begin(), container.end());
	}

	template<typename aabb, typename TRI> aabb compute_aabb(TRI **triangle, unsigned int n)
	{
		aabb bb;
		min(bb) = vertex_a(*triangle[0]);
		max(bb) = vertex_a(*triangle[0]);
		for (int i = 0; i < n; ++i)
			merge(bb, *triangle[i]);

		return bb;
	}

	template<typename aabb, typename TRI> typename enable_if<!is_container<TRI>::value, aabb>::Type compute_aabb(const TRI &t)
	{	
		aabb bb;
		min(bb) = vertex_a(t);
		max(bb) = vertex_a(t);
		merge(bb, t);
		return bb;
	}

	template<typename aabb> inline void merge(aabb &a, const aabb &b)
	{
		x_comp(min(a)) = std::min(x_comp(min(a)), x_comp(min(b)));
		y_comp(min(a)) = std::min(y_comp(min(a)), y_comp(min(b)));
		z_comp(min(a)) = std::min(z_comp(min(a)), z_comp(min(b)));
		x_comp(max(a)) = std::max(x_comp(max(a)), x_comp(max(b)));
		y_comp(max(a)) = std::max(y_comp(max(a)), y_comp(max(b)));
		z_comp(max(a)) = std::max(z_comp(max(a)), z_comp(max(b)));
	}

	template<typename aabb> inline aabb union_of(const aabb &a, const aabb &b)
	{
		aabb res;
		x_comp(min(res)) = std::min(x_comp(min(a)), x_comp(min(b)));
		y_comp(min(res)) = std::min(y_comp(min(a)), y_comp(min(b)));
		z_comp(min(res)) = std::min(z_comp(min(a)), z_comp(min(b)));
		x_comp(max(res)) = std::max(x_comp(max(a)), x_comp(max(b)));
		y_comp(max(res)) = std::max(y_comp(max(a)), y_comp(max(b)));
		z_comp(max(res)) = std::max(z_comp(max(a)), z_comp(max(b)));
		return res;
	}

	template<typename aabb> inline vec3_t center(const aabb &b)
	{
		return typename aabb::vec3_t((b.max - b.min) * 0.5 + b.min);
	}

}

#endif

