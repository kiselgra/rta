/* $Id$ */
#ifndef __RTA_TEMPLATE_MAGIC_H__ 
#define __RTA_TEMPLATE_MAGIC_H__ 

// #include "librc-test/vecs.h"

#include <vector>

namespace rta
{
	template<bool enable, typename T> struct enable_if { };
	template<bool enable, typename T> struct disable_if { };

	template<typename T> struct enable_if<true, T> { typedef T type; };
	template<typename T> struct disable_if<false, T> { typedef T type; };

	template<typename T> struct iscontainer
	{
		enum { result = 0 };
	};

	template<typename T> struct iscontainer<std::vector<T> >
	{
		enum { result = 1 };
	};

	
// 	template<typename T> struct isscalar	{ enum { Result = true }; 	};
// 	template<> struct isscalar<vec4fsse> 	{ enum { Result = false}; 	};
// 	template<> struct isscalar<vec3f> 		{ enum { Result = false}; 	};


	template<bool first, typename T1, typename T2> struct if_then_else 		{ typedef T1 result; };
	template<typename T1, typename T2> struct if_then_else<false, T1, T2> 	{ typedef T2 result; };

	template<typename T> struct is_void        { enum { result = false }; };
	template<>           struct is_void<void>  { enum { result = true }; };
	template<typename T> struct not_void       { enum { result = true }; };
	template<>           struct not_void<void> { enum { result = false }; };

	template<typename T> struct fail { enum { result = false }; };
// 	//! according to andrej a.
// 	template <bool> struct static_assertion;
// 	template<> struct static_assertion<true> {};

}

#endif

