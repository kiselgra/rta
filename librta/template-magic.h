/* $Id$ */
#ifndef __RTA_TEMPLATE_MAGIC_H__ 
#define __RTA_TEMPLATE_MAGIC_H__ 

// #include "librc-test/vecs.h"

#include <vector>
#include <type_traits>

namespace rta
{
	template<bool enable, typename T> struct enable_if { };
	template<bool enable, typename T> struct disable_if { };

	template<typename T> struct enable_if<true, T> { typedef T type; };
	template<typename T> struct disable_if<false, T> { typedef T type; };

 	// taken from http://stackoverflow.com/questions/9242209/is-container-trait-fails-on-stdset-sfinae-issue
	// comment of http://stackoverflow.com/users/34509/johannes-schaub-litb
    template<typename Container, typename = void>
    struct is_container : std::false_type { };
     
    template<typename A, A, A>
    struct is_of_type { typedef void type; };
     
    template<typename C>
    struct is_container<C,
                        typename is_of_type<typename C::iterator(C::*)(),
                                            &C::begin,
                                            &C::end>::type> 
	       : std::is_class<C> { };
	// ---
     
	
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

