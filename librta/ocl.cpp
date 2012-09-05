#include "librta/ocl.h"

#if RTA_HAVE_LIBLUMOCL == 1

using namespace std;

namespace rta {
	namespace ocl {

		bool using_ocl() {
			return context != 0;
		}
		cl::context *context = 0;

	}
}

#else

namespace rta {
	namespace ocl {
		bool using_ocl() {
			return false;
		}
	}
}

#endif

/* vim: set foldmethod=marker: */

