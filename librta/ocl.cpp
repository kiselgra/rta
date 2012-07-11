#include "librta/ocl.h"

using namespace std;

namespace rta {
	namespace ocl {

		bool using_ocl() {
			return context != 0;
		}
		cl::context *context = 0;

	}
}

/* vim: set foldmethod=marker: */

