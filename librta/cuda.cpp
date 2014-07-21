#include "librta/cuda.h"

#ifdef WITH_CUDA

using namespace std;

namespace rta {
	namespace cuda {

		bool using_cuda() {
			return plugin_with_cuda_loaded;
		}
		bool plugin_with_cuda_loaded = false;

	}
}

#else

namespace rta {
	namespace cuda {
		bool using_cuda() {
			return false;
		}
	}
}

#endif

/* vim: set foldmethod=marker: */
