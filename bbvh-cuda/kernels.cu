#include "librta/basic_types.h"
#include "bbvh-cuda-node.h"

#include <iostream>
using namespace std;

namespace rta {
	namespace cuda {

		namespace k {
		}

		void trace_dis(simple_triangle *triangles, int n, bbvh::node<simple_aabb> *nodes, vec3f *ray_orig, vec3f *ray_dir, float *max_t, 
		               int w, int h, triangle_intersection<simple_triangle> *is) {
			cout << "kernel!" << endl;
		}

	}
}
