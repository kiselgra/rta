// cuda can't handle gcc 4.7 includes...
// https://bugs.archlinux.org/task/29359
#undef _GLIBCXX_ATOMIC_BUILTINS
#undef _GLIBCXX_USE_INT128

#include "librta/basic_types.h"
#include "librta/cuda-kernels.h"
#include "librta/cuda-vec.h"
#include "librta/intersect.h"

#include "bbvh-cuda-node.h"

#include <vector_types.h>

#include <iostream>
#include <stdio.h>

#include <thrust/sequence.h>
//#include <thrust/binary_search.h>
//#include <thrust/functional.h>
#include <thrust/device_ptr.h>
#include <thrust/reduce.h>
#include <thrust/sort.h>

#define START_CUDA_TIMER cudaEvent_t start, stop;\
	cudaEventCreate(&start);cudaEventCreate(&stop);\
	cudaEventRecord(start);
#define STOP_CUDA_TIMER(TIME)  cudaEventRecord(stop);cudaEventSynchronize(stop);\
	cudaEventElapsedTime(&TIME, start, stop);\
	cudaEventDestroy(start); cudaEventDestroy(stop);

#define GRIDSIZE 1024.0f //2^K
#define GRIDSIZEV 128.0f //No Overlap voxel resolution
#define NOLAYER 21 //log(GRIDSIZEV)*3


using namespace std;

namespace rta {
	namespace cuda {

		typedef bbvh_node<cuda::simple_aabb> node_t;

		namespace k {

			__host__ __device__ float3 operator+(const float3 & a, const float3 & b) {
				return make_float3(a.x+b.x, a.y+b.y, a.z+b.z);
			}

			__host__ __device__ float3 operator-(const float3 & a, const float3 & b) {
				return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
			}

			__host__ __device__ float3 operator/(const float3 & a, const float & b) {
				return make_float3(a.x/b, a.y/b, a.z/b);
			}

			__host__ __device__ float3 center_of_gravity(const cuda::simple_triangle &tri) {
				return (tri.a+tri.b+tri.c)/3.0f;
				//return make_float3( (tri.a.x+tri.b.x+tri.c.x)/3, (tri.a.y+tri.b.y+tri.c.y)/3, (tri.a.z+tri.b.z+tri.c.z)/3);
			}

			__device__ int divideBy2Ceiling(int n){
				return n/2 + n%2;
			}

			__device__ int min(int n1, int n2){
				return (n1<n2)? n1: n2;
			}
			__device__ int max(int n1, int n2){
				return (n1>n2)? n1: n2;
			}

			__device__ float minf(float n1, float n2){
				return (n1<n2)? n1: n2;
			}
			__device__ float maxf(float n1, float n2){
				return (n1>n2)? n1: n2;
			}

			//common prefix length
			__device__ int __cpl(int m1, int m2,const uint* codes, const int &n){
				if(m2 < 0 || m2>=n)
					return -1;

				if(codes[m1]==codes[m2])
					return __clz(m1 ^ m2)+32;
				else
					return __clz(codes[m1] ^ codes[m2]);
			}

			__device__ int sign(int n){
				return (n>=0)?1:-1;
			}


			__host__ __device__ uint Morton_3D_Encode_10bit( uint index1, uint index2, uint index3 )
			{ // pack 3 10-bit indices into a 30-bit Morton code
				index1 &= 0x000003ff;
				index2 &= 0x000003ff;
				index3 &= 0x000003ff;
				index1 |= ( index1 << 16 );
				index2 |= ( index2 << 16 );
				index3 |= ( index3 << 16 );
				index1 &= 0x030000ff;
				index2 &= 0x030000ff;
				index3 &= 0x030000ff;
				index1 |= ( index1 << 8 );
				index2 |= ( index2 << 8 );
				index3 |= ( index3 << 8 );
				index1 &= 0x0300f00f;
				index2 &= 0x0300f00f;
				index3 &= 0x0300f00f;
				index1 |= ( index1 << 4 );
				index2 |= ( index2 << 4 );
				index3 |= ( index3 << 4 );
				index1 &= 0x030c30c3;
				index2 &= 0x030c30c3;
				index3 &= 0x030c30c3;
				index1 |= ( index1 << 2 );
				index2 |= ( index2 << 2 );
				index3 |= ( index3 << 2 );
				index1 &= 0x09249249;
				index2 &= 0x09249249;
				index3 &= 0x09249249;
				return( index1 | ( index2 << 1 ) | ( index3 << 2 ) );
			}

			__host__ __device__ uint morton_code( float3 center, float3 &c, float3 &d){

				center = center - c;

				uint x = center.x * d.x;
				uint y = center.y * d.y;
				uint z = center.z * d.z;
				return Morton_3D_Encode_10bit(x,y,z);
			}

			__device__ void aabbOfTri(simple_aabb &out_box,simple_triangle &tri){
#define MIN(_a,_b) ((_a)<(_b))? (_a):( _b)
#define MAX(_a,_b) ((_a)>(_b))? (_a):( _b)
				out_box.min.x = MIN(MIN(tri.a.x,tri.b.x),tri.c.x);
				out_box.min.y = MIN(MIN(tri.a.y,tri.b.y),tri.c.y);
				out_box.min.z = MIN(MIN(tri.a.z,tri.b.z),tri.c.z);

				out_box.max.x = MAX(MAX(tri.a.x,tri.b.x),tri.c.x);
				out_box.max.y = MAX(MAX(tri.a.y,tri.b.y),tri.c.y);
				out_box.max.z = MAX(MAX(tri.a.z,tri.b.z),tri.c.z);
#undef MIN
#undef MAX
			}




			__device__ int get_global_index(void)
			{
				int threadid = threadIdx.y * blockDim.x + threadIdx.x;
				int blockNumInGrid   = blockIdx.x  + gridDim.x  * blockIdx.y;
				int block_width =  blockDim.x * blockDim.y;
				int globalThreadNum = blockNumInGrid * block_width + threadid;
				return globalThreadNum;
			}



			__global__ void morton_codes2(float3* centers,uint* codes,uint* indices, float3 c, float3 d,int n){

				uint tid = get_global_index();
				if(tid<n){

					float3 center = centers[tid];
					center = center - c;

					uint x = center.x * d.x;
					uint y = center.y * d.y;
					uint z = center.z * d.z;
					codes[tid] = Morton_3D_Encode_10bit(x,y,z);
					indices[tid] = tid;
				}
			}


			__global__ void build_tree_karras(node_t *nodes,uint* codes,uint* indices,uint* parents,int n){
#define cpl(a,b) __cpl(a,b,codes,n)
				int i = get_global_index();
				if(i>=n*2-1)
					return;
				node_t &node = nodes[i];

				node.type_left_elems = 0;
				node.right_tris = 0;
				//leaf nodes are in the second half of the node array
				if(i>=n-1){
					//build leaf nodes
					node.tris(i-(n-1));
					node.elems(1);
					node.make_leaf();
					return;
				}

				int d = sign(cpl(i,i+1)-cpl(i,i-1));
				// Compute upper bound for the length of the range
				int cpl_min = cpl(i,i-d);
				int l_max = 2;
				while(cpl(i,i+l_max*d)>cpl_min)
					l_max = l_max*2;
				// Find the other end using binary search
				int l = 0;
				for(int t=l_max/2;t>=1;t=t/2){
					//cout<<t<<endl;
					if(cpl(i,i+(l+t)*d)>cpl_min)
						l = l + t;
				}
				int j = i + l * d;

				int from = i;
				int to = j;
				if(j<i){
					int tmp = i;
					from = j;
					to = tmp;
				}

				d = 1;
				// Find the split position using binary search
				int cpl_node = cpl(from,to);
				int s = 0;
				for(int t = divideBy2Ceiling(l);t>=1;t = divideBy2Ceiling(t)){
					if(cpl(from,from+(s+t)*d)>cpl_node)
						s = s + t;
					if(t<=1)
						break;

				}
				int mid = from + s * d + min(d,0);
				// Output child pointers
				int left = mid;
				int right = mid+1;

				if(from==mid){
					left = left+n-1;
				}
				if(to==mid+1){
					right = right +n-1 ;
				}

				node.left(left);
				node.right(right);
				parents[left] = i;
				parents[right] = i;
				node.make_inner();
			}

			__global__ void build_bounding_boxes_karras2(node_t *nodes,simple_aabb *boxes,uint* codes,uint* indices,uint* parents,uint* atomic_ints,int n){
#define MIN(_a,_b) ((_a)<(_b))? (_a):( _b)
#define MAX(_a,_b) ((_a)>(_b))? (_a):( _b)
#define MORTON_CODE(_index) ((_index>=(n-1))?codes[_index-n+1]:codes[_index])
#define EQUAL(v1,v2) (v1.x==v2.x && v1.y==v2.y && v1.z==v2.z)
				int current = get_global_index();
				if(current>=n)
					return;

				current = current+n-1;
				//hint to store this box in registers (the optimizer does this anyways)
				register cuda::simple_aabb box;

				//the block makes that the memory of the local variable 'tri' can be used for 'box2' after the block
				//the compiler would do this probably by itself, but just to be sure... ;)
				{
					//build leaf aabb
					node_t &node = nodes[current];
					box = boxes[indices[node.tris()]];
					node.volume(box);
				}

				cuda::simple_aabb box2;
				int last;
				//bottom up
				while(current!=0){
					last = current;
					uint code = MORTON_CODE(current);
					current = parents[current];
					//atomic operations work in global memory!
					uint old = atomicAdd(&atomic_ints[current],code);
					//the second thread passes
					if(old==0)
						return;

					//load the correct box out of memory
					if(nodes[current].left()==last){
						box2 = nodes[nodes[current].right()].gen_box();
					}else{
						box2 = nodes[nodes[current].left()].gen_box();
					}

					bool merge = false;
					//        if(EQUAL(box.min,box2.min) && EQUAL(box.max,box2.max))
					if(code == old)
						merge = true;

					//build union of bounding boxes
					box.min.x = MIN(box.min.x,box2.min.x);
					box.min.y = MIN(box.min.y,box2.min.y);
					box.min.z = MIN(box.min.z,box2.min.z);

					box.max.x = MAX(box.max.x,box2.max.x);
					box.max.y = MAX(box.max.y,box2.max.y);
					box.max.z = MAX(box.max.z,box2.max.z);

					nodes[current].volume(box);

					if(merge){
						if(!nodes[nodes[current].right()].inner() && !nodes[nodes[current].left()].inner()){


							int tris = min(nodes[nodes[current].right()].tris(),nodes[nodes[current].left()].tris());
							int length = nodes[nodes[current].right()].elems() + nodes[nodes[current].left()].elems();

							nodes[current].type_left_elems = 0;
							nodes[current].right_tris = 0;

							nodes[current].tris(tris);
							nodes[current].elems(length);
							nodes[current].make_leaf();
						}
					}




				}
#undef MIN
#undef MAX
#undef MORTON_CODE
#undef EQUAL
			}


			__global__ void initTriangles(cuda::simple_triangle *triangles, cuda::simple_triangle *src, simple_aabb* boxes, float3* centers, int n){

				int id = get_global_index();
				if(id>=n)
					return;

				cuda::simple_triangle tri = src[id];
				triangles[id] = tri;

				cuda::simple_aabb box;
				aabbOfTri(box,tri);

				boxes[id] = box;

				centers[id] = center_of_gravity(tri);
			}

			__global__ void invert_triangle_permutation_cuda(uint *indices, cuda::simple_triangle *triangles, int n) {
			}
		}




		void setupCudaMemory(uint **indices, uint **codes, node_t **nodes, uint **parents, cuda::simple_aabb **boxes, float3 **centers, cuda::simple_triangle **t_out, int n){
			int size = n*sizeof(uint);
			checked_cuda(cudaMalloc((void **)indices, size));
			checked_cuda(cudaMalloc((void **)codes, size));
			size = n*sizeof(simple_aabb);
			checked_cuda(cudaMalloc((void **)boxes, size));
			size = n*sizeof(float3);
			checked_cuda(cudaMalloc((void **)centers, size));

			int node_size = (n*2-1)*sizeof(node_t);
			checked_cuda(cudaMalloc((void **)nodes, node_size));
			node_size = (n*2-1)*sizeof(uint);
			checked_cuda(cudaMalloc((void **)parents, node_size));

			checked_cuda(cudaMalloc((void**)t_out, sizeof(cuda::simple_triangle)*n));
		}

		void* mallocCuda(int size){
			void* d_ptr;
			checked_cuda(cudaMalloc(&d_ptr, size));
			return d_ptr;
		}


		void downloadArray(void* d_ptr,void* h_ptr,int size){
			checked_cuda( cudaMemcpy(h_ptr, d_ptr, size, cudaMemcpyDeviceToHost) );
		}

		void* uploadArray(void* h_ptr,int size){
			void* d_ptr;
			checked_cuda(cudaMalloc(&d_ptr, size));
			checked_cuda( cudaMemcpy(d_ptr,h_ptr, size, cudaMemcpyHostToDevice) );
			return d_ptr;
		}



		float calculateMortonCodes2(float3* centers , simple_aabb voxel_bounds, uint* d_codes,uint *d_indices,int n){
			float time = 0;

			START_CUDA_TIMER;
			dim3 block_size(8, 8);
			dim3 grid_size;
			grid_size.x = n / (block_size.x*block_size.y) + 1;
			int blocks = n / (block_size.x*block_size.y) + 1;

			float3 d;
			d.x = GRIDSIZE/(voxel_bounds.max.x - voxel_bounds.min.x);
			d.y = GRIDSIZE/(voxel_bounds.max.y - voxel_bounds.min.y);
			d.z = GRIDSIZE/(voxel_bounds.max.z - voxel_bounds.min.z);
			k::morton_codes2<<<blocks, block_size>>>(centers, d_codes,d_indices,voxel_bounds.min,d,n);
			STOP_CUDA_TIMER(time);
			return time;
		}

		//ptxas info    : Used 14 registers, 68 bytes cmem[0]
		float buildTreeKarras(node_t* nodes,  uint* d_codes,uint *d_indices,uint* parents,int n){
			float time = 0;
			START_CUDA_TIMER;
			dim3 block_size(16, 16);
			dim3 grid_size;
			grid_size.x = (n*2-1) / (block_size.x*block_size.y) + 1;

			k::build_tree_karras<<<grid_size, block_size>>>(nodes, d_codes,d_indices,parents,n);
			STOP_CUDA_TIMER(time);
			return time;
		}

		float buildBoundingBoxesKarras2(node_t* nodes,simple_aabb *boxes,  uint* d_codes,uint *d_indices,uint* parents,int n){
			float time = 0;
			START_CUDA_TIMER;
			uint* d_atomic_ints;
			checked_cuda(cudaMalloc((void **)&d_atomic_ints, (n-1)*sizeof(uint)));
			cudaMemset(d_atomic_ints, 0, (n-1)*sizeof(uint));

			dim3 block_size(16, 16);
			dim3 grid_size;
			grid_size.x = (n) / (block_size.x*block_size.y) + 1;

			k::build_bounding_boxes_karras2<<<grid_size, block_size>>>(nodes,boxes, d_codes,d_indices,parents,d_atomic_ints,n);
			STOP_CUDA_TIMER(time);
			return time;
		}



		float initTriangles(cuda::simple_triangle *triangles, cuda::simple_triangle *src, simple_aabb* boxes, float3* centers, int n){
			float time = 0;
			START_CUDA_TIMER;

			dim3 block_size(8, 8);
			dim3 grid_size;
			int blocks = n / (block_size.x*block_size.y) + 1;
			k::initTriangles<<<blocks, block_size>>>(triangles, src, boxes,centers,n);

			STOP_CUDA_TIMER(time);
			return time;
		}


		struct reduce_max : public binary_function <simple_aabb,simple_aabb,simple_aabb>
		{
#define MIN(_a,_b) ((_a)<(_b))? (_a):( _b)
#define MAX(_a,_b) ((_a)>(_b))? (_a):( _b)
			__device__ simple_aabb operator () ( const simple_aabb box1, const simple_aabb box2)
			{
				simple_aabb box = box1;

				box.min.x = MIN(box.min.x,box2.min.x);
				box.min.y = MIN(box.min.y,box2.min.y);
				box.min.z = MIN(box.min.z,box2.min.z);

				box.max.x = MAX(box.max.x,box2.max.x);
				box.max.y = MAX(box.max.y,box2.max.y);
				box.max.z = MAX(box.max.z,box2.max.z);

				return box;
			}
#undef MIN
#undef MAX
		};

		float calculateVoxelBounds(simple_aabb &result,simple_aabb* d_boxes,int n){
#define INFINITE 100000000000.0f
#define MAKENEGATIVEBOX(box) box.min = make_float3(INFINITE,INFINITE,INFINITE);box.max = make_float3(-INFINITE,-INFINITE,-INFINITE);

			float time = 0;
			START_CUDA_TIMER;

			simple_aabb box;
			MAKENEGATIVEBOX(box)
				thrust::device_ptr<cuda::simple_aabb> boxes(d_boxes);
			result = thrust::reduce(boxes,boxes+n,box,reduce_max());


			STOP_CUDA_TIMER(time);
			return time;
#undef INFINITE
#undef MAKENEGATIVEBOX
		}

		float sortMortonCodes(uint* d_codes,uint *d_indices,int n){
			float time = 0;
			START_CUDA_TIMER;

#ifdef B40C
			// Use b40c library
			b40c::radix_sort::Enactor enactor;
			b40c::util::DoubleBuffer<uint, uint> sort_storage(d_codes, d_indices);
			enactor.Sort<0, 32, b40c::radix_sort::SMALL_SIZE>(sort_storage, n);
#else
			// Use thrust library
			thrust::device_ptr<uint> values(d_indices);
			thrust::device_ptr<uint> keys(d_codes);
			thrust::sort_by_key(keys, keys + n, values);
#endif

			STOP_CUDA_TIMER(time);
			return time;
		}

		struct permutation {
			simple_triangle *triangles;
			__host__ __device__ permutation(simple_triangle *tri) : triangles(tri) {}
			__host__ __device__ simple_triangle operator()(int id) {
				return triangles[id];
			}
		};
		float invert_triangle_permutation_cuda(uint *indices, simple_triangle *triangles, simple_triangle *t_out, int n) {
			float time = 0;
			START_CUDA_TIMER;

			thrust::device_ptr<simple_triangle> tri_start(triangles);
			thrust::device_ptr<simple_triangle> tri_end(triangles+n);
			thrust::device_ptr<simple_triangle> out_start(t_out);
			thrust::device_ptr<uint> ids_start(indices);
			thrust::device_ptr<uint> ids_end(indices+n);

			thrust::transform(ids_start, ids_end, out_start, permutation(triangles));

			STOP_CUDA_TIMER(time);
			return time;
		}
	}


}
