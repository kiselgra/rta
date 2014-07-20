#ifndef __RTA_LBVH_H__ 
#define __RTA_LBVH_H__ 

#include "bbvh-cuda.h"

namespace rta {
	namespace cuda {

// 		void setupCudaMemory(uint** indices, uint** codes,cuda::bbvh::node<cuda::simple_aabb>** nodes, uint** parents,cuda::simple_aabb** boxes, float3** centers,int n);
		void setupCudaMemory(uint** indices, uint** codes,void **nodes, int node_size, uint** parents,cuda::simple_aabb** boxes, float3** centers, cuda::simple_triangle **t_out, int n);

		template<box_t__and__tri_t, typename bvh_t_> class binary_lbvh : public bvh_t_ {
			public:
				declare_traits_types;
				typedef bvh_t_ bvh_t;
				typedef uint32_t link_t;
				typedef typename bvh_t::node_t node_t;

				uint *index_buffer;
				uint index_buffer_length;

				binary_lbvh() : index_buffer(0), index_buffer_length(0) {}

				virtual void take_index_array(uint *array, uint length) {
					index_buffer = array;
					index_buffer_length = length;
				}

				tri_t* triangle_ptr() { return this->triangle_data.data; }
				int triangle_count() { return this->triangle_data.n; }

				virtual std::string identification() { return "lbvh based on the ba-sidaruck"; }
		};

		/*! \brief Construct a LBVH (see ``Fast BVH Construction on GPUs'' by Lauterbach et al.)
		 *         as described in ``Maximizing Parallelism in the Construction of BVHs, Octrees, and k-d Trees'' by Karras.
		 *         The Implementation is adapted from sidaruck's bachelor thesis.
		 *
		 *  The bvh_t template parameter *must* be an instantiation of the \ref binary_lbvh types.
		 *  To be less confusing we chose not to take the base type of the corresponding binary_lbvh type.
		 *  
		 *  \attention Cannot handle triangle list sizes of 0, atm.
		 */
		template<box_t__and__tri_t, typename bvh_t_> class lbvh_constructor : public basic_acceleration_structure_constructor<forward_traits> {
			public:
				declare_traits_types;
				typedef bvh_t_ bvh_t;
				typedef typename bvh_t::node_t node_t;
				typedef typename bvh_t::link_t link_t;
				bool verbose;

			protected:
				tri_t* triangles;
				int tri_count, last_tri_count;
				node_t *nodes;
				uint* parents;
				bvh_t *bvh;
				uint *d_indices;
				uint *d_morton_codes;
				node_t *d_nodes;
				uint *d_parents;
				tri_t *d_triangles, *d_triangles_out;
				box_t *d_boxes;
				vec3_t *d_centers;

				//forwarding cuda functions
				inline float calculateMortonCodesCUDA2(vec3_t *centers,box_t &voxel_bounds, uint* d_codes,uint *d_indices,int n){
					float calculateMortonCodes2(float3 *centers, box_t voxel_bounds, uint* codes,uint *d_indices,int n);
					return calculateMortonCodes2((float3*)centers, voxel_bounds,d_codes,d_indices,n);
				}

				inline float sortMortonCodesCUDA(uint* d_codes,uint *d_indices,int n){
					float sortMortonCodes(uint* d_codes,uint *d_indices,int n);
					return sortMortonCodes(d_codes,d_indices,n);
				}

				inline float buildTreeCUDA(node_t *d_nodes, uint *d_codes, uint *d_indices, uint *parents, int n) {
					float buildTreeKarras(node_t *nodes, uint *d_codes, uint *d_indices, uint *parents, int node_count);
					return buildTreeKarras(d_nodes, d_codes, d_indices, parents,n);
				}
				inline float buildBoundingBoxesCUDA(node_t *d_nodes, tri_t *triangles, uint *d_codes, uint *d_indices, uint *parents, int n) {
					float buildBoundingBoxesKarras(node_t *nodes, tri_t *triangles, uint *d_codes, uint *d_indices, uint *parents, int node_count);
					return buildBoundingBoxesKarras(d_nodes, triangles, d_codes, d_indices, parents, n);
				}

				inline float buildBoundingBoxesCUDA2(node_t* d_nodes,box_t *boxes, uint* d_codes,uint *d_indices,uint* parents,int n){
					float buildBoundingBoxesKarras2(node_t* nodes, box_t *boxes, uint* d_codes,uint *d_indices,uint* parents,int node_count);
					return buildBoundingBoxesKarras2(d_nodes,boxes,d_codes,d_indices,parents,n);
				}

				inline float initTrianglesCUDA(tri_t *d_triangles, tri_t *src, box_t* d_boxes,vec3_t *d_centers,int n){
					float initTriangles(tri_t *d_triangles, tri_t *d_tri_src, box_t* d_boxes,float3 *d_centers,int n);
					return initTriangles(d_triangles, src, d_boxes,(float3*)d_centers, n);
				}

				inline float voxelBoundsCUDA(box_t &result,box_t* d_boxes,int n){
					float calculateVoxelBounds( box_t &result,box_t* d_boxes,int n);
					return calculateVoxelBounds(result, d_boxes, n);
				}

				inline float invert_triangle_permutation(uint *indices, tri_t *triangles, tri_t *t_out, int n) {
					float invert_triangle_permutation_cuda(uint *indices, tri_t *triangles, tri_t *t_out, int n);
					return invert_triangle_permutation_cuda(indices,triangles,t_out,n);
				}


			public:

				lbvh_constructor() {
					triangles = 0;
					last_tri_count = 0;
					tri_count = 0;
					nodes = 0;
					parents = 0;
					bvh = 0;
					d_indices = 0;
					d_morton_codes = 0;
					d_nodes = 0;
					d_parents = 0;
					d_triangles = 0; 
					d_triangles_out = 0;
					d_boxes = 0;
					d_centers = 0;
					verbose = false;
				}
				
				void free_data() {
					checked_cuda(cudaFree(d_indices));        d_indices = 0;
					checked_cuda(cudaFree(d_morton_codes));   d_morton_codes = 0;
					checked_cuda(cudaFree(d_nodes));          d_nodes = 0;
					checked_cuda(cudaFree(d_parents));        d_parents = 0;
					checked_cuda(cudaFree(d_boxes));          d_boxes = 0;
					checked_cuda(cudaFree(d_centers));        d_centers = 0;
					// these are not the triangles handed in, but our copy of em.
					checked_cuda(cudaFree(d_triangles));	  d_triangles = 0;
				}

				~lbvh_constructor() {
					free_data();
				}

				bvh_t* build(typename bvh_t::tri_t::input_flat_triangle_list_t *tris) {
					triangles = (tri_t*)tris->triangle;
					tri_count = tris->triangles;
					return buildCUDA();
				}

				bvh_t* buildCUDA(){
					if (verbose) std::cout<<"Triangles: "<<this->tri_count<<std::endl;
					if (verbose) std::cout<<"Building BVH on GPU..."<<std::endl;
					//device pointers
					if (d_indices == 0 || last_tri_count != tri_count) {
						if (d_indices != 0)
							free_data();
						setupCudaMemory(&d_indices, &d_morton_codes, (void**)&d_nodes, sizeof(node_t), &d_parents, &d_boxes, (float3**)&d_centers, &d_triangles, this->tri_count);
					}
					last_tri_count = tri_count;


					this->bvh = new bvh_t;
					if (verbose) std::cout<<"gpu triangle list"<<std::endl;
					this->bvh->take_triangle_array((cuda::simple_triangle*)this->triangles,this->tri_count);
// 					d_triangles = (tri_t*)this->bvh->triangle_data.data;
					d_triangles_out = (tri_t*)this->bvh->triangle_data.data;
					box_t voxel_bounds;

					if (verbose) std::cout<<"========================================================="<<std::endl;
					float time_init_triangles = this->initTrianglesCUDA(d_triangles,d_triangles_out,d_boxes,d_centers,this->tri_count);
					checked_cuda(cudaDeviceSynchronize());
					if (verbose) std::cout<<"Time for initializing Triangles: "<<time_init_triangles<<" ms"<<std::endl;


					float time_voxel_bounds = this->voxelBoundsCUDA(voxel_bounds,d_boxes,this->tri_count);
					if (verbose) std::cout<<"Time for calculating Voxel Bounds: "<<time_voxel_bounds<<" ms"<<std::endl;

					if (verbose) std::cout<<"Preprocessing finished. Total time: "<<time_init_triangles+time_voxel_bounds<<" ms"<<std::endl;

					if (verbose) std::cout<<"========================================================="<<std::endl;
					//        float time_morton = calculateMortonCodesCUDA(d_triangles,voxel_bounds,d_morton_codes,d_indices,tri_count);
					float time_morton = this->calculateMortonCodesCUDA2(d_centers,voxel_bounds,d_morton_codes,d_indices,this->tri_count);
					if (verbose) std::cout<<"Time for calculating Morton Codes: "<<time_morton<<" ms"<<std::endl;

					float time_sort = this->sortMortonCodesCUDA(d_morton_codes,d_indices,this->tri_count);
					if (verbose) std::cout<<"Time for sorting Morton Codes: "<<time_sort<<" ms"<<std::endl;

					float time_tree = this->buildTreeCUDA(d_nodes,d_morton_codes,d_indices,d_parents,this->tri_count);
					if (verbose) std::cout<<"Time for tree construction: "<<time_tree<<" ms"<<std::endl;

					//        float time_boxes = buildBoundingBoxesCUDA(d_nodes,d_triangles,d_morton_codes,d_indices,d_parents,tri_count);
					float time_boxes = this->buildBoundingBoxesCUDA2(d_nodes,d_boxes,d_morton_codes,d_indices,d_parents,this->tri_count);
					if (verbose) std::cout<<"Time for building the bounding boxes: "<<time_boxes<<" ms"<<std::endl;
					
					float time_permute = this->invert_triangle_permutation(d_indices,d_triangles,d_triangles_out,this->tri_count);
					if (verbose) std::cout<<"Time for inversion of triangle permutation: "<<time_permute<<" ms"<<std::endl;


					if (verbose) std::cout<<"BVH finished. Total time: "<<time_morton+time_sort+time_tree+time_boxes<<" ms"<<std::endl;

					if (verbose) std::cout<<"========================================================="<<std::endl;

					//        std::cout<<"Node count: "<<tri_count*2-1<<" Triangle count: "<<tri_count<<std::endl;
					this->bvh->take_node_array(d_nodes, this->tri_count*2-1);
					this->bvh->take_index_array(d_indices, this->tri_count);

					return this->bvh;
				}

				virtual bool expects_host_triangles() { return false; }

				virtual std::string identification() { return "lbvh constructor" ; }
			};

	}
}

#endif

