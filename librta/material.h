#ifndef __RTA_MATERIAL_H__ 
#define __RTA_MATERIAL_H__ 

#include "basic_types.h"

#include <string>

namespace rta {

	// we use plain *vec3f* for colors,
	// but still rta::vec3_t etc for vectors.
	
	void append_image_path(const std::string &path);
	void prepend_image_path(const std::string &path);
	void pop_image_path_front();
	void pop_image_path_back();

	struct texture {
		std::string filename;
		std::vector<vec3f*> data;
		int w, h;
		enum repeat_mode { clamp, wrap };
		repeat_mode repeat_mode_s, repeat_mode_t;
		texture(const std::string &filename);
		vec3f sample(float s, float t);
	};

	texture* load_texture(const std::string &filename);

	struct material_t {
		std::string name;

		vec3f diffuse_color;
		std::string diffuse_texture_name;
		texture *diffuse_texture;

		material_t(const std::string &name, const vec3f &diffuse_color, const std::string &diffuse_texname = "");

		vec3f operator()() {
			return diffuse_color;
		}
		vec3f operator()(vec2_t tc);
	};

	int register_material(material_t *mat);
	material_t* material(int id);
	int material(const std::string &name);
}

#endif

