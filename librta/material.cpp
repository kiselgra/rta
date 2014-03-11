#include <png.h>

#include "material.h"

#include <vector>
#include <map>
#include <list>
#include <stdexcept>
#include <sys/stat.h>
#include <png++/png.hpp>

using namespace std;

namespace rta {
	
	list<std::string> image_paths;
	struct initial_paths
	{
		initial_paths()
		{
			image_paths.push_back(".");
		}
	} initialize_paths;
	
	bool file_exists(const string &name)
	{
		struct stat st;
		int stat_res = stat(name.c_str(), &st);
		if (stat_res == 0)
			return true;
		else
			return false;
	}

	string find_file_default_version(const std::string &basename)
	{
		if (basename[0] == '/') {
			if (file_exists(basename))
				return basename;
		}
		else
			for (string p : image_paths)
			{
				if (p[p.length()-1] != '/')
					p += "/";
				p += basename;
				if (file_exists(p.c_str()))
					return p;
			}
		return basename;
	}

	void append_image_path(const std::string &path) {
		image_paths.push_back(path);
	}

	void prepend_image_path(const std::string &path) {
		image_paths.push_front(path);
	}

	void pop_image_path_front() {
		image_paths.pop_front();
	}

	void pop_image_path_back() {
		image_paths.pop_back();
	}

	static map<std::string, texture*> texture_map;

	texture::texture(const std::string &filename)
    : data(0), w(0), h(0), repeat_mode_s(wrap), repeat_mode_t(wrap) {
		cout << "looking for " << filename << endl;
		string found = find_file_default_version(filename);
		png::image<png::rgba_pixel> img(found);
		w = img.get_width();
		h = img.get_height();
		vec3f *layer0 = new vec3f[w*h];
		for (unsigned int y = 0; y < h; ++y)
			for (unsigned int x = 0; x < w; ++x) {
				layer0[y*w+x].x = img[y][x].red/255.0f;
				layer0[y*w+x].y = img[y][x].green/255.0f;
				layer0[y*w+x].z = img[y][x].blue/255.0f;
			}
		data.push_back(layer0);
	}

	texture* load_texture(const std::string &filename) {
		auto it = texture_map.find(filename);
		if (it != texture_map.end())
			return it->second;
		texture *tex = new texture(filename);
		texture_map[filename] = tex;
		return tex;
	}

	vec3f texture::sample(float s, float t) {
		float x = s*w;
		float y = (1.0f-t)*h;
		int nearest_x = int(x);
		int nearest_y = int(y);
		while (repeat_mode_s == wrap && nearest_x > w) nearest_x -= w;
		while (repeat_mode_t == wrap && nearest_y > h) nearest_y -= h;
		while (repeat_mode_s == wrap && nearest_x < 0) nearest_x += w;
		while (repeat_mode_t == wrap && nearest_y < 0) nearest_y += h;
		
		return data[0][nearest_y*w+nearest_x];
	}
	
	material_t::material_t(const std::string &name, const vec3f &diffuse_color, const std::string &diffuse_texname) 
	: name(name),
	  diffuse_color(diffuse_color), diffuse_texture_name(diffuse_texname), diffuse_texture(0) {
		if (diffuse_texture_name != "")
			diffuse_texture = load_texture(diffuse_texture_name);
	}
		
	vec3f material_t::operator()(vec2_t tc) {
		vec3f diff = diffuse_color;
		vec3f tex = diffuse_texture->sample(tc.x, tc.y);
		mul_components_vec3f(&diff, &diff, &tex);
		return diff;
	}
	
	
	static vector<material_t*> materials;

	int register_material(material_t *mat) {
		materials.push_back(mat);
		return materials.size()-1;
	}

	int material(const std::string &name) {
		for (int i = 0; i < materials.size(); ++i)
			if (materials[i]->name == name)
				return i;
		return -1;
	}

	material_t* material(int id) {
		if (id < 0 || id >= materials.size())
			throw std::runtime_error("material index out of bounds!");
		return materials[id];
	}
}

/* vim: set foldmethod=marker: */

