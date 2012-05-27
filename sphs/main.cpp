#include <libplyloader/plyloader.h>

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <fstream>
#include <float.h>
#include <cstring>

#include "cmdline.h"
#include "color.h"

using namespace std;


typedef std::map<std::string, std::string> header_info_map;
header_info_map base_header, diff_header;

void parse_comment_meta_info(ply &p, header_info_map &map) {
	for (string comment : p.comments) {
		if (comment.substr(0, strlen("meta ")) != "meta ") continue;
		comment = comment.substr(strlen("meta "));
		size_t pos = comment.find_first_of(":");
		if (pos == string::npos) continue;
		string key = comment.substr(0,pos);
		while (pos+1 < comment.length() && isspace(comment[pos+1]))
			++pos;
		string val = comment.substr(pos+1);
		map[key] = val;
	}
}

bool operator==(const vec3f &a, const vec3f &b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

std::ostream& operator<<(std::ostream &out, const vec3f &v) {
	out << v.x << " " << v.y << " " << v.z;
}

float in_units(float f) { return f/cmdline.unit; }

void clear_duplicate_vertices(ply &p, ply::element_t *elem) {
	std::vector<vec3f> verts;
	verts.reserve(elem->count);
	int x = elem->index_of("x"),
		y = elem->index_of("y"),
		z = elem->index_of("z");

	for (int i = 0; i < elem->count; ++i) {
		vec3f curr = { elem->ref(i, x), elem->ref(i, y), elem->ref(i, z) };
		for (int check = 0; check < verts.size(); ++check)
			if (verts[check] == curr) {
				p.mark_vertex_for_deletion(i, check);
				break;
			}
		verts.push_back(curr);
	}

	p.delete_marked_vertices(elem->name);
}

bool duplicate_vertices(ply &p, ply::element_t *elem) {
	std::vector<vec3f> verts;
	verts.reserve(elem->count);
	int x = elem->index_of("x"),
		y = elem->index_of("y"),
		z = elem->index_of("z");

	for (int i = 0; i < elem->count; ++i) {
		vec3f curr = { elem->ref(i, x), elem->ref(i, y), elem->ref(i, z) };
		for (auto v : verts)
			if (v == curr) {
				cout << "At vertex " << i << endl;
				cout << "   new vertex: " << v.x << "\t" << v.y << "\t" << v.z << endl;
				cout << "   old vertex: " << curr.x << "\t" << curr.y << "\t" << curr.z << endl;
				return true;
			}
		verts.push_back(curr);
	}
	return false;
}

//! calls f for each vertex. the supplied value for rps is already converted to the desired unit (rps,krps,mrps).
void apply(std::function<void(vec3f, float, int)> f, ply::element_t *elem) {
	int x = elem->index_of("x"),
		y = elem->index_of("y"),
		z = elem->index_of("z"),
		m = elem->index_of("rps");
	for (int i = 0; i < elem->count; ++i) {
		vec3f curr = { elem->ref(i, x), elem->ref(i, y), elem->ref(i, z) };
		float x_rps = in_units(elem->ref(i, m));
		f(curr, x_rps, i);
	}
}

//! calls f for each vertex. the supplied value for rps is already converted to the desired unit (rps,krps,mrps).
void apply(std::function<void(vec3f, vec3f, float, float, int)> f, ply::element_t *base_elem, ply::element_t *diff_elem) {
	int x = base_elem->index_of("x"),   xx = diff_elem->index_of("x"),
		y = base_elem->index_of("y"),   yy = diff_elem->index_of("y"),
		z = base_elem->index_of("z"),   zz = diff_elem->index_of("z"),
		m = base_elem->index_of("rps"), mm = diff_elem->index_of("rps");
	for (int i = 0; i < base_elem->count; ++i) {
		vec3f base_curr = { base_elem->ref(i, x),  base_elem->ref(i, y),  base_elem->ref(i, z) };
		vec3f diff_curr = { diff_elem->ref(i, xx), diff_elem->ref(i, yy), diff_elem->ref(i, zz) };
		float base_x_rps = in_units(base_elem->ref(i, m));
		float diff_x_rps = in_units(diff_elem->ref(i, mm));
		f(base_curr, diff_curr, base_x_rps, diff_x_rps, i);
	}
}

// single file operators

std::vector<float> find_min_max_time(ply::element_t *elem) {
	float min = FLT_MAX,
		  max = 0;
	apply([&](vec3f pos, float x_rps, int i) {
		  	if (x_rps < min) min = x_rps;
			if (x_rps > max) max = x_rps;
		  },
		  elem);
	return { min, max };
}

void scale_between(ply &p, ply::element_t *elem, float min, float max) {
	p.allocate_new_vertex_data({"red", "green", "blue"}, elem->name, ply::property_t::t_uint8);
	int r = elem->index_of("red"),
		g = elem->index_of("green"),
		b = elem->index_of("blue");
	apply([&](vec3f pos, float x_rps, int i) {
				float scaled = std::min(1.0f, std::max(0.0f, (x_rps-min)/(max-min)));
				float hue = scaled * 120;
				vec3f col = rgb(hue);
				elem->ref(i, r) = col.x * 255;
				elem->ref(i, g) = col.y * 255;
				elem->ref(i, b) = col.z * 255;
			},
			elem);
}

void scale_by_min_max(ply &p, ply::element_t *elem) {
	std::vector<float> mm = find_min_max_time(elem);
	float min = mm[0],
		  max = mm[1];
	scale_between(p, elem, min, max);
}

void scale_from_0_to_max(ply &p, ply::element_t *elem) {
	float max = find_min_max_time(elem)[1];
	scale_between(p, elem, 0, max);
}

// two-file operators

bool equal_vertices(ply::element_t *base, ply::element_t *diff) {
	bool res = true;
	apply([&](vec3f left, vec3f right, float ra, float rb, int i) {
				if (!(left == right))
					res = false;
			},
			base,
			diff);
	return res;
}

void divide_diff_by_base(ply &p, ply::element_t *base, ply::element_t *diff) {
	bool res = true;
	p.allocate_new_vertex_data({"red", "green", "blue"}, base->name, ply::property_t::t_uint8);
	int r = base->index_of("red"),
		g = base->index_of("green"),
		b = base->index_of("blue");
	apply([&](vec3f left, vec3f right, float ra, float rb, int i) {
				cout << rb << "\t/\t" << ra << "\t=\t" << rb/ra << endl;
				float scaled = rb/ra;
				float hue = scaled * 120;
				if (hue > 120) hue += 60;
				vec3f col = rgb(hue);
				base->ref(i, r) = col.x * 255;
				base->ref(i, g) = col.y * 255;
				base->ref(i, b) = col.z * 255;
			},
			base,
			diff);
}

// checking

void check_for_duplicates(ply &p, const std::string &elem_name, const std::string &filename) {
	auto elem = p.element("vertex");
	if (duplicate_vertices(p, elem)) {
		if (cmdline.clear_duplicates) {
			clear_duplicate_vertices(p, elem);
			save_ply_file(filename + ".nodup", &p);
			cout << "Written duplicate free version of " << filename << " to " << filename << ".nodup" << endl;
			exit(EXIT_SUCCESS);
		}
		cerr << "Duplicate vertices found!" << endl;
		if (!cmdline.ignore_duplicates)
			exit(EXIT_FAILURE);
		cerr << "The measurements may be biased by this!" << endl;
	}
}

void check_ply_data(ply::element_t *elem, const std::string &filename, bool verbose) {
	int vertices = elem->count;
	vec3f *vertex = new vec3f[vertices];
	float *timings = new float[vertices];
	int x = elem->index_of("x"),
		y = elem->index_of("y"),
		z = elem->index_of("z"),
		m = elem->index_of("rps");
	if (x == -1 || y == -1 || z == -1) {
		cerr << "file " << filename << " is missing a x/y/z property in the 'vertex' element." << endl;
		exit(EXIT_FAILURE);
	}
	if (m == -1) {
		cerr << "file " << filename << " is missing the 'rps' property in the 'vertex' element." << endl;
		exit(EXIT_FAILURE);
	}
	
	if (verbose) {
		float sum = 0;
		for (int i = 0; i < vertices; ++i) 
			sum += in_units(elem->ref(i, m));
		float avg = sum/vertices;

		auto mm = find_min_max_time(elem);
		cout << filename << ": average of " << avg << " " << cmdline.unit_string << "rps, [ " << mm[0] << " : " << mm[1] << " ]" << endl;
	}
}

enum difference_t { allowed, dramatic };
difference_t handle_difference(const std::string &key, const std::string &base_val, const std::string &diff_val) {
	cerr << "META DATA DIFFERENCE: " << endl;
	cerr << "--------------------- " << endl;
	cerr << key << " (base) " << base_val << endl;
	for (int i = 0; i < key.length(); ++i) cout << " ";
	cout << " (diff) " << diff_val << endl;
	auto critical = { "host" };
	for (auto c : critical)
		if (c == key)
			return dramatic;
	return allowed;
}

int main(int argc, char **argv)
{	
	parse_cmdline(argc, argv);

	if (cmdline.mode == Cmdline::none) {
		cerr << "You'll have to specify a MODE OF OPERATION" << endl;
		exit(EXIT_FAILURE); 
	}

	if (cmdline.base_file_name == "") {
		cerr << "You'll have to specify an input file!" << endl;
		exit(EXIT_FAILURE); 
	}

	// load file(s) and check for duplicate vertices
	
	ply base_sphere;
	load_ply_file(cmdline.base_file_name.c_str(), &base_sphere);
	auto base_elem = base_sphere.element("vertex");
	check_for_duplicates(base_sphere, "vertex", cmdline.base_file_name);
	
	ply diff_sphere;
	ply::element_t *diff_elem = 0;
	bool two_files = cmdline.diff_file_name != "";
	if (two_files) {
		load_ply_file(cmdline.diff_file_name.c_str(), &diff_sphere);
		diff_elem = diff_sphere.element("vertex");
		check_for_duplicates(diff_sphere, "vertex", cmdline.diff_file_name);
	}

	// check file contents and parse file meta data

	check_ply_data(base_elem, cmdline.base_file_name, true);
	if (two_files)
		check_ply_data(diff_elem, cmdline.diff_file_name, true);

	parse_comment_meta_info(base_sphere, base_header);
	if (two_files)
		parse_comment_meta_info(diff_sphere, diff_header);
 
	// check consistency of the supplied files

	if (two_files) {
		if (base_elem->count != diff_elem->count) {
			cerr << "The files consist of a different number of vertices. This won't work." << endl;
			exit(EXIT_FAILURE);
		}
		if (!equal_vertices(base_elem, diff_elem)) {
			cerr << "The vertices of the two files do not match." << endl;
			exit(EXIT_FAILURE);
		}
		for (auto base : base_header) {
			if (diff_header.count(base.first) == 0) {
				cerr << "file " << cmdline.diff_file_name << " is missing meta data field " << base.first << endl;
				exit(EXIT_FAILURE);
			}
			auto diff_val = diff_header[base.first];
			if (base.second != diff_val)
				if (handle_difference(base.first, base.second, diff_val) == dramatic) {
					cerr << "dramatic difference in file meta data!" << endl;
					exit(EXIT_FAILURE);
				}
		}
		int differences = 0;
		for (auto diff : diff_header) {
			if (base_header.count(diff.first) == 0) {
				++differences;
				cerr << "file " << cmdline.base_file_name << " is missing meta data field " << diff.first << endl;
				exit(EXIT_FAILURE);
			}
		}
		if (differences > cmdline.meta_differences) {
			cerr << "too many meta data differences." << endl;
			exit(EXIT_FAILURE);
		}
	}
	
	// ...
	
	if (cmdline.mode == Cmdline::check)
		exit(EXIT_SUCCESS);

	if (two_files) {
		if (cmdline.mode == Cmdline::perf_diff_by_base)
			divide_diff_by_base(base_sphere, base_elem, diff_elem);
		else {
			cerr << "Chosen mode is not valid when two files are spcified." << endl;
			exit(EXIT_FAILURE);
		}
	}
	else {
		if (cmdline.mode == Cmdline::scale_min_max)
			scale_by_min_max(base_sphere, base_elem);
		else if (cmdline.mode == Cmdline::scale_0_max)
			scale_from_0_to_max(base_sphere, base_elem);
		else {
			cerr << "Chosen mode is not valid when only one file is spcified." << endl;
			exit(EXIT_FAILURE);
		}
	}

	save_ply_file(cmdline.output_file_name, &base_sphere);
	cout << "Output written to " << cmdline.output_file_name << endl;

	return 0;
}


