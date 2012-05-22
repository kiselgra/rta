#include <libplyloader/plyloader.h>

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <fstream>

#include "cmdline.h"

using namespace std;

bool operator==(const vec3f &a, const vec3f &b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}


void clear_duplicate_vertices(ply &p, ply::element_t *elem) {
	std::vector<vec3f> verts;
	verts.reserve(elem->count);
	vec3f *vdata = p.flat_triangle_data("x", "y", "z", [](float f){ return f; });

	for (int i = 0; i < elem->count; ++i) {
		for (int check = 0; check < verts.size(); ++check)
			if (verts[check] == vdata[i]) {
				p.mark_vertex_for_deletion(i, check);
				cout << "marked   " << i << "\t ---repl---> " << check << endl;
				break;
			}
		verts.push_back(vdata[i]);
	}

	p.delete_marked_vertices(elem->name);
	
	delete [] vdata;
}

bool duplicate_vertices(ply &p, ply::element_t *elem) {
	std::vector<vec3f> verts;
	verts.reserve(elem->count);
	vec3f *vdata = p.flat_triangle_data("x", "y", "z", [](float f){ return f; });
	bool duplicate_found = false;
	try {
		for (int i = 0; i < elem->count; ++i) {
			for (auto v : verts)
				if (v == vdata[i])
					throw duplicate_found=true;
			verts.push_back(vdata[i]);
		}
	}
	catch (bool) {}
	delete [] vdata;
	return duplicate_found;
}

int main(int argc, char **argv)
{	
	parse_cmdline(argc, argv);

	if (cmdline.input_file_name == "") {
		cerr << "you'll have to specify an input file!" << endl;
		exit(EXIT_FAILURE); 
	}

	ply sphere;
	load_ply_file(cmdline.input_file_name.c_str(), &sphere);
	auto elem = sphere.element("vertex");

	if (duplicate_vertices(sphere, elem)) {
		if (cmdline.clear_duplicates) {
			clear_duplicate_vertices(sphere, elem);
			save_ply_file(cmdline.input_file_name + ".nodup", &sphere);
			cout << "Written duplicate free version of " << cmdline.input_file_name << " to " << cmdline.input_file_name << ".nodup" << endl;
			exit(EXIT_SUCCESS);
		}
		cerr << "Duplicate vertices found, the measurements may be biased by this!" << endl;
		if (!cmdline.ignore_duplicates)
			exit(EXIT_FAILURE);
	}

	int vertices = elem->count;
	vec3f *vertex = new vec3f[vertices];
	float *timings = new float[vertices];
	int x = elem->index_of("x"),
		y = elem->index_of("y"),
		z = elem->index_of("z"),
		m = elem->index_of("ms");
	
	float sum = 0;
	for (int i = 0; i < vertices; ++i) 
		sum += elem->ref(i, m);
	float avg = sum/vertices;

	cout << avg << "ms" << endl;

	return 0;
}


