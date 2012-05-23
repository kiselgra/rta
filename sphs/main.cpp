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
	int x = elem->index_of("x"),
		y = elem->index_of("y"),
		z = elem->index_of("z");

	for (int i = 0; i < elem->count; ++i) {
		vec3f curr = { elem->ref(i, x), elem->ref(i, y), elem->ref(i, z) };
		for (int check = 0; check < verts.size(); ++check)
			if (verts[check] == curr) {
				p.mark_vertex_for_deletion(i, check);
				cout << "marked   " << i << "\t ---repl---> " << check << endl;
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


