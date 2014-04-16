/* $Id$ */
#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>
#include <list>
#include <cstdlib>
#include <libmcm/vectors.h>

//! \brief Translated command line options
struct Cmdline
{
	bool verbose;	//!< wheather or not to be verbose

	vec3f pos, dir, up;
	vec3f axis, anchor;
	int samples;

	bool positional_series, axial_series, sphere_series;

	std::string sphere_file;
	std::string force;
	std::string outfile;
	std::string module;
	std::string png_prefix;

	vec3f back_col, light_col;
	std::string model;

	bool produce_images;
	bool binary_intersection_debug;
	float distance_factor;
	bool png_output;

	int res_x, res_y;
	std::list<std::string> image_paths;

	Cmdline() : verbose(false), positional_series(false), axial_series(false), sphere_series(false), png_prefix("/tmp/rta-"), produce_images(true), binary_intersection_debug(false), res_x(800), res_y(800), distance_factor(1.5), png_output(true)
	{
		pos = { 0,0,0 };
		dir = { 0,0,-1 };
		up = { 0,1,0 };
		axis = { 0, 0, 1 };
		anchor = { 1, 0, 0 };
		samples = 1;
		back_col = { 0,0,0 };
		light_col = { 1,1,1 };
		std::string home = std::getenv("HOME");
		model = home + "/render-data/models/drache.obj";
	}
};

extern Cmdline cmdline;

int parse_cmdline(int argc, char **argv);
char** plugin_args(int *n);

#endif

