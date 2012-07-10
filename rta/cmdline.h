/* $Id$ */
#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>
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

	vec3f back_col, light_col;
	std::string model;

	Cmdline() : verbose(false), positional_series(false), axial_series(false), sphere_series(false)
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

