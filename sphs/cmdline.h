/* $Id$ */
#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>
#include <list>
#include <libmcm/vectors.h>

//! \brief Translated command line options
struct Cmdline
{
	enum mode_t { none, check, scale_min_max, scale_0_max, perf_diff_by_base, mask };

	bool verbose;	//!< wheather or not to be verbose
	std::string base_file_name, diff_file_name;
	std::string output_file_name;
	bool print_vertices;
	bool ignore_duplicates, clear_duplicates;
	mode_t mode;
	float unit;
	std::string unit_string;
	bool force;
	int meta_differences;
	bool bump;
	std::list<vec3f> masked_octants;

	Cmdline() 
	: verbose(false), print_vertices(false), ignore_duplicates(false), clear_duplicates(false), mode(none), unit(1000), unit_string("k"), force(false), meta_differences(1), bump(false)
	{
	}
};

extern Cmdline cmdline;

int parse_cmdline(int argc, char **argv);

#endif

