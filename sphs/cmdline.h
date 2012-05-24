/* $Id$ */
#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>
#include <list>

//! \brief Translated command line options
struct Cmdline
{
	enum mode_t { none, scale_min_max, scale_0_max };

	bool verbose;	//!< wheather or not to be verbose
	std::string input_file_name;
	std::string output_file_name;
	bool print_vertices;
	bool ignore_duplicates, clear_duplicates;
	mode_t mode;
	float unit;

	Cmdline() 
	: verbose(false), print_vertices(false), ignore_duplicates(false), clear_duplicates(false), mode(none), unit(1000)
	{
	}
};

extern Cmdline cmdline;

int parse_cmdline(int argc, char **argv);

#endif

