/* $Id$ */
#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>
#include <list>

//! \brief Translated command line options
struct Cmdline
{
	bool verbose;	//!< wheather or not to be verbose
	std::string input_file_name;
	std::string output_file_name;
	bool print_vertices;
	bool ignore_duplicates, clear_duplicates;

	Cmdline() 
	: verbose(false), print_vertices(false), ignore_duplicates(false), clear_duplicates(false)
	{
	}
};

extern Cmdline cmdline;

int parse_cmdline(int argc, char **argv);

#endif

