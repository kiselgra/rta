/* $Id$ */
#ifndef __CMDLINE_H__ 
#define __CMDLINE_H__ 

#include <string>
#include <libmcm/vectors.h>

//! \brief Translated command line options
struct Cmdline
{
	bool verbose;	//!< wheather or not to be verbose

	vec3f pos, dir, up;

	Cmdline() : verbose(false)
	{
		pos = { 0,0,0 };
		dir = { 0,0,-1 };
		up = { 0,1,0 };
	}
};

extern Cmdline cmdline;

int parse_cmdline(int argc, char **argv);

#endif

