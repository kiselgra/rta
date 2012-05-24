#include "cmdline.h"


#include <argp.h>
#include <string>
#include <iostream>
#include <cstdlib>



using namespace std;

const char *argp_program_version = VERSION;

static char doc[]       = PACKAGE ": description";
static char args_doc[]  = "inputfile";

// long option without corresponding short option have to define a symbolic constant >= 300
enum { FIRST = 300, IGNORE_DUP, CLEAR_DUP, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,         0, "Be verbose." },
	{ "output", 'o', "filename", 0, "Write output to the given file." },
	{ "vertices", 'N', 0,        0, "Print the number of vertices/faces of the model." },
	{ "ignore-duplicates", IGNORE_DUP, 0, 0, "Force me to ignore duplicate vertices. Note that this option will bias the results!" },
	{ "clear-duplicates", CLEAR_DUP, 0, 0, "Write a file (called $inpufile.nodup) containing no duplicate vertices." },
	{ "mode", 'm', "mode",       0, "Select mode of operation. Valid modes are:\n"
                                    "    minmax: write a version of the file with the vertices colored with indicating the time each sample took, scaled between the minimum (red=slowest) "
											"and the maximum (green=fastest) rays per second any sample took.\n"
									"    absolute: generate vertex colors as for minmax, but the lower bound set to zero, i.e. display the absolute values as ranging from red to green." },
	{ "unit", 'u', "1,k,m",      0, "Make output in terms of rays per second, kilo rps, mega rps. Default: k. " },
	{ 0 }
};	

string& replace_nl(string &s)
{
	for (int i = 0; i < s.length(); ++i)
		if (s[i] == '\n' || s[i] == '\r')
			s[i] = ' ';
	return s;
}


static error_t parse_options(int key, char *arg, argp_state *state)
{
	// call argp_usage to stop program execution if something is wrong
	
	string sarg;
	if (arg)
		sarg = arg;
	sarg = replace_nl(sarg);

	switch (key)
	{
	case 'v':	cmdline.verbose = true; 	break;
	
	case ARGP_KEY_ARG:		// process arguments. 
							// state->arg_num gives number of current arg
		if (state->arg_num > 0)
		{
			cerr << "only one input file allowed." << endl;
			exit(EXIT_FAILURE);
		}
		cmdline.input_file_name = sarg;
		break;

	case 'o':
		cmdline.output_file_name = sarg;
		break;

	case 'N':
		cmdline.print_vertices = true;
		break;

	case 'm':
		if (sarg == "minmax") cmdline.mode = Cmdline::scale_min_max;
		else if (sarg == "absolute") cmdline.mode = Cmdline::scale_0_max;
		else {
			cerr << "invalid mode '" << sarg << "'" << endl;
			argp_usage(state);
		}
		break;

	case 'u':
		if (sarg == "1") cmdline.unit = 1;
		else if (sarg == "k" || sarg == "K") cmdline.unit = 1000;
		else if (sarg == "m" || sarg == "M") cmdline.unit = 1000000;
		else {
			cerr << "Unit not supported (yet?): " << sarg << endl;
			argp_usage(state);
		}
		break;

	case IGNORE_DUP:
		cmdline.ignore_duplicates = true;
		break;

	case CLEAR_DUP:
		cmdline.clear_duplicates = true;
		break;

	default:
		return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

static struct argp parser = { options, parse_options, args_doc, doc };

int parse_cmdline(int argc, char **argv)
{
	int ret = argp_parse(&parser, argc, argv, /*ARGP_NO_EXIT*/0, 0, 0);
	return ret;
}
	
Cmdline cmdline;

