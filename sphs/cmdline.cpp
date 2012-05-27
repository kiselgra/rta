#include "cmdline.h"


#include <argp.h>
#include <string>
#include <iostream>
#include <cstdlib>



using namespace std;

const char *argp_program_version = VERSION;

static char doc[]       = PACKAGE ": Visualize performance measurements taken on a sphere. When two input files are given: visualize the performance of the second relative to the first.";
static char args_doc[]  = "basefile [second]";

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
	                                "    none: the default mode is to raise an error (in the hope that it helps more than it is a nuisance).\n"
	                                "    check: just check the input data and return 0 if everything is fine.\n"
                                    "    minmax: write a version of the file with the vertices colored with indicating the time each sample took, scaled between the minimum (red=slowest) "
											"and the maximum (green=fastest) rays per second any sample took.\n"
									"    absolute: generate vertex colors as for minmax, but the lower bound set to zero, i.e. display the absolute values as ranging from red to green."
// 	                                "    relative-to: show performance relative to the specified reference value (see --reference).\n"
	                                "    perf: show the performance of the second file relative to the first (b/a).\n"
									},
	{ "unit", 'u', "1,k,m",      0, "Make output in terms of rays per second, kilo rps, mega rps. Default: k. " },
	{ "force", 'f', 0,           0, "Force the comparison, disregarding meta data differences." },
	{ "max-differences", 'd', "n", 0, "How many different values in the rta meta data are allowed to still compare two files. Default: 1." },
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
		if (state->arg_num > 1)
		{
			cerr << "a maximum of two input files are allowed." << endl;
			argp_usage(state);
		}
		if (state->arg_num == 0) cmdline.base_file_name = sarg;
		if (state->arg_num == 1) cmdline.diff_file_name = sarg;
		break;

	case 'o':
		cmdline.output_file_name = sarg;
		break;

	case 'N':
		cmdline.print_vertices = true;
		break;

	case 'm':
		if      (sarg == "check")    cmdline.mode = Cmdline::check;
		else if (sarg == "minmax")   cmdline.mode = Cmdline::scale_min_max;
		else if (sarg == "absolute") cmdline.mode = Cmdline::scale_0_max;
		else if (sarg == "perf")     cmdline.mode = Cmdline::perf_diff_by_base;
		else {
			cerr << "invalid mode '" << sarg << "'" << endl;
			argp_usage(state);
		}
		break;

	case 'u':
		if (sarg == "1") cmdline.unit = 1, cmdline.unit_string = "";
		else if (sarg == "k" || sarg == "K") cmdline.unit = 1000, cmdline.unit_string = "k";
		else if (sarg == "m" || sarg == "M") cmdline.unit = 1000000, cmdline.unit_string = "m";
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

	case 'f':
		cmdline.force = true;
		break;

	case 'd':
		cmdline.meta_differences = atoi(arg);
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

