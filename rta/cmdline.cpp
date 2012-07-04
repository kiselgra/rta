#include "cmdline.h"


#include <argp.h>
#include <string>
#include <iostream>
#include <sstream>
#include <dlfcn.h>


using namespace std;

const char *argp_program_version = VERSION;

static char doc[]       = PACKAGE ": ray tracing test suite\n\n"
                          "The specified module is looked up (in built-plugins/) as librta-${name}.so and ${name}.so.";
static char args_doc[]  = "module -- [OPTIONS for module...]";

// long option without corresponding short option have to define a symbolic constant >= 300
enum { FIRST = 300, AXIS, ANCHOR, SF, BT, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,         0, "Be verbose." },
	{ "pos",     'p', "x,y,z",   0, "Camera position."},
	{ "dir",     'd', "x,y,z",   0, "Camera viewing direction."},
	{ "up",      'u', "x,y,z",   0, "Camera up vector."},
	{ "axis",    AXIS,  "x,y,z", 0, "Start axial rotation measure series, see --samples and --anchor."},
	{ "anchor",  ANCHOR, "x,y,z", 0, "Starting position of the axial rotation measure series. Preferably on the unit circle around --axis. Default: 1,0,0." },
	{ "samples", 's', "n",       0, "How many samples to take. Used by --axis, only." },
	{ "sphere-file", SF, "filename.ply", 0, "Start spherical measure series using the points on the sphere specified in the given .ply file. Peferred sphere radius: 1."},
	{ "force-mode", 'f', "[pas]", 0, "When otherwise invalid combinations of --pos, --asix and --sphere_file are given, instead of erroring out, choose the one selected by this flag." },
	{ "outfile", 'o', "filename", 0, "Write pass specific output to this file, e.g. a modified ply file containing timings." },
	{ "module", 'm', "basename",  0, "Use this module." },
	{ "background", 'b', "r,g,b",  0, "Background color for the generated images. Default 0,0,0." },
	{ "light-color", 'l', "r,g,b",  0, "Color of the lights placed around the rendered object. Default 1,1,1" },
	{ "help", '?', 0,             0, "Give this help list (or show help of a previously specified module, see -m)." },
	{ 0 }
};	

string& replace_nl(string &s)
{
	for (int i = 0; i < s.length(); ++i)
		if (s[i] == '\n' || s[i] == '\r')
			s[i] = ' ';
	return s;
}

vec3f read_vec3f(const std::string &s) {
	istringstream iss(s);
	vec3f v;
	char sep;
	iss >> v.x >> sep >> v.y >> sep >> v.z;
	return v;
}

char *uncaught_arg[256];
int uncaught_args = 0;

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
	case '?':	argp_state_help(state, stdout, ARGP_HELP_SHORT_USAGE|ARGP_HELP_PRE_DOC|ARGP_HELP_LONG|ARGP_HELP_POST_DOC); 	exit(EXIT_SUCCESS);
	
	case 'p':     cmdline.pos    = read_vec3f(sarg); cmdline.positional_series = true; break;
	case 'd':     cmdline.dir    = read_vec3f(sarg); break;
	case 'u':     cmdline.up     = read_vec3f(sarg); break;
	case AXIS:    cmdline.axis   = read_vec3f(sarg); cmdline.axial_series = true; break;
	case ANCHOR:  cmdline.anchor = read_vec3f(sarg); break;
	case 's':     cmdline.samples = atoi(arg); break;
	case SF:      cmdline.sphere_file = sarg; cmdline.sphere_series = true; break;
	case 'f':     cmdline.force = sarg; break;
	case 'o':     cmdline.outfile = sarg; break;
	case 'm':     cmdline.module = sarg; break;
	case 'b':     cmdline.back_col  = read_vec3f(sarg); break;
	case 'l':     cmdline.light_col = read_vec3f(sarg); break;

	case ARGP_KEY_ARG:		// process arguments. 
							// state->arg_num gives number of current arg
			  if (state->quoted)
				  uncaught_arg[uncaught_args++] = arg;
			  else if (state->arg_num != 0)
				  return ARGP_ERR_UNKNOWN;
			  else
				  cmdline.module = sarg;
		break;

	default:
		return ARGP_ERR_UNKNOWN;
	}


	return 0;
}

static struct argp parser = { options, parse_options, args_doc, doc };

int parse_cmdline(int argc, char **argv)
{
	uncaught_arg[0] = argv[0];
	uncaught_args=1;
	int ret = argp_parse(&parser, argc, argv, ARGP_NO_HELP|ARGP_IN_ORDER/*ARGP_NO_EXIT*//*0*/, 0, 0);
		
	if (cmdline.force == "p") cmdline.positional_series = true, cmdline.axial_series = false, cmdline.sphere_series = false;
	if (cmdline.force == "a") cmdline.positional_series = false, cmdline.axial_series = true, cmdline.sphere_series = false;
	if (cmdline.force == "s") cmdline.positional_series = false, cmdline.axial_series = false, cmdline.sphere_series = true;
	int p = cmdline.positional_series ? 1 : 0;
	int s = cmdline.sphere_series ? 1 : 0;
	int a = cmdline.axial_series ? 1 : 0;
	if (p+s+a > 1) {
		cerr << "--pos --axis and --sphere-file are exclusive." << endl;
		exit(EXIT_FAILURE);
	}

	return ret;
}

char** plugin_args(int *n) {
	*n = uncaught_args;
	return uncaught_arg;
}

Cmdline cmdline;

