#include "cmdline.h"

#include <argp.h>
#include <string>

using namespace std;

Cmdline cmdline;

const char *argp_program_version = VERSION;

static char plugin_name[] = "exampleplugin/bruteforce";
static char doc[]       = "exampleplugin/bruteforce: an example rta plugin.";
static char args_doc[]  = "";

enum { FIRST = 300, BT, OPTS };

static struct argp_option options[] = 
{
	// --[opt]		short/const		arg-descr		?		option-descr
	{ "verbose", 'v', 0,          0, "Be verbose." },	// note: verbose would/should be available via rta, too.
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
				  return ARGP_ERR_UNKNOWN;
	default:
		return ARGP_ERR_UNKNOWN;
	}

	return 0;
}

static struct argp parser = { options, parse_options, args_doc, doc };


int my_parse_cmdline(int argc, char **argv)
{
	argv[0] = plugin_name;
	int ret = argp_parse(&parser, argc, argv, 0, 0, 0);
	return ret;
}


