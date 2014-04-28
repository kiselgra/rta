struct Cmdline {
	bool verbose;

	Cmdline() : verbose(false) {}
};
extern Cmdline cmdline;

int my_parse_cmdline(int argc, char **argv);
