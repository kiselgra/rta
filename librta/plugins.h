#ifndef __RTA_PLUGINS_H__ 
#define __RTA_PLUGINS_H__ 

#include <string>
#include "librta.h"

// extern "C" {
// 	char* rta_plugin_description();
// 	void rta_plugin_initialize();
// 	int rta_plugin_parse_cmdline(int argc, char **argv);
// 	rt_set<simple_aabb, simple_triangle> rta_plugin_create_rt_set(flat_triangle_list &triangle_lists, int w, int h);
// }
// 
namespace rta {
	extern char* (*plugin_description)();
	extern int (*plugin_parse_cmdline)(int argc, char **argv);
	extern void (*plugin_initialize)();
	extern rt_set (*plugin_create_rt_set)(basic_flat_triangle_list<simple_triangle>&,int,int);

	void register_plugin_search_path(const std::string &path);
	bool load_plugin_functions(const std::string &plugin_name);
}



#endif

