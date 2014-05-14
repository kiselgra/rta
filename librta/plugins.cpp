#include "plugins.h"

#include <dlfcn.h>
#include <list>

using namespace std;

namespace rta {
	// here we store our function pointers
	char* (*plugin_description)() = 0;
	int (*plugin_parse_cmdline)(int argc, char **argv) = 0;
	void (*plugin_initialize)() = 0;
	rt_set (*plugin_create_rt_set)(flat_triangle_list&,int,int) = 0;

	list<string> lib_search_paths;
	void register_plugin_search_path(const std::string &path) {
		lib_search_paths.push_front(path);
	}

	template<typename T> bool load_plugin_function(const std::string &name, T &to, void *lib_handle) {
		to = (T)dlsym(lib_handle, name.c_str());
		char *error;
		if ((error = dlerror()) != NULL)  {
			fprintf(stderr, "%s\n", error);
			return false;
		}
		return true;
	}
	
	void* find_lib(const std::string &filename) {
		for (string path : lib_search_paths) {
			if (path[path.length()-1] != '/')
				path += "/";
			void *lib_handle = dlopen((path + filename).c_str(), RTLD_LAZY);
			if (lib_handle)
				return lib_handle;
		}
		return 0;
	}

	bool load_plugin_functions(const std::string &plugin_name) {
		if (plugin_name == "") {
			cerr << "no plugin name specified!" << endl;
			return false;
		}
		string so_name, lib_name;
		bool do_rel = true, do_so, do_lib;

		if (plugin_name.substr(plugin_name.length()-3) == ".so")
			so_name = plugin_name, do_so = false;
		else
			so_name = plugin_name + ".so", do_so = true;
		
		if (plugin_name.substr(plugin_name.length(),3) == "lib")
			lib_name = so_name, do_lib = false;
		else
			lib_name = so_name + ".so", do_lib = true;

		if (plugin_name[0] == '/')
			do_rel = do_so = do_lib = false;
			
		void *lib_handle = 0;
		if (do_rel)
			lib_handle = find_lib(plugin_name);
		if (do_so && !lib_handle)
			lib_handle = find_lib(so_name);
		if (do_lib && !lib_handle)
			lib_handle = find_lib(lib_name);
		if (!lib_handle) {
			lib_handle = dlopen(plugin_name.c_str(), RTLD_LAZY);
		}

		if (!lib_handle) {
			cerr << "cannot find plugin '" << plugin_name << "'" << endl;
			cerr << "\ti looked in" << endl;
			for (string p : lib_search_paths)
				cerr << "\t\t- " << p << endl;
			cerr << "error message for absolute path: " << dlerror() << endl;
			return false;
		}

		if (!load_plugin_function("description",   plugin_description,   lib_handle)) return false;
		if (!load_plugin_function("initialize",    plugin_initialize,    lib_handle)) return false;
		if (!load_plugin_function("create_rt_set", plugin_create_rt_set, lib_handle)) return false;
		if (!load_plugin_function("parse_cmdline", plugin_parse_cmdline, lib_handle)) return false;
		plugin_initialize();

		return true;
	}

	struct initial_plugin_directory_registration {
		initial_plugin_directory_registration() {
			register_plugin_search_path(PKG_LIB_DIR);
		}
	};
	static initial_plugin_directory_registration ipdr;
}


/* vim: set foldmethod=marker: */

