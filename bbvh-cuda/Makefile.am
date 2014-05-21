pkglib_LTLIBRARIES = bbvh-cuda.la
plugin_name = bbvh-cuda.so

bbvh_cuda_la_SOURCES = main.cpp kernels.cu bbvh-cuda.h bbvh-cuda-node.h

bbvh_cuda_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
bbvh_cuda_la_CXXFLAGS = -Wreturn-type -fopenmp

bbvh_cuda_la_LDFLAGS = -shared -module -avoid-version -fopenmp # do i want "avoid-version"?

library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)/bbvh-cuda
library_include_HEADERS = bbvh-cuda.h bbvh-cuda-node.h

local_plugin_dir = $(top_builddir)/built-plugins
plugin = $(local_plugin_dir)/$(plugin_name)
all-local: $(plugin)

.cu.lo:
	$(LIBTOOL) --tag=CXX --mode=compile $(top_srcdir)/nvcc-wrapper $(DEFAULT_INCLUDES) $(CPPFLAGS) $(CXXFLAGS) $(exampleplugin_la_CPPFLAGS) -c -o $@ $< -v

$(plugin): $(pkglib_LTLIBRARIES)
	cp .libs/$(plugin_name) $@

#$(plugin_name):
#	cp .libs/$(plugin_name) $(plugin_name)
#
#plugindir = $(libdir)/$(PACKAGE)-$(VERSION)
#plugin_DATA = $(plugin)
