pkglib_LTLIBRARIES = bbvh.la
plugin_name = bbvh.so

bbvh_la_SOURCES = main.cpp \
					  bbvh.h

bbvh_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
bbvh_la_CXXFLAGS = -Wreturn-type -fopenmp

bbvh_la_LDFLAGS = -shared -module -avoid-version -fopenmp # do i want "avoid-version"?

library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)/bbvh
library_include_HEADERS = bbvh.h

local_plugin_dir = $(top_builddir)/built-plugins
plugin = $(local_plugin_dir)/$(plugin_name)
all-local: $(plugin)

$(plugin): $(pkglib_LTLIBRARIES)
	cp .libs/$(plugin_name) $@

#$(plugin_name):
#	cp .libs/$(plugin_name) $(plugin_name)
#
#plugindir = $(libdir)/$(PACKAGE)-$(VERSION)
#plugin_DATA = $(plugin)
