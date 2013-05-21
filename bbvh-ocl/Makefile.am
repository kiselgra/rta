pkglib_LTLIBRARIES = bbvh-ocl.la
plugin_name = bbvh-ocl.so

bbvh_ocl_la_SOURCES = bbvh-ocl-mod.cpp \
					     bbvh-ocl.h

bbvh_ocl_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
bbvh_ocl_la_CXXFLAGS = -Wreturn-type

bbvh_ocl_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?

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
