pkglib_LTLIBRARIES = sbvh-ocl.la
plugin_name = sbvh-ocl.so

sbvh_ocl_la_SOURCES = sbvh-ocl-mod.cpp \
					     sbvh-ocl.h

sbvh_ocl_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
sbvh_ocl_la_CXXFLAGS = -Wreturn-type

sbvh_ocl_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?

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
