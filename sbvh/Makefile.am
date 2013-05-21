pkglib_LTLIBRARIES = sbvh.la
plugin_name = sbvh.so

sbvh_la_SOURCES = sbvh_mod.cpp \
					  sbvh.h

sbvh_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
sbvh_la_CXXFLAGS = -Wreturn-type

sbvh_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?

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
