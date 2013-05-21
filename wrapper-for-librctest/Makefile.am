pkglib_LTLIBRARIES = librctest.la
plugin_name = librctest.so

librctest_la_SOURCES = librctest.cpp rct-cmdline.cpp rct-cmdline.h

librctest_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
librctest_la_CXXFLAGS = -msse3 -Wreturn-type

librctest_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?
librctest_la_LIBADD = -ltbb -lrc-test -lpthread

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
