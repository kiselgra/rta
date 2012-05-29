lib_LTLIBRARIES = librta-librctest.la

librta_librctest_la_SOURCES = librctest.cpp rct-cmdline.cpp rct-cmdline.h

librta_librctest_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
librta_librctest_la_CXXFLAGS = -msse3

librta_librctest_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?
librta_librctest_la_LIBADD = -ltbb -lrc-test -lpthread


plugin_dir = $(top_builddir)/built-plugins
plugin_name = librctest.so
plugin = $(plugin_dir)/$(plugin_name)
all-local: $(plugin)

$(plugin): librta-librctest.la
	cp .libs/librta-librctest.so $@
