lib_LTLIBRARIES = librta-bbvh.la

librta_bbvh_la_SOURCES = main.cpp \
					  bbvh.h

librta_bbvh_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)

librta_bbvh_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?


plugin_dir = $(top_builddir)/built-plugins
plugin_name = bbvh.so
plugin = $(plugin_dir)/$(plugin_name)
all-local: $(top_builddir)/built-plugins/bbvh.so

$(plugin): librta-bbvh.la
	cp .libs/librta-bbvh.so $@
