lib_LTLIBRARIES = librta-sbvh.la

librta_sbvh_la_SOURCES = sbvh_mod.cpp \
					  sbvh.h

librta_sbvh_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
librta_sbvh_la_CXXFLAGS = -Wreturn-type

librta_sbvh_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?


plugin_dir = $(top_builddir)/built-plugins
plugin_name = sbvh.so
plugin = $(plugin_dir)/$(plugin_name)
all-local: $(top_builddir)/built-plugins/sbvh.so

$(plugin): librta-sbvh.la
	cp .libs/librta-sbvh.so $@
