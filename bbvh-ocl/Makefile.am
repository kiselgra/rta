lib_LTLIBRARIES = librta-bbvh-ocl.la

librta_bbvh_ocl_la_SOURCES = bbvh-ocl-mod.cpp \
					     bbvh-ocl.h

librta_bbvh_ocl_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
librta_bbvh_ocl_la_CXXFLAGS = -Wreturn-type

librta_bbvh_ocl_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?

plugin_dir = $(top_builddir)/built-plugins
plugin_name = bbvh-ocl.so
plugin = $(plugin_dir)/$(plugin_name)
all-local: $(top_builddir)/built-plugins/bbvh-ocl.so

$(plugin): librta-bbvh-ocl.la
	cp .libs/librta-bbvh-ocl.so $@
