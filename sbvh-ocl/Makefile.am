lib_LTLIBRARIES = librta-sbvh-ocl.la

librta_sbvh_ocl_la_SOURCES = sbvh-ocl-mod.cpp \
					     sbvh-ocl.h

librta_sbvh_ocl_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
librta_sbvh_ocl_la_CXXFLAGS = -Wreturn-type

librta_sbvh_ocl_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?

plugin_dir = $(top_builddir)/built-plugins
plugin_name = sbvh-ocl.so
plugin = $(plugin_dir)/$(plugin_name)
all-local: $(top_builddir)/built-plugins/sbvh-ocl.so

$(plugin): librta-sbvh-ocl.la
	cp .libs/librta-sbvh-ocl.so $@
