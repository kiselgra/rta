lib_LTLIBRARIES = librta-bruteforce.la

librta_bruteforce_la_SOURCES = bruteforce.cpp

librta_bruteforce_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" -I$(top_builddir)
librta_bruteforce_la_CXXFLAGS = -Wreturn-type

librta_bruteforce_la_LDFLAGS = -module -avoid-version # do i want "avoid-version"?


plugin_dir = $(top_builddir)/built-plugins
plugin_name = bruteforce.so
plugin = $(plugin_dir)/$(plugin_name)
all-local: $(top_builddir)/built-plugins/bruteforce.so

$(plugin): librta-bruteforce.la
	cp .libs/librta-bruteforce.so $@
