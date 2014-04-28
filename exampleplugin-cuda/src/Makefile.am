pkglib_LTLIBRARIES = exampleplugin.la
plugin_name = exampleplugin.so

SUFFIXES = .cu

# libconfigfiles_la_SOURCES = grammar.yy  lexer.ll  parser.h  token.h lexer.h config_creator.h configfile.h configfile.cpp
exampleplugin_la_SOURCES = 	main.cpp \
							bruteforce.h \
							cmdline.cpp cmdline.h

#library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)
#library_include_HEADERS = exampleplugin.h

exampleplugin_la_LDFLAGS  = -module -avoid-version -shared $(RTA_LDFLAGS)
exampleplugin_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\" $(RTA_CFLAGS)

.cu.lo:
	$(LIBTOOL) --tag=CXX --mode=compile $(top_srcdir)/nvcc-wrapper $(CXXFLAGS) $(exampleplugin_la_CPPFLAGS) -o $@ $<

$(plugin_name): .libs/$(plugin_name)
	cp $< $@

#plugindir = $(rtaplugindir)
#plugin_DATA = $(plugin_name)

