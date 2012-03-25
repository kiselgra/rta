lib_LTLIBRARIES = librta.la
noinst_PROGRAMS = rta

#
#  - - - - lib - - - - 
#

# libconfigfiles_la_SOURCES = grammar.yy  lexer.ll  parser.h  token.h lexer.h config_creator.h configfile.h configfile.cpp
librta_la_SOURCES = main.cpp \
					aabb.h \
					image.h \
					cmdline.cpp cmdline.h

# for bison/flex
# AM_CPPFLAGS = -Isrc/
# AM_YFLAGS = -d -p__yacc_prefix__
# AM_LFLAGS = -P__lex_prefix__ -o lex.yy.c

library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)
library_include_HEADERS = 

# setting c++ flags:
# AM_CXXFLAGS = -Wall -ansi -pedantic -ggdb -pg

# setting c preprocessor flags for main project
librta_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\"


#
#  - - - - example - - - - 
#

rta_SOURCES = $(librta_la_SOURCES)
rta_CPPFLAGS = $(librta_la_CPPFLAGS)

