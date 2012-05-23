
bin_PROGRAMS = sphs

# libconfigfiles_la_SOURCES = grammar.yy  lexer.ll  parser.h  token.h lexer.h config_creator.h configfile.h configfile.cpp
sphs_SOURCES = 	main.cpp \
				color.cpp color.h \
				cmdline.cpp cmdline.h

# for bison/flex
# AM_CPPFLAGS = -Isrc/
# AM_YFLAGS = -d -p__yacc_prefix__
# AM_LFLAGS = -P__lex_prefix__ -o lex.yy.c

#library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)
#library_include_HEADERS = objtool.h

# setting c++ flags:
# AM_CXXFLAGS = -Wall -ansi -pedantic -ggdb -pg

# setting c preprocessor flags for main project
sphs_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\"
#objtool_LDADD = @VEC_LIBS@
