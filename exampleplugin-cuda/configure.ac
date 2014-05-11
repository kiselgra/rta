#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ(2.61)
AC_INIT([rta], [0.0.1])
AM_INIT_AUTOMAKE

# name one of your source files here
AC_CONFIG_SRCDIR([src/main.cpp])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_FILES(Makefile src/Makefile)

# Checks for programs.
AC_PROG_CC

AC_PROG_CXX
AC_MSG_CHECKING([for g++ version >= 4.6])
GCC_VER=$($CXX -v 2>&1 | grep "gcc version" | sed -e 's/gcc version \(.\)\.\(.\).*/\1\2/')
GCC_VER_D=$($CXX -v 2>&1 | grep "gcc version" | sed -e 's/gcc version \(.\)\.\(.\).*/\1.\2/')
if test -z "$GCC_VER" ; then
        AC_MSG_ERROR([could not determine gcc version!])
fi
if test "46" -gt "$GCC_VER" ; then
        AC_MSG_ERROR([gcc version must be higher than 4.6! got $GCC_VER_D])
fi
AC_MSG_RESULT([found g++-$GCC_VER_D])
CXXFLAGS="$CXXFLAGS -std=gnu++0x"


AX_ENABLE_GDB_OPTION

# when using flex/bison:
# AC_PROG_LEX
# AC_PROG_YACC

# when building libraries
AC_PROG_LIBTOOL

# Checks for libraries.

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([inttypes.h stdlib.h string.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_CHECK_LIB([cudart], [main])

PKG_CHECK_MODULES(RTA, rta)
AC_SUBST(RTA_CFLAGS)
AC_SUBST(RTA_LDFLAGS)
rtaplugindir="$(pkg-config --variable=plugindir rta)"
AC_SUBST([rtaplugindir])

AC_OUTPUT
