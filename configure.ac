#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ(2.61)
AC_INIT([rta], [0.0.1], [((lambda (x y z) (string-append x @ y . z .org)) stuff projects selgrad)])
AM_INIT_AUTOMAKE
AC_CONFIG_MACRO_DIR([m4])

# name one of your source files here
AC_CONFIG_SRCDIR([rta/main.cpp])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_FILES(Makefile librta/Makefile rta/Makefile bbvh/Makefile 
				doc/Makefile doc/doxyfile doc/doxygen_mainpage.h 
				wrapper-for-librctest/Makefile 
				sbvh/Makefile sphs/Makefile bbvh-ocl/Makefile sbvh-ocl/Makefile 
				brute-force/Makefile
				rta.pc)

AX_PREFIX_CONFIG_H(librta/rta-config.h)

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
AC_CHECK_LIB([mcm], [make_vec3f])
AC_CHECK_LIB([png], [main])
AC_CHECK_LIB([objloader], [main])
AC_CHECK_LIB([plyloader], [main])
AC_CHECK_LIB([dl], [main])

# optional, well... to become...
AC_CHECK_LIB([lumocl], [main])
AC_CHECK_LIB([OpenCL], [main])
AC_CHECK_LIB([GL], [main])
#AC_CHECK_LIB([GLblub], [main])
AC_CHECK_LIB([cudart], [main])

AM_CONDITIONAL([opencl], [test "x$ac_cv_lib_lumocl_main" == "xyes"])
AM_CONDITIONAL([use_librctest], [test "x" == "y"])

AM_CONDITIONAL([cuda], [test "x$ac_cv_lib_cudart_main" == "xyes"])

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([inttypes.h stdlib.h string.h unistd.h])
AC_CHECK_HEADERS([png/png.hpp])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_OUTPUT
