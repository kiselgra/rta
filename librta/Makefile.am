lib_LTLIBRARIES = librta.la

librta_la_SOURCES = aabb.h  basic_types.h  image.h  intersect.h  raytrav.h  template-magic.h  tri.h ocl.h ocl.cpp

library_includedir = $(includedir)/$(PACKAGE)-$(VERSION)
library_include_HEADERS = 

# setting c preprocessor flags for main project
librta_la_CPPFLAGS = -D PACKAGE=\"$(PACKAGE)\" -D VERSION=\"$(VERSION)\"
librta_la_CXXFLAGS = -Wreturn-type

